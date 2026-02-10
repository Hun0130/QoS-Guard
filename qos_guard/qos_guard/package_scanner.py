#!/usr/bin/env python3
"""
ROS 2 package scanner.

Scan package path for XML files and source code (.cpp, .hpp, .h, .py).
Extract entities with topic-based 1:N, N:1 topology matching.
Code QoS overrides XML (Effective QoS).
"""
import os
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from . import xml_parser
from . import code_scanner
from . import topic_extractor
from . import node_resolver
from .xml_parser import EntityBlock

# profile_name에서 base 추출 (매칭용)
_BASE_EXTRACT_RE = re.compile(
    r"(?:_?(?:pub(?:lisher)?|sub(?:scriber)?|data_?writer|data_?reader"
    r"|writer|reader|writer_profile|reader_profile|pub_profile|sub_profile"
    r"|publisher_profile|subscriber_profile|profile))+$",
    re.I,
)
_WILDCARD_KEYWORDS = ("default", "common", "generic")


def qos_entity_to_block(entity: "QosEntity") -> EntityBlock:
    """QosEntity를 resolve_profile_for_block용 EntityBlock으로 변환."""
    tag = entity.entity_tag or (
        "publisher" if entity.entity_type == "pub" else "subscriber"
    )
    return EntityBlock(
        tag=tag,
        profile_name=entity.profile_name,
        topic_name=entity.topic_name,
        block_xml=entity.block_xml,
        is_default=entity.is_default,
        base_profile_name=entity.base_profile_name,
    )


def get_fastrtps_env_xml_paths() -> list[Path]:
    """
    FASTRTPS_DEFAULT_PROFILES_FILE, RMW_FASTRTPS_CONFIG_FILE 환경변수로 지정된
    외부 XML 파일 경로 목록 반환.
    """
    paths: list[Path] = []
    seen: set[Path] = set()
    for env_name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "RMW_FASTRTPS_CONFIG_FILE"):
        val = os.environ.get(env_name)
        if not val or not str(val).strip():
            continue
        p = Path(val.strip()).resolve()
        if p.is_file() and p not in seen:
            paths.append(p)
            seen.add(p)
    return paths


def find_all_xml_files(package_path: Path, dds: str = "fast") -> list[Path]:
    """
    Recursively find all *.xml files under package path, excluding package.xml.
    dds='fast'일 때 FASTRTPS_DEFAULT_PROFILES_FILE/RMW_FASTRTPS_CONFIG_FILE 환경변수
    XML 경로도 포함.
    """
    package_path = Path(package_path).resolve()
    result: list[Path] = []

    if package_path.is_dir():
        result.extend(
            sorted(
                p for p in package_path.rglob("*.xml")
                if p.is_file() and p.name != "package.xml"
            )
        )

    if dds == "fast":
        for p in get_fastrtps_env_xml_paths():
            if p not in result:
                result.append(p)

    return sorted(result)


@dataclass
class QosEntity:
    """단일 QoS 엔티티 (publisher/subscriber/data_writer/data_reader)."""

    source_path: Path
    entity_type: Literal["pub", "sub"]
    profile_name: str
    topic_name: str | None
    block_xml: str
    full_file_content: str
    code_qos: dict[str, str] | None = None  # 코드에서 추출한 QoS (L1 최우선)
    entity_tag: str = ""  # publisher | subscriber | data_writer | data_reader
    is_default: bool = False
    base_profile_name: str | None = None  # Fast DDS base_profile_name 상속
    node_name: str = ""  # ros2 run 실행 단위(executable). 없으면 A,B,C.. fallback


@dataclass
class XmlPair:
    """pub/sub XML 쌍 (하위 호환용)."""

    pub_path: Path
    sub_path: Path
    pub_xml: str
    sub_xml: str


def _profile_as_node_name(profile_name: str) -> str:
    """profile_name이 executable 형식이면 그대로 사용 (cmd_vel_pub 등)."""
    if not profile_name or profile_name.startswith("/"):
        return ""
    s = profile_name.strip()
    if s and s.replace("_", "").replace("-", "").isalnum():
        return s
    return ""


def scan_package(package_path: Path, dds: str = "fast") -> list[QosEntity]:
    """Scan package for QoS XML and source code, return entity list."""
    package_path = Path(package_path).resolve()
    if not package_path.is_dir():
        return []

    source_to_exec = node_resolver.get_source_to_executable_map(package_path)
    entities: list[QosEntity] = []

    # 1) XML 엔티티
    for xml_path in find_all_xml_files(package_path, dds=dds):
        try:
            content = xml_path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue

        try:
            blocks = xml_parser.extract_all_entity_blocks(content, dds=dds)
        except (ValueError, AttributeError):
            continue

        for block in blocks:
            if block.tag == "topic":
                continue
            if block.tag in ("publisher", "data_writer"):
                entity_type: Literal["pub", "sub"] = "pub"
            else:
                entity_type = "sub"

            node = _profile_as_node_name(block.profile_name)
            entities.append(
                QosEntity(
                    source_path=xml_path,
                    entity_type=entity_type,
                    profile_name=block.profile_name,
                    topic_name=block.topic_name,
                    block_xml=block.block_xml,
                    full_file_content=content,
                    code_qos=None,
                    entity_tag=block.tag,
                    is_default=block.is_default,
                    base_profile_name=block.base_profile_name,
                    node_name=node,
                )
            )

    # 2) 코드 엔티티 스캔 및 병합
    # 동일 (topic, role)에 여러 XML 엔티티가 있을 수 있음 (1:N, N:1)
    # → 모든 해당 XML 엔티티에 code_qos 적용
    code_entities = code_scanner.scan_code(package_path)
    topic_role_to_xml_list: dict[tuple[str, str], list[QosEntity]] = defaultdict(list)
    for e in entities:
        if e.topic_name:
            key = (_normalize_topic(e.topic_name), e.entity_type)
            topic_role_to_xml_list[key].append(e)

    for ce in code_entities:
        key = (_normalize_topic(ce.topic_name), ce.entity_type)
        code_node = source_to_exec.get(ce.source_path.resolve(), "")
        if key in topic_role_to_xml_list:
            for xml_entity in topic_role_to_xml_list[key]:
                xml_entity.code_qos = ce.qos_from_code
                if code_node:
                    xml_entity.node_name = code_node  # executable > profile_name
        else:
            entities.append(
                QosEntity(
                    source_path=ce.source_path,
                    entity_type=ce.entity_type,
                    profile_name="",
                    topic_name=ce.topic_name,
                    block_xml="",
                    full_file_content="",
                    code_qos=ce.qos_from_code,
                    node_name=code_node,
                )
            )

    # 3) topic_extractor: Parameter/Constant 토픽 -> XML topic profile 매칭용 entity 보강
    extracted = topic_extractor.extract_topics(package_path)
    for topic, info in extracted.items():
        norm = _normalize_topic(topic)
        if not norm:
            continue
        if info.get("source_type") == "Literal":
            continue  # 이미 code_scanner에서 처리됨
        has_pub = any(
            _normalize_topic(e.topic_name) == norm and e.entity_type == "pub"
            for e in entities
        )
        has_sub = any(
            _normalize_topic(e.topic_name) == norm and e.entity_type == "sub"
            for e in entities
        )
        if has_pub and has_sub:
            continue
        qos = info.get("detected_qos") or {}
        code_qos = {k: v for k, v in qos.items() if v} if qos else None
        src = Path(info["file_path"])
        ext_node = source_to_exec.get(src.resolve(), "")

        if not has_pub:
            entities.append(
                QosEntity(
                    source_path=src,
                    entity_type="pub",
                    profile_name="",
                    topic_name=topic,
                    block_xml="",
                    full_file_content="",
                    code_qos=code_qos,
                    node_name=ext_node,
                )
            )
        if not has_sub:
            entities.append(
                QosEntity(
                    source_path=src,
                    entity_type="sub",
                    profile_name="",
                    topic_name=topic,
                    block_xml="",
                    full_file_content="",
                    code_qos=code_qos,
                    node_name=ext_node,
                )
            )

    # 4) 소스코드로 엔티티화 되지 않은 XML 프로파일 제외
    # 유지: 코드/topic_extractor 전용, code와 merged, topic에 code 엔티티 있음
    # 제외: topic_name 없음 또는 topic에 code 엔티티 없음
    topics_with_code: set[str] = set()
    for e in entities:
        if e.code_qos and e.topic_name:
            topics_with_code.add(_normalize_topic(e.topic_name))

    entities = [
        e for e in entities
        if not e.block_xml  # 코드/topic_extractor 전용 → 유지
        or e.code_qos is not None  # code와 merged → 유지
        or (e.topic_name and _normalize_topic(e.topic_name) in topics_with_code)
    ]

    node_resolver.assign_fallback_nodes(entities)
    return entities


def _normalize_topic(name: str | None) -> str:
    """Normalize topic name: /cmd_vel, cmd_vel -> cmd_vel."""
    return (name or "").strip().strip("/") or ""


def _profile_base_name(profile_name: str) -> str:
    """
    Extract matching base from profile_name.

    cmd_vel_pub, cmd_vel_subscriber_profile -> cmd_vel.
    """
    base = (profile_name or "").strip()
    while True:
        new_base = _BASE_EXTRACT_RE.sub("", base)
        if new_base == base:
            break
        base = new_base.rstrip("_")
    return base


def _is_wildcard_base(base: str | None) -> bool:
    """Return True if base contains default, common, or generic for match-all."""
    if not base:
        return False
    lower = base.lower()
    return any(kw in lower for kw in _WILDCARD_KEYWORDS)


def build_entity_pairs(entities: list[QosEntity]) -> list[tuple[QosEntity, QosEntity]]:
    """
    토픽 이름 기준으로 pub/sub 쌍을 생성합니다.

    1. 토픽 기반: 동일 토픽의 모든 Pub ↔ Sub 조합 (1:N, N:1 지원)
    2. 익명 프로파일: topic_name 없을 때
       - profile base 매칭: cmd_vel_pub ↔ cmd_vel_subscriber_profile
       - wildcard(default/common/generic): 전체 조합
       - 폴백: 전체 매칭(Broad Match)
    """
    pubs = [e for e in entities if e.entity_type == "pub"]
    subs = [e for e in entities if e.entity_type == "sub"]
    pairs: list[tuple[QosEntity, QosEntity]] = []

    # 1) 토픽 이름이 있는 엔티티: 토픽별 그룹화
    pubs_by_topic: dict[str, list[QosEntity]] = defaultdict(list)
    subs_by_topic: dict[str, list[QosEntity]] = defaultdict(list)

    def _valid_topic_key(key: str) -> bool:
        k = (key or "").strip().strip("/")
        return len(k) >= 2 and k not in ("[]", ")") and "[" not in k

    for pub in pubs:
        key = _normalize_topic(pub.topic_name) if pub.topic_name else None
        if key is not None and _valid_topic_key(key):
            pubs_by_topic[key].append(pub)

    for sub in subs:
        key = _normalize_topic(sub.topic_name) if sub.topic_name else None
        if key is not None and _valid_topic_key(key):
            subs_by_topic[key].append(sub)

    # 동일 토픽의 모든 Pub ↔ Sub 조합 (1:N, N:1) — 매칭만, synthetic 없음
    for topic_key in pubs_by_topic:
        if topic_key in subs_by_topic:
            for pub in pubs_by_topic[topic_key]:
                for sub in subs_by_topic[topic_key]:
                    pairs.append((pub, sub))

    # 2) 익명 프로파일 (topic_name 없음)
    anon_pubs = [p for p in pubs if not p.topic_name]
    anon_subs = [s for s in subs if not s.topic_name]

    if not anon_pubs or not anon_subs:
        return pairs

    # base 이름 추출
    pub_bases = [(p, _profile_base_name(p.profile_name) or None) for p in anon_pubs]
    sub_bases = [(s, _profile_base_name(s.profile_name) or None) for s in anon_subs]

    has_wildcard = any(
        _is_wildcard_base(b) for _, b in pub_bases + sub_bases if b
    )

    if has_wildcard:
        for pub in anon_pubs:
            for sub in anon_subs:
                pairs.append((pub, sub))
    else:
        # base 매칭
        matched = False
        for pub, pub_base in pub_bases:
            for sub, sub_base in sub_bases:
                if pub_base and sub_base and pub_base.lower() == sub_base.lower():
                    pairs.append((pub, sub))
                    matched = True

        if not matched:
            for pub in anon_pubs:
                for sub in anon_subs:
                    pairs.append((pub, sub))

    return pairs


def get_orphan_topics(entities: list[QosEntity]) -> list[tuple[str, Literal["pub", "sub"], QosEntity]]:
    """
    Pub-only 또는 Sub-only 토픽 리스트. (topic_key, side, entity)
    매칭 없이 표시용, 룰 검사 건너뜀.
    """
    pubs = [e for e in entities if e.entity_type == "pub"]
    subs = [e for e in entities if e.entity_type == "sub"]
    pubs_by_topic: dict[str, list[QosEntity]] = defaultdict(list)
    subs_by_topic: dict[str, list[QosEntity]] = defaultdict(list)

    def _valid(key: str) -> bool:
        k = (key or "").strip().strip("/")
        return len(k) >= 2 and k not in ("[]", ")") and "[" not in k

    for pub in pubs:
        key = _normalize_topic(pub.topic_name) if pub.topic_name else None
        if key and _valid(key):
            pubs_by_topic[key].append(pub)
    for sub in subs:
        key = _normalize_topic(sub.topic_name) if sub.topic_name else None
        if key and _valid(key):
            subs_by_topic[key].append(sub)

    orphans: list[tuple[str, Literal["pub", "sub"], QosEntity]] = []
    for topic_key in pubs_by_topic:
        if topic_key not in subs_by_topic:
            topic_name = f"/{topic_key}" if not topic_key.startswith("/") else topic_key
            orphans.append((topic_name, "pub", pubs_by_topic[topic_key][0]))
    for topic_key in subs_by_topic:
        if topic_key not in pubs_by_topic:
            topic_name = f"/{topic_key}" if not topic_key.startswith("/") else topic_key
            orphans.append((topic_name, "sub", subs_by_topic[topic_key][0]))
    return orphans
