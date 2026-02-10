#!/usr/bin/env python3
"""
QoS Guard - DDS QoS 프로파일 검증 도구.

CLI 인자 파싱, XML 파싱, 규칙 검사, 출력을 각 모듈에 위임합니다.
"""
import sys
from pathlib import Path

from . import cli
from . import output
from . import package_scanner
from . import rules_fastdds_humble as rules
from . import xml_parser


def _is_code_non_system_default(code_qos: dict | None) -> bool:
    """코드 QoS가 SYSTEM_DEFAULT가 아닌 명시적 값이 있는지 확인 (L1 적용 조건)."""
    if not code_qos:
        return False
    for k, v in code_qos.items():
        if v and str(v).strip():
            return True
    return False


def _apply_code_qos_override(
    qos: dict,
    code_qos: dict | None,
    source_info: dict,
) -> tuple[dict, dict]:
    """
    L1 (Code): 코드 QoS가 SYSTEM_DEFAULT가 아니면 Full Override 적용.

    Returns (merged_qos, updated_source_info).
    """
    if not _is_code_non_system_default(code_qos):
        return qos, source_info
    out = dict(qos)
    src = dict(source_info)
    for k, v in code_qos.items():
        if v and str(v).strip():
            keys_upper = ("reliability", "durability", "history")
            out[k] = str(v).strip().upper() if k in keys_upper else str(v)
            src[k] = "Level_1_Code"
    return out, src


def _run_checks_for_pair(
    pair_xml: tuple[str, str], ctx: rules.CheckContext
) -> list[tuple[str, str, str]]:
    """Run checks for a single pub/sub pair (XML pair mode)."""
    pub_xml, sub_xml = pair_xml
    pub_q, pub_entity_xml, pub_src, pub_logs = xml_parser.resolve_profile(
        pub_xml, "pub", dds=ctx.dds
    )
    sub_q, sub_entity_xml, sub_src, sub_logs = xml_parser.resolve_profile(
        sub_xml, "sub", dds=ctx.dds
    )
    pub_q = _apply_qos_defaults(pub_q, dds=ctx.dds)
    sub_q = _apply_qos_defaults(sub_q, dds=ctx.dds)
    return rules.run_checks(pub_entity_xml, sub_entity_xml, pub_q, sub_q, ctx)


_QOS_DEFAULTS = {
    "reliability": "",
    "durability": "",
    "history": "KEEP_LAST",
    "history_depth": "10",
    "max_samples": "",
    "max_instances": "",
    "max_samples_per_instance": "",
    "ownership": "",
    "dest_order": "",
    "autodispose": "",
    "autoenable": "",
    "liveliness": "",
    "nowriter_sec_r": "",
    "nowriter_nsec_r": "",
    "autopurge_disposed_samples_delay": "",
    "userdata": "",
    "partition_list": [""],
}

# RMW 벤더별 ResourceLimits 기본값 (코드/XML 미지정 시 적용)
# ref: Fast DDS default, Cyclone DDS default
_RMW_RESOURCE_DEFAULTS = {
    "fast": {
        "max_samples": "5000",
        "max_instances": "10",
        "max_samples_per_instance": "400",
    },
    "cyclone": {
        "max_samples": "-1",
        "max_instances": "-1",
        "max_samples_per_instance": "-1",
    },
    "connext": {
        "max_samples": "5000",
        "max_instances": "10",
        "max_samples_per_instance": "400",
    },
}


def _apply_qos_defaults(qos: dict, dds: str = "fast") -> dict:
    """
    규칙 검사용 필수 키가 모두 존재하도록 기본값 병합.

    ResourceLimits가 비어있으면 RMW 벤더 기본값 적용 (False Positive 방지).
    """
    out = dict(_QOS_DEFAULTS)
    out.update({k: v for k, v in qos.items() if k in out})
    if "partition_list" in qos:
        out["partition_list"] = qos["partition_list"]
    # 비어있는 resource limit에 RMW 기본값 적용
    vendor = _RMW_RESOURCE_DEFAULTS.get(dds, _RMW_RESOURCE_DEFAULTS["fast"])
    for key in ("max_samples", "max_instances", "max_samples_per_instance"):
        if not (out.get(key) and str(out[key]).strip()):
            out[key] = vendor[key]
    return out


def _code_qos_to_full_dict(code_qos: dict[str, str], dds: str = "fast") -> dict:
    """코드 QoS를 규칙 검사용 전체 dict로 변환 (기본값 포함)."""
    out = dict(_QOS_DEFAULTS)
    for k, v in code_qos.items():
        if v and str(v).strip():
            keys = ("reliability", "durability", "history")
            out[k] = str(v).strip().upper() if k in keys else str(v)
    vendor = _RMW_RESOURCE_DEFAULTS.get(dds, _RMW_RESOURCE_DEFAULTS["fast"])
    for key in ("max_samples", "max_instances", "max_samples_per_instance"):
        if not (out.get(key) and str(out[key]).strip()):
            out[key] = vendor[key]
    return out


def _load_combined_package_xml(package_path: Path, dds: str = "fast") -> str:
    """Load and concatenate all XML files in package for profile resolution.
    dds='fast'일 때 FASTRTPS_DEFAULT_PROFILES_FILE 등 환경변수 XML도 포함."""
    xml_files = package_scanner.find_all_xml_files(package_path, dds=dds)
    parts: list[str] = []
    for p in xml_files:
        try:
            parts.append(p.read_text(encoding="utf-8", errors="ignore"))
        except OSError:
            continue
    return "\n".join(parts)


def _resolve_single_entity(
    entity: package_scanner.QosEntity,
    combined_xml: str,
    dds: str = "fast",
) -> tuple[dict, str]:
    """단일 엔티티의 QoS와 entity_xml resolve. (q, entity_xml) 반환."""
    if entity.block_xml:
        block = package_scanner.qos_entity_to_block(entity)
        q, entity_xml, _src, _logs = xml_parser.resolve_profile_for_block(
            combined_xml, block, dds=dds
        )
    else:
        topic_profile_qos = xml_parser.get_topic_profile_qos(
            combined_xml, entity.topic_name, dds=dds
        )
        base = dict(topic_profile_qos) if topic_profile_qos else {}
        base.update({
            k: v for k, v in (entity.code_qos or {}).items()
            if v and str(v).strip()
        })
        q = _apply_qos_defaults(base, dds=dds)
        entity_xml = ""
    q, _ = _apply_code_qos_override(q, entity.code_qos, {})
    q = _apply_qos_defaults(q, dds=dds)
    return q, entity_xml


def _run_checks_for_entity_pair(
    pub_entity: package_scanner.QosEntity,
    sub_entity: package_scanner.QosEntity,
    ctx: rules.CheckContext,
    combined_xml: str,
    info_seen_topics: set[str] | None = None,
) -> list[tuple[str, str, str]]:
    """엔티티 쌍에 대해 검사를 실행합니다. (패키지 모드용). L1 Code > XML."""
    pub_src: dict = {}
    sub_src: dict = {}

    pub_logs: list[str] = []
    sub_logs: list[str] = []
    if pub_entity.block_xml:
        pub_block = package_scanner.qos_entity_to_block(pub_entity)
        pub_q, pub_entity_xml, pub_src, pub_logs = xml_parser.resolve_profile_for_block(
            combined_xml, pub_block, dds=ctx.dds
        )
    else:
        # 코드 전용: XML topic profile(L2) 매칭 시도 후 code_qos(L1) 적용
        topic_profile_qos = xml_parser.get_topic_profile_qos(
            combined_xml, pub_entity.topic_name, dds=ctx.dds
        )
        base = dict(topic_profile_qos) if topic_profile_qos else {}
        base.update({k: v for k, v in (pub_entity.code_qos or {}).items() if v and str(v).strip()})
        pub_q = _apply_qos_defaults(base, dds=ctx.dds)
        pub_entity_xml = ""

    if sub_entity.block_xml:
        sub_block = package_scanner.qos_entity_to_block(sub_entity)
        sub_q, sub_entity_xml, sub_src, sub_logs = xml_parser.resolve_profile_for_block(
            combined_xml, sub_block, dds=ctx.dds
        )
    else:
        topic_profile_qos = xml_parser.get_topic_profile_qos(
            combined_xml, sub_entity.topic_name, dds=ctx.dds
        )
        base = dict(topic_profile_qos) if topic_profile_qos else {}
        base.update({k: v for k, v in (sub_entity.code_qos or {}).items() if v and str(v).strip()})
        sub_q = _apply_qos_defaults(base, dds=ctx.dds)
        sub_entity_xml = ""

    # L1 (Code): SYSTEM_DEFAULT가 아니면 Full Override
    pub_q, pub_src = _apply_code_qos_override(pub_q, pub_entity.code_qos, pub_src)
    sub_q, sub_src = _apply_code_qos_override(sub_q, sub_entity.code_qos, sub_src)

    # 출처 시각화 (버퍼에 수집, 토픽별 1회만)
    pub_had_override = bool(pub_logs)
    sub_had_override = bool(sub_logs)
    _log_qos_sources(
        pub_entity, sub_entity, pub_src, sub_src,
        pub_had_override, sub_had_override,
        info_seen_topics=info_seen_topics,
    )

    pub_q = _apply_qos_defaults(pub_q, dds=ctx.dds)
    sub_q = _apply_qos_defaults(sub_q, dds=ctx.dds)
    return rules.run_checks(pub_entity_xml, sub_entity_xml, pub_q, sub_q, ctx)


def _log_qos_sources(
    pub_entity: package_scanner.QosEntity,
    sub_entity: package_scanner.QosEntity,
    pub_src: dict,
    sub_src: dict,
    pub_had_override: bool,
    sub_had_override: bool,
    info_seen_topics: set[str] | None = None,
) -> None:
    """Output policy source (Level) to 버퍼. 토픽별 1회만."""
    def _summarize(src: dict) -> str:
        if not src:
            return "Default"
        levels = sorted(set(src.values()))
        return ", ".join(levels)
    pub_sum = _summarize(pub_src)
    sub_sum = _summarize(sub_src)
    topic = pub_entity.topic_name or sub_entity.topic_name or "?"
    if pub_sum != "Default" or sub_sum != "Default":
        if info_seen_topics is not None:
            norm = (topic or "").strip().strip("/") or "?"
            if norm in info_seen_topics:
                return
            info_seen_topics.add(norm)
        output.buffer_qos_sources(
            topic, pub_sum, sub_sum,
            pub_had_override, sub_had_override,
            pub_entity.entity_tag, sub_entity.entity_tag,
            pub_entity.node_name or "", sub_entity.node_name or "",
        )


def main() -> None:
    """메인 진입점."""
    args = cli.parse_args(sys.argv)

    # list mode: list all XML files in package
    if args.mode == "list":
        xml_files = package_scanner.find_all_xml_files(args.package_path)
        output.print_list_mode(xml_files, args.package_path)
        return

    ctx = rules.CheckContext(
        publish_period_ms=args.publish_period_ms,
        rtt_ns=args.rtt_ns,
        dds=args.dds,
        ros_version=args.ros_version,
    )

    if args.dds != "fast" or args.ros_version != "humble":
        output.print_warn(
            f"Currently only fast+humble rules are implemented. "
            f"Proceeding with dds={args.dds}, ros_version={args.ros_version}."
        )

    # Cyclone DDS: XML QoS profiles not supported -> reject XML pair mode
    if args.mode == "xml_pair" and args.dds == "cyclone":
        output.print_cyclone_unsupported()
        sys.exit(0)

    all_warnings: list[tuple[str, str, str | None, str, str, str]] = []
    package_name = ""
    node_count = 0
    topic_count = 0
    endpoint_count = 0

    if args.mode == "xml_pair":
        pub_xml = cli.load_text(args.pub_path)
        sub_xml = cli.load_text(args.sub_path)
        warnings = _run_checks_for_pair((pub_xml, sub_xml), ctx)
        for sev, side, msg in warnings:
            all_warnings.append((sev, side, None, msg, "", ""))
        package_name = "-"
        node_count = 0
        topic_count = 1
        endpoint_count = 2

    else:
        entities = package_scanner.scan_package(args.package_path, dds=ctx.dds)
        pairs = package_scanner.build_entity_pairs(entities)
        orphans = package_scanner.get_orphan_topics(entities)

        package_name = args.package_path.name if args.package_path else "-"
        node_count = len({(e.node_name or e.profile_name or "").strip() for e in entities if (e.node_name or e.profile_name or "").strip()})
        topic_count = len({(pub.topic_name or sub.topic_name or "?").strip().strip("/") for pub, sub in pairs} | {t.strip().strip("/") for t, _, _ in orphans})
        endpoint_count = len(entities)

        if not pairs and not orphans:
            output.print_no_entity_pairs(args.package_path)
            return

        combined_xml = _load_combined_package_xml(args.package_path, dds=ctx.dds)
        for topic, side, entity in orphans:
            level = "L1" if not (entity.block_xml or "").strip() else "L2"
            output.buffer_orphan_topic(topic, side, entity.node_name or "", level=level)
            # Stage 1: orphan에도 자기 자신만 검사하는 단일 룰 적용
            entity_q, entity_xml = _resolve_single_entity(entity, combined_xml, dds=ctx.dds)
            orphan_warnings = rules.run_checks_single_entity(
                entity_xml, entity_q, ctx, side.upper()
            )
            for sev, warn_side, msg in orphan_warnings:
                all_warnings.append((
                    sev, warn_side, topic, msg,
                    entity.node_name or "", "",
                ))
        seen: set[tuple[str, str, str]] = set()
        info_seen: set[str] = set()
        for pub_entity, sub_entity in pairs:
            topic = pub_entity.topic_name or sub_entity.topic_name or "?"
            warnings = _run_checks_for_entity_pair(
                pub_entity, sub_entity, ctx, combined_xml,
                info_seen_topics=info_seen,
            )
            for sev, side, msg in warnings:
                key = (topic or "", sev, msg)
                if key in seen:
                    continue
                seen.add(key)
                all_warnings.append((
                    sev, side, topic, msg,
                    pub_entity.node_name or "", sub_entity.node_name or "",
                ))

    if all_warnings:
        for row in all_warnings:
            sev, side, topic, msg = row[0], row[1], row[2], row[3]
            pub_node = row[4] if len(row) > 4 else ""
            sub_node = row[5] if len(row) > 5 else ""
            output.buffer_warning(pub_node, sub_node, sev, side, topic, msg)
    output.flush_output_buffer()

    struct = sum(1 for r in all_warnings if (r[0] or "").lower() == "structural")
    func = sum(1 for r in all_warnings if (r[0] or "").lower() == "functional")
    oper = sum(1 for r in all_warnings if (r[0] or "").lower() == "operational")
    output.print_summary(
        package_name=package_name,
        node_count=node_count,
        topic_count=topic_count,
        endpoint_count=endpoint_count,
        structural_count=struct,
        functional_count=func,
        operational_count=oper,
    )

    if all_warnings:
        sys.exit(0)
    output.print_success()


if __name__ == "__main__":
    main()
