#!/usr/bin/env python3
"""
XML 파싱 모듈.
Fast DDS QoS 프로파일 XML을 파싱하여 딕셔너리로 추출합니다.
"""
import re
from typing import Dict

# ────────── 태그 추출 패턴 ──────────
TAG_PATTERNS = {
    "reliability": re.compile(
        r"<\s*reliability\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*reliability\s*>",
        re.I | re.S,
    ),
    "history": re.compile(
        r"<\s*historyQos\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*historyQos\s*>",
        re.I | re.S,
    ),
    "history_depth": re.compile(
        r"<\s*historyQos\s*>.*?<\s*depth\s*>(\d+)</depth>", re.I | re.S
    ),
    "durability": re.compile(
        r"<\s*durability\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*durability\s*>",
        re.I | re.S,
    ),
    "ownership": re.compile(
        r"<\s*ownership\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*ownership\s*>",
        re.I | re.S,
    ),
    "dest_order": re.compile(
        r"<\s*destinationOrder\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*destinationOrder\s*>",
        re.I | re.S,
    ),
    "max_samples": re.compile(
        r"<\s*resourceLimitsQos\s*>.*?<\s*max_samples\s*>(\d+)</max_samples>",
        re.I | re.S,
    ),
    "max_instances": re.compile(
        r"<\s*resourceLimitsQos\s*>.*?<\s*max_instances\s*>(\d+)</max_instances>",
        re.I | re.S,
    ),
    "max_samples_per_instance": re.compile(
        r"<\s*resourceLimitsQos\s*>.*?<\s*max_samples_per_instance\s*>(\d+)"
        r"</max_samples_per_instance>",
        re.I | re.S,
    ),
    "autodispose": re.compile(
        r"<\s*writerDataLifecycle\s*>.*?"
        r"<\s*autodispose_unregistered_instances\s*>\s*(\w+)\s*"
        r"</autodispose_unregistered_instances\s*>.*?"
        r"</\s*writerDataLifecycle\s*>",
        re.I | re.S,
    ),
    "autoenable": re.compile(
        r"<\s*autoenable_created_entities\s*>\s*(\w+)\s*</autoenable_created_entities\s*>",
        re.I | re.S,
    ),
    "liveliness": re.compile(
        r"<\s*liveliness\s*>.*?<\s*kind\s*>(.*?)</kind\s*>.*?</\s*liveliness\s*>",
        re.I | re.S,
    ),
    "nowriter_sec_r": re.compile(
        r"<\s*readerDataLifecycle\s*>.*?"
        r"<\s*autopurge_nowriter_samples_delay\s*>.*?"
        r"<\s*sec\s*>([^<]+)</sec\s*>",
        re.I | re.S,
    ),
    "nowriter_nsec_r": re.compile(
        r"<\s*readerDataLifecycle\s*>.*?"
        r"<\s*autopurge_nowriter_samples_delay\s*>.*?"
        r"<\s*nanosec\s*>([^<]+)</nanosec\s*>",
        re.I | re.S,
    ),
    "userdata": re.compile(
        r"<\s*userData\s*>.*?<\s*value\s*>([^<]+)</value\s*>", re.I | re.S
    ),
    "autopurge_disposed_samples_delay": re.compile(
        r"<\s*readerDataLifecycle\s*>.*?"
        r"<\s*autopurge_disposed_samples_delay\s*>.*?"
        r"<\s*sec\s*>(\d+)</sec\s*>",
        re.I | re.S,
    ),
}

# ────────── DEADLINE 헬퍼 ──────────
DEADLINE_RE = re.compile(
    r"<\s*deadline\s*>[^<]*?<\s*period\s*>"
    r"(?:[^<]*?<\s*sec\s*>(\d+)\s*</sec\s*>)?"
    r"(?:[^<]*?<\s*nanosec\s*>(\d+)\s*</nanosec\s*>)?",
    re.I | re.S,
)

# ────────── LIVELINESS 헬퍼 ──────────
LEASE_RE = re.compile(
    r"<\s*liveliness\s*>.*?<\s*lease_duration\s*>"
    r"(?:[^<]*?<\s*sec\s*>(\d+)\s*</sec\s*>)?"
    r"(?:[^<]*?<\s*nanosec\s*>(\d+)\s*</nanosec\s*>)?",
    re.I | re.S,
)

ANNOUNCE_RE = re.compile(
    r"<\s*liveliness\s*>.*?<\s*announcement_period\s*>"
    r"(?:[^<]*?<\s*sec\s*>([^<]+)</sec\s*>)?"
    r"(?:[^<]*?<\s*nanosec\s*>([^<]+)</nanosec\s*>)?",
    re.I | re.S,
)

# ────────── LIFESPAN 헬퍼 ──────────
LIFESPAN_RE = re.compile(
    r"<\s*lifespan\s*>.*?</\s*lifespan\s*>",
    re.I | re.S,
)

# ────────── partition 헬퍼 ──────────
PART_ALL_RE = re.compile(
    r"<\s*partition\s*>.*?</\s*partition\s*>", re.I | re.S
)
NAME_RE = re.compile(r"<\s*name\s*>([^<]+)</name\s*>", re.I | re.S)

INF_SET = {"DURATION_INFINITY", "4294967295"}


def parse_duration_field(txt: str | None) -> int | None:
    """무한대(INF)이면 None, 아니면 정수 반환."""
    if not txt:
        return 0
    t = txt.strip().upper()
    if t in INF_SET:
        return None
    return int(t)


def is_inf(txt: str | None) -> bool:
    """텍스트가 무한대 값을 나타내는지 판정."""
    return txt and txt.strip().upper() in INF_SET


def deadline_enabled(xml: str) -> bool:
    """DEADLINE이 설정되어 있는지 확인."""
    m = DEADLINE_RE.search(xml)
    if not m:
        return False
    sec = int(m.group(1) or 0)
    nsec = int(m.group(2) or 0)
    return (sec != 0) or (nsec != 0)


def deadline_period_ns(xml: str) -> int | None:
    """DEADLINE period를 ns 단위 정수로 반환. 미설정이면 None."""
    m = DEADLINE_RE.search(xml)
    if not m:
        return None
    sec = int(m.group(1) or 0)
    nsec = int(m.group(2) or 0)
    return sec * 1_000_000_000 + nsec


def lease_duration_ns(xml: str) -> int | None:
    """liveliness lease_duration을 ns 단위로 반환."""
    m = LEASE_RE.search(xml)
    if not m:
        return None
    sec = int(m.group(1) or 0)
    nsec = int(m.group(2) or 0)
    return sec * 1_000_000_000 + nsec


def announcement_period_ns(xml: str) -> int | None:
    """liveliness announcement_period를 ns 단위로 반환."""
    m = ANNOUNCE_RE.search(xml)
    if not m:
        return None
    sec = parse_duration_field(m.group(1))
    nsec = parse_duration_field(m.group(2))
    if sec is None or nsec is None:
        return None
    return sec * 1_000_000_000 + nsec


def partition_list(xml: str) -> list[str]:
    """partition name 리스트를 추출합니다."""
    blk = PART_ALL_RE.search(xml)
    if not blk:
        return [""]
    names = NAME_RE.findall(blk.group(0))
    return [n.strip() for n in names] or [""]


def parse_profile(xml: str) -> Dict[str, str | list[str]]:
    """XML 문자열에서 QoS 프로파일을 파싱하여 딕셔너리로 반환합니다."""
    out: Dict[str, str | list[str]] = {}
    for k, pat in TAG_PATTERNS.items():
        m = pat.search(xml)
        out[k] = (m.group(1).upper() if m else "")
    out["partition_list"] = partition_list(xml)
    return out
