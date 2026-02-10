#!/usr/bin/env python3
"""
규칙 검사 모듈.
QoS 프로파일에 대해 단일/교차 규칙을 적용하여 위반 메시지를 반환합니다.
"""
import math
import re
from dataclasses import dataclass
from typing import Callable, List, Tuple

from . import xml_parser as xp

# xml_parser에서 재export
deadline_enabled = xp.deadline_enabled
deadline_period_ns = xp.deadline_period_ns
lease_duration_ns = xp.lease_duration_ns
partition_list = xp.partition_list
parse_duration_field = xp.parse_duration_field
is_inf = xp.is_inf
DEADLINE_RE = xp.DEADLINE_RE
LEASE_RE = xp.LEASE_RE
ANNOUNCE_RE = xp.ANNOUNCE_RE
LIFESPAN_RE = xp.LIFESPAN_RE
INF_SET = xp.INF_SET
NON_VOLATILE = {"TRANSIENT_LOCAL", "TRANSIENT", "PERSISTENT"}


@dataclass
class CheckContext:
    """규칙 검사에 필요한 컨텍스트 정보."""

    publish_period_ms: int
    rtt_ns: int


# ────────── 단일 프로파일 규칙 ──────────
def rule_durability_needs_rel(_xml, q, _ctx):
    if q["durability"] in NON_VOLATILE and q["reliability"] != "RELIABLE":
        return (
            "Invalid QoS: durability_kind is TRANSIENT_LOCAL/TRANSIENT/PERSISTENT "
            "but reliability_kind is not RELIABLE.\n"
            "Recommendation: use reliability_kind = RELIABLE with non-volatile durability."
        )
    return None


def rule_deadline_vs_durability(xml, q, _ctx):
    if deadline_enabled(xml) and q["durability"] in NON_VOLATILE:
        return (
            "QoS warning: durable samples may arrive late and reset the DEADLINE "
            "timer, potentially masking real timing violations.\n"
            "Recommendation: use VOLATILE durability when DEADLINE is critical, "
            "or relax / disable DEADLINE to tolerate replayed samples."
        )
    return None


def rule_lease_vs_deadline(xml, _q, _ctx):
    dl_m = DEADLINE_RE.search(xml)
    ld_m = LEASE_RE.search(xml)
    if not dl_m or not ld_m:
        return None
    dl_sec, dl_nsec = int(dl_m.group(1) or 0), int(dl_m.group(2) or 0)
    ld_sec, ld_nsec = int(ld_m.group(1) or 0), int(ld_m.group(2) or 0)
    dl_ns = dl_sec * 1_000_000_000 + dl_nsec
    ld_ns = ld_sec * 1_000_000_000 + ld_nsec
    if ld_ns < dl_ns:
        return (
            "lease_duration < deadline_period: DEADLINE timer may stop prematurely, "
            "hiding real-time deadline violations.\n"
            f"lease_duration  : {ld_sec}s {ld_nsec}ns ({ld_ns/1_000_000:.1f} ms)\n"
            f"deadline_period : {dl_sec}s {dl_nsec}ns ({dl_ns/1_000_000:.1f} ms)\n"
            "Recommendation  : set lease_duration ≥ deadline_period or relax the DEADLINE QoS."
        )
    return None


def rule_exclusive_best_effort_deadline(xml, q, _ctx):
    if not deadline_enabled(xml):
        return None
    if q["reliability"] == "BEST_EFFORT" and q["ownership"] == "EXCLUSIVE":
        return (
            "EXCLUSIVE + BEST_EFFORT may cause false DEADLINE misses and invalid ownership transitions.\n"
            "Recommendation: use RELIABLE for stable EXCLUSIVE ownership."
        )
    return None


def rule_autodispose_with_best_effort(_xml, q, _ctx):
    auto = q.get("autodispose", "").strip().upper() == "TRUE"
    if q["reliability"] == "BEST_EFFORT" and auto:
        return (
            "WRITER_DATA_LIFECYCLE may be ineffective under BEST_EFFORT.\n"
            "Dispose/unregister messages can be lost.\n"
            "Recommendation: use RELIABLE when relying on autodispose_unregistered_instances."
        )
    return None


def rule_lifespan_vs_deadline(xml, _q, _ctx):
    dl_m = DEADLINE_RE.search(xml)
    if not dl_m:
        return None
    lifespan_m = LIFESPAN_RE.search(xml)
    if not lifespan_m:
        return None
    lifespan_block = lifespan_m.group(0)
    sec_m = re.search(r"<\s*sec\s*>(\d+)</sec\s*>", lifespan_block, re.I)
    nsec_m = re.search(r"<\s*nanosec\s*>(\d+)</nanosec\s*>", lifespan_block, re.I)
    if not sec_m and not nsec_m:
        return None
    ls_sec = int(sec_m.group(1)) if sec_m else 0
    ls_nsec = int(nsec_m.group(1)) if nsec_m else 0
    dl_sec = int(dl_m.group(1) or 0)
    dl_nsec = int(dl_m.group(2) or 0)
    ls_ns = ls_sec * 1_000_000_000 + ls_nsec
    dl_ns = dl_sec * 1_000_000_000 + dl_nsec
    if ls_ns < dl_ns:
        return (
            "Invalid QoS: LIFESPAN duration is shorter than DEADLINE period.\n"
            f"LIFESPAN : {ls_sec}s {ls_nsec}ns ({ls_ns/1_000_000:.1f} ms)\n"
            f"DEADLINE: {dl_sec}s {dl_nsec}ns ({dl_ns/1_000_000:.1f} ms)\n"
            "Recommendation: set lifespan ≥ deadline to ensure samples remain valid "
            "until the deadline timer expires."
        )
    return None


def rule_dest_order_vs_depth(_xml, q, _ctx):
    if q["dest_order"] != "BY_SOURCE_TIMESTAMP":
        return None
    if not q.get("history_depth", "").isdigit():
        return None
    depth = int(q["history_depth"])
    if depth <= 1:
        return (
            "BY_SOURCE_TIMESTAMP with history depth ≤ 1 may drop out-of-order samples "
            "due to lack of reordering buffer.\n"
            "Recommendation: increase history depth to at least 2 when using BY_SOURCE_TIMESTAMP."
        )
    return None


def rule_history_vs_max_per_instance(_xml, q, _ctx):
    hist_kind = q.get("history", "").strip().upper()
    depth_txt = q.get("history_depth", "").strip()
    mpi_txt = q.get("max_samples_per_instance", "").strip() or "0"
    depth = int(depth_txt) if depth_txt.isdigit() else 0
    mpi = int(mpi_txt) if mpi_txt.isdigit() else 0
    if hist_kind == "KEEP_LAST" and depth > mpi:
        return (
            f"Invalid QoS: KEEP_LAST depth={depth} exceeds "
            f"max_samples_per_instance={mpi}.\n"
            "Recommendation: increase max_samples_per_instance or reduce history depth."
        )
    if hist_kind == "KEEP_ALL" and mpi == 0:
        return (
            "Invalid QoS: KEEP_ALL with max_samples_per_instance=0 stores no samples at all.\n"
            "Recommendation: set max_samples_per_instance to a positive value."
        )
    return None


def rule_autoenable_vs_volatile_reader(_xml, q, _ctx):
    auto_off = q.get("autoenable", "").strip().upper() == "FALSE"
    is_volatile = q.get("durability", "").upper() == "VOLATILE"
    if auto_off and is_volatile:
        return (
            "QoS warning: autoenable_created_entities=false while durability_kind=VOLATILE.\n"
            "Late-enabled DataReaders will MISS all samples published before enable().\n"
            "Recommendation: set autoenable_created_entities=true, or switch to "
            "TRANSIENT_LOCAL (or higher) durability to retain data for late joiners."
        )
    return None


def rule_max_samples_vs_per_instance(_xml, q, _ctx):
    max_s_txt = q.get("max_samples", "").strip()
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if not max_s_txt.isdigit() or not mpi_txt.isdigit():
        return None
    max_s = int(max_s_txt)
    mpi = int(mpi_txt)
    if max_s < mpi:
        return (
            f"Invalid QoS: max_samples ({max_s}) is less than "
            f"max_samples_per_instance ({mpi}).\n"
            "This setting prevents even a single instance from storing the expected number of samples.\n"
            "Recommendation: increase max_samples ≥ max_samples_per_instance."
        )
    return None


def rule_destorder_keepall_mpi(_xml, q, _ctx):
    if q.get("dest_order", "").upper() != "BY_SOURCE_TIMESTAMP":
        return None
    if q.get("history", "").upper() != "KEEP_ALL":
        return None
    if q.get("max_samples_per_instance", "").strip() != "1":
        return None
    return (
        "Invalid QoS: BY_SOURCE_TIMESTAMP + KEEP_ALL + max_samples_per_instance = 1 "
        "does not provide sufficient buffer to reorder samples.\n"
        "Recommendation: increase max_samples_per_instance > 1 "
        "or switch to destination_order = BY_RECEPTION_TIMESTAMP."
    )


def rule_rdlife_autopurge_vs_durability(_xml, q, _ctx):
    dur_kind = q.get("durability", "").strip().upper()
    auto_delay = q.get("autopurge_disposed_samples_delay", "").strip()
    if dur_kind in NON_VOLATILE and auto_delay == "0":
        return (
            "Invalid QoS: DURABILITY.kind ≥ TRANSIENT and autopurge_disposed_samples_delay = 0.\n"
            "This setting causes DISPOSED samples to be purged immediately, negating the durability.\n"
            "Recommendation: set autopurge_disposed_samples_delay > 0 "
            "to allow late-joiners to observe disposed instances."
        )
    return None


def rule_liveliness_manual_partition(_xml, q, _ctx):
    if q.get("liveliness", "").strip().upper() != "MANUAL_BY_TOPIC":
        return None
    part_list = q.get("partition_list", [])
    if part_list and any(p.strip() != "" for p in part_list):
        return (
            "Invalid QoS: LIVELINESS.kind = MANUAL_BY_TOPIC with non-empty PARTITION.\n"
            "Manual-by-topic requires the Writer to assert liveliness per partition, "
            "which may cause unexpected liveliness loss in unused partitions.\n"
            "Recommendation: use AUTOMATIC or MANUAL_BY_PARTICIPANT, or remove partition."
        )
    return None


def rule_autodispose_with_exclusive(_xml, q, _ctx):
    auto = q.get("autodispose", "").strip().upper()
    owner_kind = q.get("ownership", "").strip().upper()
    if auto == "TRUE" and owner_kind == "EXCLUSIVE":
        return (
            "Invalid QoS: autodispose_unregistered_instances = TRUE with EXCLUSIVE ownership.\n"
            "When the exclusive Writer unregisters, its instance is disposed immediately, "
            "preventing smooth ownership handover.\n"
            "Recommendation: set autodispose_unregistered_instances = FALSE to allow new "
            "exclusive Writers to take over without premature instance disposal."
        )
    return None


def rule_lifespan_too_short_for_durability(xml, q, ctx):
    dur_kind = q.get("durability", "").strip().upper()
    lifespan_sec_match = re.search(r"<\s*lifespan\s*>.*?<\s*sec\s*>(\d+)</sec\s*>", xml, re.I | re.S)
    lifespan_nsec_match = re.search(r"<\s*lifespan\s*>.*?<\s*nanosec\s*>(\d+)</nanosec\s*>", xml, re.I | re.S)
    if not lifespan_sec_match and not lifespan_nsec_match:
        return None
    ls_sec = int(lifespan_sec_match.group(1)) if lifespan_sec_match else 0
    ls_nsec = int(lifespan_nsec_match.group(1)) if lifespan_nsec_match else 0
    lifespan_ns = ls_sec * 1_000_000_000 + ls_nsec
    rtt_ns = ctx.rtt_ns
    if dur_kind in NON_VOLATILE and lifespan_ns < rtt_ns:
        return (
            f"Invalid QoS: DURABILITY.kind = {dur_kind} with LIFESPAN duration < RTT.\n"
            f"LIFESPAN: {ls_sec}s {ls_nsec}ns ({lifespan_ns/1e6:.1f} ms) < RTT ({rtt_ns/1e6:.1f} ms).\n"
            "This setting may cause samples to expire before they are delivered to late-joiners.\n"
            "Recommendation: set lifespan ≥ RTT, or relax durability if replay is not required."
        )
    return None


def rule_exclusive_lease_infinite(xml, q, _ctx):
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    m = LEASE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return (
            "Invalid QoS: EXCLUSIVE ownership with infinite lease_duration.\n"
            "The Writer may never be considered 'dead', preventing ownership transfer.\n"
            "Recommendation: set a finite lease_duration (e.g., 1s) to enable liveliness loss detection."
        )
    return None


def rule_nowriter_delay_vs_infinite_lease(xml, q, _ctx):
    nowriter_sec = q.get("nowriter_sec_r", "").strip()
    nowriter_nsec = q.get("nowriter_nsec_r", "").strip()
    try:
        sec = int(nowriter_sec) if nowriter_sec else 0
        nsec = int(nowriter_nsec) if nowriter_nsec else 0
    except ValueError:
        return None
    purge_ns = sec * 1_000_000_000 + nsec
    if purge_ns == 0:
        return None
    m = LEASE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return (
            "Invalid QoS: Reader wants to purge samples after Writer disappearance "
            f"(autopurge_nowriter_samples_delay = {purge_ns / 1e6:.1f} ms), "
            "but liveliness lease_duration is infinite.\n"
            "→ DDS can never detect Writer loss.\n"
            "Recommendation: set a finite lease_duration to enable liveliness loss detection."
        )
    return None


def rule_reliable_keep_last_depth_too_small(xml, q, ctx):
    if q.get("reliability", "").strip().upper() != "RELIABLE":
        return None
    if q.get("history", "").strip().upper() != "KEEP_LAST":
        return None
    depth_txt = q.get("history_depth", "").strip()
    if not depth_txt.isdigit():
        return None
    depth = int(depth_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required_depth = math.ceil(rtt_sec / pp_sec) + 2
    if depth < required_depth:
        return (
            f"Invalid QoS: RELIABLE + KEEP_LAST({depth}) is too shallow.\n"
            f"Required depth ≥ ⌈RTT / PP⌉ + 2 = ⌈{rtt_sec:.3f}s / {pp_sec:.3f}s⌉ + 2 = {required_depth}.\n"
            "Samples may be dropped before NACK retransmission is possible.\n"
            "Recommendation: increase history depth to at least this value."
        )
    return None


def rule_keepall_max_samples_per_instance(xml, q, ctx):
    if q.get("reliability", "").strip().upper() != "RELIABLE":
        return None
    if q.get("history", "").strip().upper() != "KEEP_ALL":
        return None
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if not mpi_txt.isdigit():
        return None
    mpi = int(mpi_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required_samples = math.ceil(rtt_sec / pp_sec) + 2
    if mpi < required_samples:
        return (
            f"Invalid QoS: RELIABLE + KEEP_ALL + max_samples_per_instance = {mpi} is too small.\n"
            f"Required ≥ ⌈RTT / PP⌉ + 2 = ⌈{rtt_sec:.3f}s / {pp_sec:.3f}s⌉ + 2 = {required_samples}.\n"
            "This setting may cause loss of samples before retransmission is completed.\n"
            "Recommendation: increase max_samples_per_instance to at least this value."
        )
    return None


def rule_lifespan_too_short_for_reliability(xml, q, ctx):
    if q.get("reliability", "").strip().upper() != "RELIABLE":
        return None
    lifespan_sec_match = re.search(r"<\s*lifespan\s*>.*?<\s*sec\s*>(\d+)</sec\s*>", xml, re.I | re.S)
    lifespan_nsec_match = re.search(r"<\s*lifespan\s*>.*?<\s*nanosec\s*>(\d+)</nanosec\s*>", xml, re.I | re.S)
    if not lifespan_sec_match and not lifespan_nsec_match:
        return None
    ls_sec = int(lifespan_sec_match.group(1)) if lifespan_sec_match else 0
    ls_nsec = int(lifespan_nsec_match.group(1)) if lifespan_nsec_match else 0
    lifespan_ns = ls_sec * 1_000_000_000 + ls_nsec
    if lifespan_ns < ctx.rtt_ns:
        return (
            f"Invalid QoS: RELIABLE set but LIFESPAN duration < RTT.\n"
            f"LIFESPAN = {ls_sec}s {ls_nsec}ns = {lifespan_ns/1e6:.1f} ms < RTT = {ctx.rtt_ns/1e6:.1f} ms.\n"
            "This causes samples to expire before retransmission can occur.\n"
            "Recommendation: set lifespan ≥ RTT when using RELIABLE."
        )
    return None


def rule_best_effort_with_manual_liveliness(_xml, q, _ctx):
    live_kind = q.get("liveliness", "").strip().upper()
    reliab = q.get("reliability", "").strip().upper()
    if live_kind == "MANUAL_BY_TOPIC" and reliab == "BEST_EFFORT":
        return (
            "Invalid QoS: MANUAL_BY_TOPIC liveliness requires reliable communication.\n"
            "Using BEST_EFFORT may cause liveliness assertions to be lost,\n"
            "resulting in false WRITER_NOT_ALIVE detection.\n"
            "Recommendation: use RELIABLE reliability_kind with MANUAL_BY_TOPIC liveliness."
        )
    return None


def rule_deadline_too_short_for_exclusive(xml, q, ctx):
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    if not deadline_enabled(xml):
        return None
    deadline_ns = deadline_period_ns(xml)
    if deadline_ns is None:
        return None
    pub_ns = ctx.publish_period_ms * 1_000_000
    min_required = 2 * pub_ns
    if deadline_ns < min_required:
        return (
            f"Invalid QoS: EXCLUSIVE ownership with DEADLINE period < 2×publish_period.\n"
            f"DEADLINE = {deadline_ns/1e6:.1f} ms, publish_period = {ctx.publish_period_ms} ms → "
            f"required ≥ {2*ctx.publish_period_ms} ms.\n"
            "This may cause false ownership transfer due to minor publish delays.\n"
            "Recommendation: increase DEADLINE period to ≥ 2×publish_period."
        )
    return None


def rule_lease_too_short_for_exclusive(xml, q, ctx):
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    if q.get("liveliness", "").strip().upper() == "":
        return None
    lease_ns = lease_duration_ns(xml)
    if lease_ns is None:
        return None
    pub_ns = ctx.publish_period_ms * 1_000_000
    required_ns = 2 * pub_ns
    if lease_ns < required_ns:
        return (
            f"Invalid QoS: EXCLUSIVE ownership with liveliness lease_duration < 2×publish_period.\n"
            f"lease_duration = {lease_ns/1e6:.1f} ms, publish_period = {ctx.publish_period_ms} ms → "
            f"required ≥ {2*ctx.publish_period_ms} ms.\n"
            "This may cause false Writer death detection and unwanted ownership transfer.\n"
            "Recommendation: increase lease_duration to ≥ 2×publish_period."
        )
    return None


def rule_keepall_durable_instance_budget(xml, q, ctx):
    dur_kind = q.get("durability", "").strip().upper()
    if dur_kind not in NON_VOLATILE:
        return None
    if q.get("history", "").strip().upper() != "KEEP_ALL":
        return None
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if not mpi_txt.isdigit():
        return None
    mpi = int(mpi_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required = math.ceil(rtt_sec / pp_sec) + 2
    if mpi < required:
        return (
            f"Invalid QoS: DURABILITY.kind = {dur_kind}, KEEP_ALL, but max_samples_per_instance = {mpi} "
            f"is too small.\nRequired ≥ ⌈RTT / PP⌉ + 2 = ⌈{rtt_sec:.3f}s / {pp_sec:.3f}s⌉ + 2 = {required}.\n"
            "This may cause durable samples to be dropped before late-joiners arrive or NACKs are processed.\n"
            "Recommendation: increase max_samples_per_instance to at least this value."
        )
    return None


def rule_durable_keep_last_depth_1(xml, q, ctx):
    dur_kind = q.get("durability", "").strip().upper()
    hist_kind = q.get("history", "").strip().upper()
    if dur_kind not in NON_VOLATILE or hist_kind != "KEEP_LAST":
        return None
    depth_txt = q.get("history_depth", "").strip()
    if not depth_txt.isdigit():
        return None
    depth = int(depth_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required_depth = math.ceil(rtt_sec / pp_sec) + 2
    if depth < required_depth:
        return (
            f"Invalid QoS: DURABILITY.kind = {dur_kind}, KEEP_LAST({depth}) is too small.\n"
            f"Required depth ≥ ⌈RTT / PP⌉ + 2 = ⌈{rtt_sec:.3f}s / {pp_sec:.3f}s⌉ + 2 = {required_depth}.\n"
            "Durable samples may be lost before late-joiners or retransmission.\n"
            "Recommendation: increase history depth to at least this value."
        )
    return None


def rule_keepall_durable_instance_budget_1(xml, q, ctx):
    dur_kind = q.get("durability", "").strip().upper()
    if dur_kind not in NON_VOLATILE:
        return None
    if q.get("history", "").strip().upper() != "KEEP_ALL":
        return None
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if not mpi_txt.isdigit():
        return None
    mpi = int(mpi_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required = math.ceil(rtt_sec / pp_sec) + 2
    if mpi > required:
        return (
            f"Invalid QoS: KEEP_ALL + DURABILITY enabled, but max_samples_per_instance = {mpi} is too large.\n"
            f"Only ⌈RTT/PP⌉+2 = ⌈{rtt_sec:.3f}/{pp_sec:.3f}⌉+2 = {required} samples needed.\n"
            "Recommendation: reduce max_samples_per_instance to save memory."
        )
    return None


def rule_durable_keep_last_depth_2(xml, q, ctx):
    dur_kind = q.get("durability", "").strip().upper()
    hist_kind = q.get("history", "").strip().upper()
    if dur_kind not in NON_VOLATILE or hist_kind != "KEEP_LAST":
        return None
    depth_txt = q.get("history_depth", "").strip()
    if not depth_txt.isdigit():
        return None
    depth = int(depth_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required_depth = math.ceil(rtt_sec / pp_sec) + 2
    if depth > required_depth:
        return (
            f"Invalid QoS: DURABILITY={dur_kind} + KEEP_LAST({depth}) is too deep.\n"
            f"Only ⌈RTT/PP⌉+2 = ⌈{rtt_sec:.3f}/{pp_sec:.3f}⌉+2 = {required_depth} needed.\n"
            f"Recommendation: reduce history depth to ≤ {required_depth} to save memory."
        )
    return None


def rule_exclusive_deadline_infinite(xml, q, _ctx):
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    m = DEADLINE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return (
            "Invalid QoS: EXCLUSIVE ownership with DEADLINE = ∞.\n"
            "The system cannot detect Writer staleness, preventing ownership handover.\n"
            "Recommendation: set a finite DEADLINE period (e.g., 1s) to allow handover if Writer becomes inactive."
        )
    return None


def rule_lifespan_exceeds_per_instance(xml, q, ctx):
    if q.get("history", "").strip().upper() != "KEEP_ALL":
        return None
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if not mpi_txt.isdigit():
        return None
    mpi = int(mpi_txt)
    pp_sec = ctx.publish_period_ms / 1000
    lifespan_m_sec = re.search(r"<\s*lifespan\s*>.*?<\s*sec\s*>(\d+)</sec\s*>", xml, re.I | re.S)
    lifespan_m_nsec = re.search(r"<\s*lifespan\s*>.*?<\s*nanosec\s*>(\d+)</nanosec\s*>", xml, re.I | re.S)
    if not lifespan_m_sec and not lifespan_m_nsec:
        return None
    ls_sec = int(lifespan_m_sec.group(1)) if lifespan_m_sec else 0
    ls_nsec = int(lifespan_m_nsec.group(1)) if lifespan_m_nsec else 0
    lifespan_sec = ls_sec + (ls_nsec / 1_000_000_000)
    allowed_sec = mpi * pp_sec
    if lifespan_sec > allowed_sec:
        return (
            f"Invalid QoS: KEEP_ALL with max_samples_per_instance = {mpi} cannot store samples "
            f"for lifespan = {lifespan_sec:.3f}s.\n"
            f"Lifespan > max_samples_per_instance × publish_period = {mpi} × {pp_sec:.3f}s = {allowed_sec:.3f}s.\n"
            "This causes valid samples to be discarded early.\n"
            "Recommendation: increase max_samples_per_instance or reduce lifespan."
        )
    return None


def rule_keep_last_lifespan_overflow(xml, q, ctx):
    if q.get("history", "").strip().upper() != "KEEP_LAST":
        return None
    depth_txt = q.get("history_depth", "").strip()
    if not depth_txt.isdigit():
        return None
    depth = int(depth_txt)
    pp_sec = ctx.publish_period_ms / 1000
    lifespan_sec_match = re.search(r"<lifespan>.*?<sec>(\d+)</sec>", xml, re.I | re.S)
    lifespan_nsec_match = re.search(r"<lifespan>.*?<nanosec>(\d+)</nanosec>", xml, re.I | re.S)
    if not lifespan_sec_match and not lifespan_nsec_match:
        return None
    ls_sec = int(lifespan_sec_match.group(1)) if lifespan_sec_match else 0
    ls_nsec = int(lifespan_nsec_match.group(1)) if lifespan_nsec_match else 0
    lifespan_sec = ls_sec + ls_nsec / 1e9
    if lifespan_sec > depth * pp_sec:
        return (
            f"Invalid QoS: KEEP_LAST(depth={depth}) × publish_period({pp_sec:.3f}s) "
            f"= {depth * pp_sec:.3f}s < lifespan = {lifespan_sec:.3f}s.\n"
            "Samples may be overwritten before they expire.\n"
            "Recommendation: reduce lifespan or increase history depth."
        )
    return None


# ────────── 단일 규칙 목록 ──────────
RULES: List[Tuple[Callable, str]] = [
    (rule_durability_needs_rel, "Critical"),
    (rule_deadline_vs_durability, "Incidental"),
    (rule_lease_vs_deadline, "Conditional"),
    (rule_exclusive_best_effort_deadline, "Conditional"),
    (rule_autodispose_with_best_effort, "Conditional"),
    (rule_lifespan_vs_deadline, "Critical"),
    (rule_dest_order_vs_depth, "Conditional"),
    (rule_history_vs_max_per_instance, "Critical"),
    (rule_autoenable_vs_volatile_reader, "Incidental"),
    (rule_max_samples_vs_per_instance, "Critical"),
    (rule_destorder_keepall_mpi, "Conditional"),
    (rule_rdlife_autopurge_vs_durability, "Incidental"),
    (rule_liveliness_manual_partition, "Incidental"),
    (rule_autodispose_with_exclusive, "Incidental"),
    (rule_lifespan_too_short_for_durability, "Conditional"),
    (rule_exclusive_lease_infinite, "Conditional"),
    (rule_nowriter_delay_vs_infinite_lease, "Conditional"),
    (rule_reliable_keep_last_depth_too_small, "Conditional"),
    (rule_keepall_max_samples_per_instance, "Conditional"),
    (rule_lifespan_too_short_for_reliability, "Conditional"),
    (rule_best_effort_with_manual_liveliness, "Conditional"),
    (rule_deadline_too_short_for_exclusive, "Conditional"),
    (rule_lease_too_short_for_exclusive, "Conditional"),
    (rule_keepall_durable_instance_budget, "Conditional"),
    (rule_durable_keep_last_depth_1, "Conditional"),
    (rule_keepall_durable_instance_budget_1, "Conditional"),
    (rule_durable_keep_last_depth_2, "Conditional"),
    (rule_exclusive_deadline_infinite, "Conditional"),
    (rule_lifespan_exceeds_per_instance, "Conditional"),
    (rule_keep_last_lifespan_overflow, "Conditional"),
]


# ────────── 교차 규칙 ──────────
RELIABILITY_LEVEL = {"BEST_EFFORT": 0, "RELIABLE": 1}
DURABILITY_LEVEL = {
    "VOLATILE": 0,
    "TRANSIENT_LOCAL": 1,
    "TRANSIENT": 2,
    "PERSISTENT": 3,
}
LIVELINESS_PRIORITY = {
    "AUTOMATIC": 0,
    "MANUAL_BY_PARTICIPANT": 1,
    "MANUAL_BY_TOPIC": 2,
}


def rule_dest_order_compat(pub_q, sub_q):
    w_kind = (pub_q.get("dest_order", "") or "BY_RECEPTION_TIMESTAMP").strip().upper()
    r_kind = (sub_q.get("dest_order", "") or "BY_RECEPTION_TIMESTAMP").strip().upper()
    if w_kind == "BY_RECEPTION_TIMESTAMP" and r_kind == "BY_SOURCE_TIMESTAMP":
        return (
            "Incompatible destination_order_kind: Writer='BY_RECEPTION_TIMESTAMP', "
            "Reader='BY_SOURCE_TIMESTAMP'.\n"
            "Reader expects stricter BY_SOURCE_TIMESTAMP ordering than Writer provides.\n"
            "Recommendation: set writer destination_order_kind = BY_SOURCE_TIMESTAMP, "
            "or relax reader requirement to BY_RECEPTION_TIMESTAMP."
        )
    return None


def rule_ownership_compat(pub_q, sub_q):
    r_kind = (sub_q.get("ownership", "") or "SHARED").strip().upper()
    w_kind = (pub_q.get("ownership", "") or "SHARED").strip().upper()
    if r_kind == "EXCLUSIVE" and w_kind != "EXCLUSIVE":
        return (
            "Reader requests EXCLUSIVE ownership but Writer is not EXCLUSIVE.\n"
            "Data-instance hand-over rules will not be honoured.\n"
            "Recommendation: set writer ownership_kind to EXCLUSIVE to match "
            "the reader, or change reader to SHARED."
        )
    return None


def rule_reliability_compat(pub_q, sub_q):
    w_kind = (pub_q.get("reliability", "") or "BEST_EFFORT").strip().upper()
    r_kind = (sub_q.get("reliability", "") or "BEST_EFFORT").strip().upper()
    w_lvl = RELIABILITY_LEVEL.get(w_kind, 0)
    r_lvl = RELIABILITY_LEVEL.get(r_kind, 0)
    if w_lvl < r_lvl:
        return (
            f"Incompatible reliability_kind: Writer='{w_kind}', Reader='{r_kind}'.\n"
            "Reader expects RELIABLE delivery but Writer is BEST_EFFORT.\n"
            "Recommendation: set writer reliability_kind = RELIABLE, "
            "or relax reader requirement to BEST_EFFORT."
        )
    return None


def rule_durability_compat(pub_q, sub_q):
    w_kind = (pub_q.get("durability", "") or "VOLATILE").strip().upper()
    r_kind = (sub_q.get("durability", "") or "VOLATILE").strip().upper()
    w_lvl = DURABILITY_LEVEL.get(w_kind, 0)
    r_lvl = DURABILITY_LEVEL.get(r_kind, 0)
    if w_lvl < r_lvl:
        return (
            f"Incompatible durability_kind: Writer='{w_kind}', Reader='{r_kind}'.\n"
            "Reader expects stronger durability than Writer provides.\n"
            "Recommendation: raise writer durability_kind "
            f"to at least '{r_kind}', or lower reader requirement."
        )
    return None


def rule_deadline_period_compat(pub_xml, sub_xml):
    w_ns = xp.deadline_period_ns(pub_xml)
    r_ns = xp.deadline_period_ns(sub_xml)
    if r_ns is None:
        return None
    if w_ns is None:
        return (
            "Incompatible DEADLINE: Reader specifies a DEADLINE period "
            "but Writer has none.\nRecommendation: add a DEADLINE period "
            "to the Writer that is ≤ Reader's requirement "
            "or remove DEADLINE from the Reader."
        )
    if r_ns != 0 and w_ns > r_ns:
        return (
            f"Incompatible DEADLINE periods: Writer={w_ns/1e9:.3f}s "
            f"> Reader={r_ns/1e9:.3f}s.\n"
            "Recommendation: shorten Writer DEADLINE period "
            "or relax Reader requirement."
        )
    return None


def rule_nowriter_autodispose_cross(pub_q, sub_q):
    auto_off = pub_q.get("autodispose", "").strip().upper() == "FALSE"
    sec_txt = sub_q.get("nowriter_sec_r", "")
    nsec_txt = sub_q.get("nowriter_nsec_r", "")
    if not sec_txt and not nsec_txt:
        return None
    inf_delay = is_inf(sec_txt) or is_inf(nsec_txt)
    zero_delay = (
        (sec_txt.strip() == "0" or sec_txt == "")
        and (nsec_txt.strip() == "0" or nsec_txt == "")
    )
    if auto_off and (inf_delay or zero_delay):
        return (
            "Invalid QoS: autodispose_unregistered_instances=FALSE in the Writer "
            "while Reader autopurge_nowriter_samples_delay is INFINITE/0.\n"
            "Samples may never be purged when all writers disappear, causing "
            "unbounded memory growth.\n"
            "Recommendation: enable autodispose_unregistered_instances in the Writer "
            "or set a finite autopurge_nowriter_samples_delay in the Reader."
        )
    return None


def rule_partition_overlap(pub_xml, sub_xml):
    w_parts = set(xp.partition_list(pub_xml))
    r_parts = set(xp.partition_list(sub_xml))
    if w_parts.isdisjoint(r_parts):
        return (
            "No matching partition names between Writer and Reader; "
            "data exchange will not occur.\n"
            f"Writer partitions : {sorted(w_parts)}\n"
            f"Reader partitions : {sorted(r_parts)}\n"
            "Recommendation: configure at least one identical <partition><name> "
            "string on both sides."
        )
    return None


def rule_durable_partition_miss(pub_xml, sub_xml, pub_q, sub_q):
    if pub_q.get("durability", "").strip().upper() not in NON_VOLATILE:
        return None
    w_parts = set(xp.partition_list(pub_xml))
    r_parts = set(xp.partition_list(sub_xml))
    if w_parts.isdisjoint(r_parts):
        return (
            "Durable samples are retransmitted only to Readers in the same "
            "partition. Writer partitions and Reader partitions share no "
            "common name, so late-joiner will start with an empty cache.\n"
            f"Writer partitions : {sorted(w_parts)}\n"
            f"Reader partitions : {sorted(r_parts)}\n"
            "Recommendation: configure at least one identical partition name "
            "or use VOLATILE durability if replay is not required."
        )
    return None


def rule_deadline_partition_reset(pub_xml, sub_xml):
    if not xp.deadline_enabled(sub_xml):
        return None
    w_parts = set(xp.partition_list(pub_xml))
    r_parts = set(xp.partition_list(sub_xml))
    if w_parts.isdisjoint(r_parts):
        return (
            "Partition mismatch causes the Reader to perceive the Writer as "
            "a 'new' instance, resetting the DEADLINE timer. Miss detection "
            "may be masked or delayed.\n"
            f"Writer partitions : {sorted(w_parts)}\n"
            f"Reader partitions : {sorted(r_parts)}\n"
            "Recommendation: share at least one partition or disable DEADLINE "
            "if Writer mobility across partitions is expected."
        )
    return None


def rule_liveliness_incompatibility(pub_xml, sub_xml, pub_q, sub_q):
    pub_kind = (pub_q.get("liveliness", "") or "AUTOMATIC").strip().upper()
    sub_kind = (sub_q.get("liveliness", "") or "AUTOMATIC").strip().upper()
    pub_lvl = LIVELINESS_PRIORITY.get(pub_kind, 0)
    sub_lvl = LIVELINESS_PRIORITY.get(sub_kind, 0)
    pub_lease = xp.lease_duration_ns(pub_xml)
    sub_lease = xp.lease_duration_ns(sub_xml)
    msgs = []
    if pub_lvl < sub_lvl:
        msgs.append(
            f"LIVELINESS.kind mismatch: Writer='{pub_kind}' < Reader='{sub_kind}'.\n"
            "Recommendation: increase Writer's liveliness kind to match or exceed Reader's requirement."
        )
    if pub_lease is not None and sub_lease is not None and pub_lease > sub_lease:
        msgs.append(
            f"LIVELINESS.lease_duration mismatch: Writer={pub_lease/1e9:.3f}s > Reader={sub_lease/1e9:.3f}s.\n"
            "Writer refreshes liveliness less frequently than Reader expects.\n"
            "Recommendation: set Writer lease_duration ≤ Reader lease_duration."
        )
    if msgs:
        return "Invalid QoS:\n" + "\n".join(msgs)
    return None


# ────────── 교차 규칙 목록 (rule_fn, severity, needs_xml) ──────────
CROSS_RULES_XML_ONLY = [
    (rule_deadline_period_compat, "Critical"),
    (rule_partition_overlap, "Critical"),
    (rule_deadline_partition_reset, "Incidental"),
]
CROSS_RULES_PROFILES = [
    (rule_dest_order_compat, "Critical"),
    (rule_ownership_compat, "Critical"),
    (rule_reliability_compat, "Critical"),
    (rule_durability_compat, "Critical"),
    (rule_nowriter_autodispose_cross, "Conditional"),
]
CROSS_RULES_FULL = [
    (rule_durable_partition_miss, "Incidental"),
    (rule_liveliness_incompatibility, "Critical"),
]


def run_checks(
    pub_xml: str, sub_xml: str, pub_q: dict, sub_q: dict, ctx: CheckContext
) -> List[Tuple[str, str, str]]:
    """
    모든 규칙을 실행하고 위반 목록을 반환합니다.
    반환: [(severity, side, msg), ...]
    """
    warnings: List[Tuple[str, str, str]] = []

    for side, (xml, prof) in (("PUB", (pub_xml, pub_q)), ("SUB", (sub_xml, sub_q))):
        for rule_fn, severity in RULES:
            msg = rule_fn(xml, prof, ctx)
            if msg:
                warnings.append((severity, side, msg))

    for rule_fn, severity in CROSS_RULES_XML_ONLY:
        msg = rule_fn(pub_xml, sub_xml)
        if msg:
            warnings.append((severity, "CROSS", msg))

    for rule_fn, severity in CROSS_RULES_PROFILES:
        msg = rule_fn(pub_q, sub_q)
        if msg:
            warnings.append((severity, "CROSS", msg))

    for rule_fn, severity in CROSS_RULES_FULL:
        msg = rule_fn(pub_xml, sub_xml, pub_q, sub_q)
        if msg:
            warnings.append((severity, "CROSS", msg))

    return warnings
