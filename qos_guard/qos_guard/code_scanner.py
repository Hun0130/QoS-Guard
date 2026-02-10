#!/usr/bin/env python3
"""
ROS 2 소스 코드 QoS 정적 분석기.

3단계 일반화 스캐닝:
1) Global Symbol Collection: rclcpp::QoS를 반환/생성하는 심볼 수집
2) Internal Logic Parsing: 정의부 내 .reliable(), .keep_last(N) 등 메서드 체이닝 파싱
3) Caller Mapping: create_publisher(..., MyQoS())에서 사전 조회 후 대입
"""
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from . import qos_profile_resolver
from . import yaml_qos_loader


def find_source_files(package_path: Path) -> list[Path]:
    """Recursively find .cpp, .hpp, .h, .py source files under package path."""
    package_path = Path(package_path).resolve()
    if not package_path.is_dir():
        return []

    exts = ("*.cpp", "*.hpp", "*.h", "*.py")
    result: list[Path] = []
    for ext in exts:
        result.extend(package_path.rglob(ext))
    return sorted(p for p in result if p.is_file())


# ────────── C++ (rclcpp) 패턴 ──────────
# create_publisher<Msg>(topic, qos) / create_subscription<Msg>(topic, qos, cb)
_RE_CPP_CREATE_PUB = re.compile(
    r"create_publisher\s*<[^>]+>\s*\(\s*([^,)]+)\s*,\s*([^)]+)\s*\)",
    re.I | re.S,
)
_RE_CPP_CREATE_SUB = re.compile(
    r"create_subscription\s*<[^>]+>\s*\(\s*([^,)]+)\s*,\s*([^,)]+)\s*,\s*[^)]+\)",
    re.I | re.S,
)

# topic: "/cmd_vel" 또는 "\"/cmd_vel\"" 등
_RE_STRING_LITERAL = re.compile(r'["\']([^"\']*)["\']')

# 파라미터–변수 연결: var = declare_parameter("topic_name", "map") → var → "map"
_RE_PARAM_ASSIGN_STD = re.compile(
    r"(\w+)\s*=\s*[^;]*declare_(?:or_get_)?parameter\s*"
    r"(?:<[^>]+>\s*)?"
    r"\(\s*[\"']([^\"']+)[\"']\s*,\s*std::string\s*[({\s]*[\"']([^\"']*)[\"']\s*[)}\s]*\s*\)",
    re.I | re.S,
)
_RE_PARAM_ASSIGN_QUOTE = re.compile(
    r"(\w+)\s*=\s*[^;]*declare_(?:or_get_)?parameter\s*"
    r"(?:<[^>]+>\s*)?"
    r"\(\s*[\"']([^\"']+)[\"']\s*,\s*[\"']([^\"']*)[\"']\s*\)",
    re.I | re.S,
)
# 동적 파라미터: map_topic_ = declare_or_get_parameter(name_ + "." + "map_topic", std::string("map"))
# 첫 인자에 topic 관련 suffix가 있으면 default를 topic으로 사용
_RE_PARAM_ASSIGN_DYNAMIC = re.compile(
    r"(\w+)\s*=\s*[^;]*declare_(?:or_get_)?parameter\s*"
    r"\(\s*([^)]+),\s*std::string\s*[({\s]*[\"']([^\"']*)[\"']",
    re.I | re.S,
)
_TOPIC_PARAM_SUFFIXES = ("topic", "topic_name", "_topic", "map_topic", "graph", "info")

# QoS(10) 또는 QoS(KeepLast(10))
_RE_QOS_CTOR = re.compile(
    r"(?:rclcpp::)?QoS\s*\(\s*(?:rclcpp::)?(?:KeepLast|KeepAll)\s*\(\s*(\d+)\s*\)\s*\)",
    re.I,
)
# rclcpp::QoS(10) - create_publisher의 [^)]+ 캡처로 "QoS(10" 처럼 )가 잘릴 수 있음
_RE_QOS_CTOR_SIMPLE = re.compile(
    r"(?:rclcpp::)?QoS\s*\(\s*(\d+)\s*\)?",
    re.I,
)

# qos.reliability(ReliabilityPolicy::Reliable)
_RE_RELIABILITY = re.compile(
    r"\.reliability\s*\(\s*(?:rclcpp::)?ReliabilityPolicy\s*::\s*(\w+)\s*\)",
    re.I,
)
_RE_DURABILITY = re.compile(
    r"\.durability\s*\(\s*(?:rclcpp::)?DurabilityPolicy\s*::\s*(\w+)\s*\)",
    re.I,
)
_RE_KEEP_LAST = re.compile(r"\.keep_last\s*\(\s*(\d+)\s*\)", re.I)
_RE_KEEP_ALL = re.compile(r"\.keep_all\s*\(\s*\)", re.I)

# ────────── Python (rclpy) 패턴 ──────────
# create_publisher(msg_type, topic, qos_profile) - positional 3 args
_RE_PY_CREATE_PUB_POS3 = re.compile(
    r"create_publisher\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*([^)]+)\s*\)",
    re.I | re.S,
)
# create_publisher(msg_type, topic) - positional 2 args, qos 기본값
_RE_PY_CREATE_PUB_POS2 = re.compile(
    r"create_publisher\s*\(\s*[^,]+,\s*([^,)]+)\s*\)",
    re.I | re.S,
)
# create_subscription(msg_type, topic, callback, qos_profile) - positional 4 args
_RE_PY_CREATE_SUB_POS4 = re.compile(
    r"create_subscription\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*[^,]+,\s*([^)]+)\s*\)",
    re.I | re.S,
)
# create_subscription(msg_type, topic, callback) - positional 3 args, qos 기본값
_RE_PY_CREATE_SUB_POS3 = re.compile(
    r"create_subscription\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*[^,]+\)",
    re.I | re.S,
)
# create_publisher(topic=..., qos_profile=...) - keyword (topic, qos_profile 추출)
_RE_PY_TOPIC_KW = re.compile(
    r"topic\s*=\s*[\"']([^\"']*)[\"']|topic\s*=\s*(\w+)",
    re.I,
)
_RE_PY_QOS_KW = re.compile(
    r"qos_profile\s*=\s*([^,)]+)",
    re.I | re.S,
)
# create_publisher(..., topic="x", ...) 또는 create_publisher(..., qos_profile=q, ...)
# 함수 호출 전체에서 keyword 인자 추출용
_RE_PY_CREATE_PUB_CALL = re.compile(
    r"create_publisher\s*\(([^)]*(?:\([^)]*\)[^)]*)*)\)",
    re.I | re.S,
)
_RE_PY_CREATE_SUB_CALL = re.compile(
    r"create_subscription\s*\(([^)]*(?:\([^)]*\)[^)]*)*)\)",
    re.I | re.S,
)

# QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=..., depth=10)
_RE_PY_QOS_PROFILE = re.compile(
    r"QoSProfile\s*\(([^)]*)\)",
    re.I | re.S,
)
_RE_PY_RELIABILITY = re.compile(
    r"reliability\s*=\s*(?:ReliabilityPolicy\.)?(\w+)",
    re.I,
)
_RE_PY_DURABILITY = re.compile(
    r"durability\s*=\s*(?:DurabilityPolicy\.)?(\w+)",
    re.I,
)
_RE_PY_DEPTH = re.compile(r"depth\s*=\s*(\d+)", re.I)


def _extract_string_literal(s: str) -> str | None:
    """Extract string literal from expression. '"  /cmd_vel  "' -> '/cmd_vel'."""
    m = _RE_STRING_LITERAL.search(s.strip())
    return m.group(1).strip() if m else None


def _build_param_topic_map(content: str) -> dict[str, str]:
    """
    declare_parameter/declare_or_get_parameter → 변수 대입에서 var → topic 기본값 매핑.
    create_publisher(topic_name)에서 topic_name이 변수일 때 추적용.
    """
    result: dict[str, str] = {}
    for m in _RE_PARAM_ASSIGN_STD.finditer(content):
        var_name, param_name, default_val = m.group(1), m.group(2), m.group(3)
        if not default_val or param_name.lower() in ("plugin", "type", "class"):
            continue
        if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
            continue
        if not _is_valid_topic_value(default_val):
            continue
        result[var_name] = default_val.strip()

    for m in _RE_PARAM_ASSIGN_QUOTE.finditer(content):
        var_name, param_name, default_val = m.group(1), m.group(2), m.group(3)
        if not default_val or param_name.lower() in ("plugin", "type", "class"):
            continue
        if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
            continue
        if not _is_valid_topic_value(default_val):
            continue
        result[var_name] = default_val.strip()

    for m in _RE_PARAM_ASSIGN_DYNAMIC.finditer(content):
        var_name, first_arg, default_val = m.group(1), m.group(2), m.group(3)
        if not default_val:
            continue
        if not any(s in (first_arg or "").lower() for s in _TOPIC_PARAM_SUFFIXES):
            continue
        if not _is_valid_topic_value(default_val):
            continue
        result[var_name] = default_val.strip()

    return result


def _is_valid_topic_value(val: str) -> bool:
    """토픽이 아닌 값(png, yaml, 파일경로 등) 필터."""
    v = (val or "").strip()
    if len(v) < 2:
        return False
    lower = v.lower()
    if lower.startswith(".") or lower.endswith((".png", ".yaml", ".pgm", ".bmp", ".xml")):
        return False
    if "/tmp" in lower or lower.startswith("tmp/"):
        return False
    invalid = ("png", "yaml", "tmp", "foo", "bar", "open_loop", "closed_loop",
               "_raw", "_updates")
    if lower in invalid:
        return False
    if v.startswith("_") and len(v) <= 5:
        return False
    return True


def _cpp_policy_to_dds(reliability: str) -> str:
    """Map rclcpp ReliabilityPolicy to DDS kind."""
    r = (reliability or "").upper()
    if "RELIABLE" in r:
        return "RELIABLE"
    return "BEST_EFFORT"


def _cpp_durability_to_dds(durability: str) -> str:
    """Map rclcpp DurabilityPolicy to DDS kind."""
    d = (durability or "").upper()
    if "TRANSIENT_LOCAL" in d or "TRANSIENTLOCAL" in d:
        return "TRANSIENT_LOCAL"
    if "TRANSIENT" in d:
        return "TRANSIENT"
    if "PERSISTENT" in d:
        return "PERSISTENT"
    return "VOLATILE"


# Chained QoS: QoS(KeepLast(N)).reliability(...).durability(...) (여러 줄 가능)
_RE_CHAINED_QOS = re.compile(
    r"(?:rclcpp::)?QoS\s*\(\s*(?:rclcpp::)?(?:KeepLast|KeepAll)\s*\(\s*(\d+)\s*\)\s*\)"
    r"(?:[\s\S]*?\.reliability\s*\(\s*(?:rclcpp::)?ReliabilityPolicy\s*::\s*(\w+)\s*\))?"
    r"(?:[\s\S]*?\.durability\s*\(\s*(?:rclcpp::)?DurabilityPolicy\s*::\s*(\w+)\s*\))?",
    re.I,
)


def _extract_cpp_qos_from_context(
    content: str,
    qos_expr: str,
    qos_dict: dict[str, dict[str, str]] | None = None,
    package_contents: dict[Path, str] | None = None,
    yaml_params: dict[str, dict[str, str]] | None = None,
) -> dict[str, str]:
    """Extract QoS from context. 3단계: Custom QoS Dictionary 우선 조회."""
    # ③ 멤버 변수: 패키지 전체 컨텍스트 (헤더+소스)
    search_contents = [content]
    if package_contents:
        search_contents = list(package_contents.values())

    out: dict[str, str] = {
        "reliability": "",
        "durability": "",
        "history": "KEEP_LAST",
        "history_depth": "10",
    }

    # 3단계: 호출부 매핑 - MyQoS(), nav2::qos::LatchedPublisherQoS() 등 사전 조회
    if qos_dict:
        resolved = qos_profile_resolver.resolve_qos_from_expression(qos_expr, qos_dict)
        if resolved:
            return resolved

    # 인라인: QoS(10) 또는 QoS(KeepLast(10))
    m = _RE_QOS_CTOR.search(qos_expr)
    if m:
        out["history_depth"] = m.group(1)
        out["history"] = "KEEP_LAST"
    else:
        m2 = _RE_QOS_CTOR_SIMPLE.search(qos_expr)
        if m2:
            out["history_depth"] = m2.group(1)
            out["history"] = "KEEP_LAST"

    # 변수명 추출 (qos_expr이 단순 식별자인 경우)
    var_match = re.match(r"^\s*([a-zA-Z_]\w*)\s*$", qos_expr.strip())
    if var_match:
        var_name = var_match.group(1)
        # 1) ① Composition: var = SensorDataQoS().keep_last(10).reliable()
        if qos_dict:
            for search_content in search_contents:
                assign = re.search(
                    rf"\b{re.escape(var_name)}\s*=\s*([^;]+);",
                    search_content,
                    re.I | re.S,
                )
                if assign:
                    rhs = assign.group(1).strip()
                    chain_qos = qos_profile_resolver.parse_qos_chain_rhs(rhs, qos_dict)
                    if chain_qos:
                        return chain_qos
                    break

        # 2) Chained QoS: var = QoS(KeepLast(N)).reliability(...).durability(...)
        chained = _RE_CHAINED_QOS.search(content)
        if chained:
            out["history_depth"] = chained.group(1)
            out["history"] = "KEEP_LAST"
            if chained.lastindex and chained.lastindex >= 2 and chained.group(2):
                out["reliability"] = _cpp_policy_to_dds(chained.group(2))
            if chained.lastindex and chained.lastindex >= 3 and chained.group(3):
                out["durability"] = _cpp_durability_to_dds(chained.group(3))

        # 3) ③ 멤버 변수: 동일 파일에서만 var_name.reliability(...), .reliable() 검색
        # (다른 파일의 동일 변수명과 혼동 방지)
        for line in content.split("\n"):
            if re.search(rf"\b{re.escape(var_name)}\s*\.reliable\s*\(", line, re.I):
                out["reliability"] = "RELIABLE"
            elif re.search(rf"\b{re.escape(var_name)}\s*\.best_effort\s*\(", line, re.I):
                out["reliability"] = "BEST_EFFORT"
            elif re.search(rf"\b{re.escape(var_name)}\s*\.reliability\s*\(", line, re.I):
                r = _RE_RELIABILITY.search(line)
                if r:
                    out["reliability"] = _cpp_policy_to_dds(r.group(1))
            if re.search(rf"\b{re.escape(var_name)}\s*\.durability\s*\(", line, re.I):
                d = _RE_DURABILITY.search(line)
                if d:
                    out["durability"] = _cpp_durability_to_dds(d.group(1))
            if re.search(rf"\b{re.escape(var_name)}\s*\.keep_last\s*\(", line, re.I):
                k = _RE_KEEP_LAST.search(line)
                if k:
                    out["history"] = "KEEP_LAST"
                    out["history_depth"] = k.group(1)
            if re.search(rf"\b{re.escape(var_name)}\s*\.keep_all\s*\(", line, re.I):
                if _RE_KEEP_ALL.search(line):
                    out["history"] = "KEEP_ALL"

        # 4) QoS(10) 또는 QoS(KeepLast(N)) 형태의 선언
        decl = re.search(
            rf"(?:rclcpp::)?QoS\s+{re.escape(var_name)}\s*\(\s*(\d+)\s*\)",
            content,
            re.I,
        )
        if decl:
            out["history_depth"] = decl.group(1)
            out["history"] = "KEEP_LAST"

        decl2 = re.search(
            rf"(?:rclcpp::)?QoS\s+{re.escape(var_name)}\s*\(\s*"
            rf"(?:rclcpp::)?KeepLast\s*\(\s*(\d+)\s*\)\s*\)",
            content,
            re.I,
        )
        if         decl2:
            out["history_depth"] = decl2.group(1)
            out["history"] = "KEEP_LAST"

    # ② YAML: config/*.yaml 파라미터 병합 (빈 필드 보강)
    if yaml_params:
        out = yaml_qos_loader.merge_yaml_qos_into(out, yaml_params)

    return out


def _extract_py_qos_from_profile(
    content: str, qos_expr: str, qos_dict: dict[str, dict[str, str]] | None = None
) -> dict[str, str]:
    """Python QoSProfile(...) 또는 변수에서 QoS 추출. 3단계: Custom QoS Dictionary 우선 조회."""
    out: dict[str, str] = {
        "reliability": "",
        "durability": "",
        "history": "KEEP_LAST",
        "history_depth": "10",
    }

    # 3단계: 호출부 매핑 - get_latched_qos(), my_qos 등 사전 조회
    if qos_dict and qos_expr.strip():
        resolved = qos_profile_resolver.resolve_qos_from_expression(qos_expr.strip(), qos_dict)
        if resolved:
            return resolved

    # 인라인 QoSProfile(...)
    m = _RE_PY_QOS_PROFILE.search(qos_expr)
    if m:
        args = m.group(1)
        r = _RE_PY_RELIABILITY.search(args)
        if r:
            out["reliability"] = _cpp_policy_to_dds(r.group(1))
        d = _RE_PY_DURABILITY.search(args)
        if d:
            out["durability"] = _cpp_durability_to_dds(d.group(1))
        dep = _RE_PY_DEPTH.search(args)
        if dep:
            out["history_depth"] = dep.group(1)
        return out

    # 변수명인 경우: content에서 var = QoSProfile(...) 검색
    var_match = re.match(r"^\s*([a-zA-Z_]\w*)\s*$", qos_expr.strip())
    if var_match:
        var_name = var_match.group(1)
        decl = re.search(
            rf"\b{re.escape(var_name)}\s*=\s*QoSProfile\s*\(([^)]*)\)",
            content,
            re.I | re.S,
        )
        if decl:
            args = decl.group(1)
            r = _RE_PY_RELIABILITY.search(args)
            if r:
                out["reliability"] = _cpp_policy_to_dds(r.group(1))
            d = _RE_PY_DURABILITY.search(args)
            if d:
                out["durability"] = _cpp_durability_to_dds(d.group(1))
            dep = _RE_PY_DEPTH.search(args)
            if dep:
                out["history_depth"] = dep.group(1)

    return out


@dataclass
class CodeEntity:
    """소스 코드에서 추출한 Pub/Sub 엔티티."""

    source_path: Path
    entity_type: Literal["pub", "sub"]
    topic_name: str
    qos_from_code: dict[str, str]
    line_content: str


def _build_package_param_topic_map(package_contents: dict[Path, str]) -> dict[str, str]:
    """패키지 전체에서 var → topic 매핑을 한 번만 구축 (성능 최적화)."""
    combined: dict[str, str] = {}
    for content in package_contents.values():
        combined.update(_build_param_topic_map(content))
    return combined


def _resolve_topic_from_param(
    topic_str: str,
    param_topic_map: dict[str, str] | None = None,
) -> str | None:
    """파라미터–변수 연결: create_publisher(topic_name)에서 topic_name → declare_parameter 기본값."""
    s = topic_str.strip()
    # topic_name + "_raw" 같은 표현식은 무시 (부분 문자열만 추출됨)
    if "+" in s:
        return None
    topic = _extract_string_literal(s)
    if topic and _is_valid_topic_value(topic):
        return topic
    var_match = re.match(r"^\s*([a-zA-Z_]\w*)\s*$", s)
    if not var_match or not param_topic_map:
        return None
    return param_topic_map.get(var_match.group(1))


def scan_cpp_file(
    path: Path,
    content: str,
    qos_dict: dict[str, dict[str, str]] | None = None,
    package_contents: dict[Path, str] | None = None,
    yaml_params: dict[str, dict[str, str]] | None = None,
    param_topic_map: dict[str, str] | None = None,
) -> list[CodeEntity]:
    """C++ 파일에서 create_publisher, create_subscription 추출."""
    entities: list[CodeEntity] = []

    for m in _RE_CPP_CREATE_PUB.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip()
        topic = _resolve_topic_from_param(topic_str, param_topic_map)
        if topic:
            qos = _extract_cpp_qos_from_context(
                content, qos_expr, qos_dict, package_contents, yaml_params
            )
            entities.append(
                CodeEntity(
                    source_path=path,
                    entity_type="pub",
                    topic_name=topic,
                    qos_from_code=qos,
                    line_content=m.group(0)[:80],
                )
            )

    for m in _RE_CPP_CREATE_SUB.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip()
        topic = _resolve_topic_from_param(topic_str, param_topic_map)
        if topic:
            qos = _extract_cpp_qos_from_context(
                content, qos_expr, qos_dict, package_contents, yaml_params
            )
            entities.append(
                CodeEntity(
                    source_path=path,
                    entity_type="sub",
                    topic_name=topic,
                    qos_from_code=qos,
                    line_content=m.group(0)[:80],
                )
            )

    return entities


def _extract_py_call_args(content: str, create_name: str) -> list[tuple[str | None, str | None]]:
    """
    Extract (topic, qos_expr) from create_publisher or create_subscription calls.

    Supports positional and keyword arguments.
    """
    results: list[tuple[str | None, str | None]] = []
    if create_name == "create_publisher":
        with_qos_re = _RE_PY_CREATE_PUB_POS3
        no_qos_re = _RE_PY_CREATE_PUB_POS2
        call_re = _RE_PY_CREATE_PUB_CALL
    else:
        with_qos_re = _RE_PY_CREATE_SUB_POS4
        no_qos_re = _RE_PY_CREATE_SUB_POS3
        call_re = _RE_PY_CREATE_SUB_CALL

    for m in with_qos_re.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip() if m.lastindex and m.lastindex >= 2 else ""
        topic = _extract_string_literal(topic_str)
        results.append((topic, qos_expr))

    for m in no_qos_re.finditer(content):
        topic_str = m.group(1).strip()
        topic = _extract_string_literal(topic_str)
        results.append((topic, None))

    for m in call_re.finditer(content):
        args = m.group(1)
        if "topic" in args.lower():
            topic_m = re.search(r"topic\s*=\s*[\"']([^\"']*)[\"']", args, re.I)
            topic = topic_m.group(1).strip() if topic_m else None
            qos_m = _RE_PY_QOS_KW.search(args)
            qos_expr = qos_m.group(1).strip() if qos_m else None
            if topic:
                results.append((topic, qos_expr))

    return results


def scan_py_file(
    path: Path, content: str, qos_dict: dict[str, dict[str, str]] | None = None
) -> list[CodeEntity]:
    """Python 파일에서 create_publisher, create_subscription 추출."""
    entities: list[CodeEntity] = []

    for topic, qos_expr in _extract_py_call_args(content, "create_publisher"):
        if topic:
            qos = _extract_py_qos_from_profile(content, qos_expr or "", qos_dict)
            entities.append(
                CodeEntity(
                    source_path=path,
                    entity_type="pub",
                    topic_name=topic,
                    qos_from_code=qos,
                    line_content="create_publisher",
                )
            )

    for topic, qos_expr in _extract_py_call_args(content, "create_subscription"):
        if topic:
            qos = _extract_py_qos_from_profile(content, qos_expr or "", qos_dict)
            entities.append(
                CodeEntity(
                    source_path=path,
                    entity_type="sub",
                    topic_name=topic,
                    qos_from_code=qos,
                    line_content="create_subscription",
                )
            )

    return entities


def scan_code(package_path: Path) -> list[CodeEntity]:
    """
    Scan package for .cpp, .hpp, .h, .py and extract QoS entities.

    1단계+2단계: build_qos_dictionary로 Custom QoS Dictionary 구축
    3단계: create_publisher/subscription 호출 시 사전 조회 후 매핑
    """
    package_path = Path(package_path).resolve()
    entities: list[CodeEntity] = []

    # 1단계+2단계: 전역 심볼 수집 및 정의부 파싱
    qos_dict = qos_profile_resolver.build_qos_dictionary(package_path)

    # ② YAML: config/*.yaml QoS 파라미터
    yaml_params = yaml_qos_loader.load_qos_from_yaml_files(package_path)

    # ③ 멤버 변수: 패키지 전체 컨텍스트 (.h + .cpp)
    src_files = find_source_files(package_path)
    package_contents: dict[Path, str] = {}
    for src_path in src_files:
        try:
            package_contents[src_path] = src_path.read_text(
                encoding="utf-8", errors="ignore"
            )
        except OSError:
            pass

    param_topic_map = _build_package_param_topic_map(package_contents)

    for src_path in src_files:
        content = package_contents.get(src_path, "")
        if not content:
            continue

        if src_path.suffix in (".cpp", ".hpp", ".h"):
            entities.extend(
                scan_cpp_file(
                    src_path, content, qos_dict, package_contents, yaml_params,
                    param_topic_map=param_topic_map,
                )
            )
        elif src_path.suffix == ".py":
            entities.extend(scan_py_file(src_path, content, qos_dict))

    return entities
