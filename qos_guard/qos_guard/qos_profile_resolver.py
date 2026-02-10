#!/usr/bin/env python3
"""
QoS 프로파일 일반화 해석기.

1단계: 프로젝트 전역 심볼 수집 (Global Symbol Collection)
2단계: 정의부 내부 로직 파싱 (Internal Logic Parsing)
3단계: 호출부 매핑 (Caller Mapping) - code_scanner에서 사전 참조
"""
import re
from pathlib import Path


def _find_matching_brace(
    content: str, start: int, open_c: str = "{", close_c: str = "}"
) -> int | None:
    """Return index of matching close brace after first open brace from start."""
    depth = 0
    i = content.find(open_c, start)
    if i < 0:
        return None
    depth = 1
    i += 1
    while i < len(content):
        if content[i] == open_c:
            depth += 1
        elif content[i] == close_c:
            depth -= 1
            if depth == 0:
                return i
        elif content[i] in '"\'':
            quote = content[i]
            i += 1
            while i < len(content) and content[i] != quote:
                if content[i] == "\\":
                    i += 1
                i += 1
        i += 1
    return None


def _parse_cpp_qos_body(body: str) -> dict[str, str]:
    """
    Parse C++ QoS definition body for method chaining.

    Extract .reliable(), .best_effort(), .durability_volatile(),
    .transient_local(), .keep_last(N), .durability(...) etc.
    """
    out: dict[str, str] = {
        "reliability": "",
        "durability": "",
        "history": "KEEP_LAST",
        "history_depth": "10",
    }

    # KeepLast(N) 또는 KeepAll - 초기화 리스트/본문
    m = re.search(
        r"(?:rclcpp::)?(?:KeepLast|KeepAll)\s*\(\s*(\d+)\s*\)",
        body,
        re.I,
    )
    if m:
        out["history_depth"] = m.group(1)
        out["history"] = "KEEP_LAST" if "keepall" not in body.lower()[:m.start()] else "KEEP_ALL"
    # KeepLast(depth) - 파라미터 사용 시 기본값은 별도 처리
    m_param = re.search(
        r"(?:rclcpp::)?KeepLast\s*\(\s*(\w+)\s*\)",
        body,
        re.I,
    )
    if m_param and not m:
        param_name = m_param.group(1)
        # 같은 함수/생성자 시그니처에서 param = default 추출
        def_match = re.search(
            rf"\b{re.escape(param_name)}\s*=\s*(\d+)",
            body,
            re.I,
        )
        if def_match:
            out["history_depth"] = def_match.group(1)
        out["history"] = "KEEP_LAST"

    # this->reliable() / .reliable() / reliability(ReliabilityPolicy::Reliable)
    if re.search(r"(?:this\s*->|\.)\s*reliable\s*\(", body, re.I):
        out["reliability"] = "RELIABLE"
    elif re.search(r"(?:this\s*->|\.)\s*best_effort\s*\(", body, re.I):
        out["reliability"] = "BEST_EFFORT"
    elif re.search(
            r"\.reliability\s*\(\s*(?:rclcpp::)?ReliabilityPolicy\s*::\s*(\w+)",
            body, re.I):
        r = re.search(r"ReliabilityPolicy\s*::\s*(\w+)", body, re.I)
        if r:
            val = (r.group(1) or "").upper()
            out["reliability"] = "RELIABLE" if "RELIABLE" in val else "BEST_EFFORT"

    # this->durability_volatile() / .durability_volatile()
    if re.search(r"(?:this\s*->|\.)\s*durability_volatile\s*\(", body, re.I):
        out["durability"] = "VOLATILE"
    elif re.search(r"(?:this\s*->|\.)\s*transient_local\s*\(", body, re.I):
        out["durability"] = "TRANSIENT_LOCAL"
    elif re.search(r"(?:this\s*->|\.)\s*transient\s*\(", body, re.I):
        out["durability"] = "TRANSIENT"
    elif re.search(r"(?:this\s*->|\.)\s*persistent\s*\(", body, re.I):
        out["durability"] = "PERSISTENT"
    elif re.search(r"\.durability\s*\(\s*(?:rclcpp::)?DurabilityPolicy\s*::\s*(\w+)", body, re.I):
        d = re.search(r"DurabilityPolicy\s*::\s*(\w+)", body, re.I)
        if d:
            val = (d.group(1) or "").upper()
            if "TRANSIENT_LOCAL" in val or "TRANSIENTLOCAL" in val:
                out["durability"] = "TRANSIENT_LOCAL"
            elif "TRANSIENT" in val:
                out["durability"] = "TRANSIENT"
            elif "PERSISTENT" in val:
                out["durability"] = "PERSISTENT"
            else:
                out["durability"] = "VOLATILE"

    # .keep_last(N)
    k = re.search(r"(?:this\s*->|\.)\s*keep_last\s*\(\s*(\d+)\s*\)", body, re.I)
    if k:
        out["history_depth"] = k.group(1)
        out["history"] = "KEEP_LAST"
    ka = re.search(r"(?:this\s*->|\.)\s*keep_all\s*\(\s*\)", body, re.I)
    if ka:
        out["history"] = "KEEP_ALL"

    return out


def _parse_py_qos_content(content: str) -> dict[str, str]:
    """Python QoSProfile(...) 또는 함수 본문에서 reliability, durability, depth 추출."""
    out: dict[str, str] = {
        "reliability": "",
        "durability": "",
        "history": "KEEP_LAST",
        "history_depth": "10",
    }
    r = re.search(r"reliability\s*=\s*(?:ReliabilityPolicy\.)?(\w+)", content, re.I)
    if r:
        val = (r.group(1) or "").upper()
        out["reliability"] = "RELIABLE" if "RELIABLE" in val else "BEST_EFFORT"
    d = re.search(r"durability\s*=\s*(?:DurabilityPolicy\.)?(\w+)", content, re.I)
    if d:
        val = (d.group(1) or "").upper()
        if "TRANSIENT_LOCAL" in val or "TRANSIENTLOCAL" in val:
            out["durability"] = "TRANSIENT_LOCAL"
        elif "TRANSIENT" in val:
            out["durability"] = "TRANSIENT"
        elif "PERSISTENT" in val:
            out["durability"] = "PERSISTENT"
        else:
            out["durability"] = "VOLATILE"
    dep = re.search(r"depth\s*=\s*(\d+)", content, re.I)
    if dep:
        out["history_depth"] = dep.group(1)
    return out


# ────────── 1단계: 심볼 수집 패턴 ──────────
# class X : public rclcpp::QoS (또는 : rclcpp::QoS)
_RE_CPP_CLASS_QOS = re.compile(
    r"class\s+(\w+)\s*:\s*(?:public\s+)?(?:rclcpp::)?QoS\b",
    re.I,
)
# rclcpp::QoS foo() 또는 QoS foo()
_RE_CPP_FUNC_RET_QOS = re.compile(
    r"(?:rclcpp::)?QoS\s+(\w+)\s*\(",
    re.I,
)
# 생성자 파라미터 기본값: (const int depth = 1)
_RE_CPP_PARAM_DEFAULT = re.compile(
    r"\(\s*(?:const\s+)?(?:int|size_t)\s+(\w+)\s*=\s*(\d+)\s*\)",
    re.I,
)


_RE_CPP_NAMESPACE = re.compile(r"namespace\s+(\w+)\s*\{", re.I)


def _get_namespace_prefix(content: str, before_pos: int) -> str:
    """
    Extract namespace prefix for position before_pos.

    Only includes namespaces whose closing } is after before_pos
    (i.e. we are inside that block). Returns outer::inner:: form.
    """
    head = content[:before_pos]
    # (namespace_name, start_pos) for each namespace, ordered by position
    candidates = [(m.group(1), m.end() - 1) for m in _RE_CPP_NAMESPACE.finditer(head)]
    namespaces: list[str] = []
    for ns_name, brace_start in candidates:
        brace_end = _find_matching_brace(content, brace_start)
        if brace_end is not None and brace_end > before_pos:
            namespaces.append(ns_name)
    return "::".join(namespaces) + "::" if namespaces else ""


def _extract_cpp_qos_symbols(content: str, file_path: Path) -> dict[str, dict[str, str]]:
    """C++ 파일에서 QoS를 반환/생성하는 심볼(클래스, 함수) 수집 및 본문 파싱."""
    result: dict[str, dict[str, str]] = {}

    # 클래스: class X : public rclcpp::QoS { ... }
    for m in _RE_CPP_CLASS_QOS.finditer(content):
        name = m.group(1)
        ns_prefix = _get_namespace_prefix(content, m.start())
        full_name = ns_prefix + name if ns_prefix else name
        start = m.end()
        end = _find_matching_brace(content, start)
        if end is None:
            continue
        body = content[start:end + 1]
        # 생성자 파라미터 기본값 (depth = 1 등)
        param_default = _RE_CPP_PARAM_DEFAULT.search(body)
        default_depth = param_default.group(2) if param_default else "10"
        qos = _parse_cpp_qos_body(body)
        if qos["history_depth"] == "10" and param_default:
            qos["history_depth"] = default_depth
        result[name] = qos
        if full_name != name:
            result[full_name] = qos

    # 함수: rclcpp::QoS foo() { ... }
    for m in _RE_CPP_FUNC_RET_QOS.finditer(content):
        name = m.group(1)
        if name in result:
            continue  # 클래스에서 이미 처리됨 (생성자와 구분)
        # 함수 본문 { 찾기 - 괄호 짝 맞추기
        paren_start = content.find("(", m.start())
        if paren_start < 0:
            continue
        paren_end = paren_start + 1
        depth = 1
        i = paren_end
        while i < len(content) and depth > 0:
            c = content[i]
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
            i += 1
        paren_end = i - 1
        after_sig = content[paren_end + 1:].lstrip()
        if after_sig.startswith("{"):
            brace_end = _find_matching_brace(content, paren_end + 1)
            if brace_end is not None:
                body = content[paren_end + 2:brace_end]
                qos = _parse_cpp_qos_body(body)
                result[name] = qos

    return result


# Python: def foo(): return QoSProfile(...) 또는 var = QoSProfile(...)
_RE_PY_DEF = re.compile(r"def\s+(\w+)\s*\([^)]*\)\s*:", re.I)
_RE_PY_QOS_PROFILE = re.compile(r"QoSProfile\s*\(([^)]*)\)", re.I | re.S)
_RE_PY_RETURN_QOS = re.compile(r"return\s+(?:QoSProfile\s*\([^)]*\)|\w+)", re.I)


def _extract_py_qos_symbols(content: str, file_path: Path) -> dict[str, dict[str, str]]:
    """Python 파일에서 QoSProfile을 반환하는 함수/변수 수집."""
    result: dict[str, dict[str, str]] = {}

    # 모듈 레벨: var = QoSProfile(...)
    for m in _RE_PY_QOS_PROFILE.finditer(content):
        args = m.group(1)
        qos = _parse_py_qos_content(args)
        # 변수명 추출 - 같은 줄에서 var = QoSProfile
        before = content[: m.start()].split("\n")[-1]
        var_m = re.search(r"(\w+)\s*=\s*QoSProfile\s*$", before)
        if var_m:
            result[var_m.group(1)] = qos

    # def foo(): ... return QoSProfile(...) 또는 return qos_var
    for m in _RE_PY_DEF.finditer(content):
        name = m.group(1)
        start = m.end()
        # 함수 본문 (들여쓰기 기준)
        lines = content[start:].split("\n")
        body_lines: list[str] = []
        base_indent = None
        for line in lines:
            if not line.strip():
                if body_lines:
                    body_lines.append(line)
                continue
            stripped = line.lstrip()
            indent = len(line) - len(stripped)
            if base_indent is None:
                base_indent = indent
            if base_indent is not None and indent < base_indent and stripped:
                break
            body_lines.append(line)
        body = "\n".join(body_lines)
        if _RE_PY_RETURN_QOS.search(body):
            # return QoSProfile(...) 직접
            ret_qos = _RE_PY_QOS_PROFILE.search(body)
            if ret_qos:
                qos = _parse_py_qos_content(ret_qos.group(1))
                result[name] = qos
            else:
                # return qos_var - 변수 정의 찾기
                ret_var = re.search(r"return\s+(\w+)\s*$", body, re.M | re.I)
                if ret_var:
                    var_name = ret_var.group(1)
                    for v, q in _extract_py_qos_symbols(body, file_path).items():
                        if v == var_name:
                            result[name] = q
                            break

    return result


# rclcpp built-in profiles (ROS 2 standard)
_RCLCPP_BUILTIN_PROFILES: dict[str, dict[str, str]] = {
    "SensorDataQoS": {
        "reliability": "BEST_EFFORT",
        "durability": "VOLATILE",
        "history": "KEEP_LAST",
        "history_depth": "10",
    },
    "StandardTopicQoS": {
        "reliability": "RELIABLE",
        "durability": "VOLATILE",
        "history": "KEEP_LAST",
        "history_depth": "10",
    },
    "ServicesQoS": {
        "reliability": "RELIABLE",
        "durability": "VOLATILE",
        "history": "KEEP_LAST",
        "history_depth": "10",
    },
    "ParametersQoS": {
        "reliability": "RELIABLE",
        "durability": "VOLATILE",
        "history": "KEEP_LAST",
        "history_depth": "10",
    },
    "ParameterEventsQoS": {
        "reliability": "RELIABLE",
        "durability": "VOLATILE",
        "history": "KEEP_LAST",
        "history_depth": "1000",
    },
    "SystemDefaultsQoS": {
        "reliability": "RELIABLE",
        "durability": "VOLATILE",
        "history": "KEEP_LAST",
        "history_depth": "10",
    },
}


def get_builtin_rclcpp_profiles() -> dict[str, dict[str, str]]:
    """Return rclcpp built-in QoS profiles for composition/inheritance."""
    return dict(_RCLCPP_BUILTIN_PROFILES)


def parse_qos_chain_rhs(rhs: str, qos_dict: dict[str, dict[str, str]]) -> dict[str, str] | None:
    """
    Parse RHS of assignment: BaseQoS().keep_last(10).reliable().

    Returns base from dict + overrides from chain, or None.
    """
    if not rhs or not qos_dict:
        return None
    rhs = rhs.strip()

    # Extract base: SensorDataQoS(), rclcpp::SensorDataQoS(), etc.
    base_m = re.search(r"([\w:]+)\s*\(\s*\)", rhs, re.I)
    if not base_m:
        return None
    base_name = base_m.group(1).strip()
    short = base_name.split("::")[-1] if "::" in base_name else base_name
    base_qos = qos_dict.get(base_name) or qos_dict.get(short)
    if not base_qos:
        return None

    out = dict(base_qos)

    # Apply overrides: .keep_last(N), .reliable(), .best_effort(), etc.
    k = re.search(r"\.keep_last\s*\(\s*(\d+)\s*\)", rhs, re.I)
    if k:
        out["history_depth"] = k.group(1)
        out["history"] = "KEEP_LAST"
    ka = re.search(r"\.keep_all\s*\(\s*\)", rhs, re.I)
    if ka:
        out["history"] = "KEEP_ALL"
    if re.search(r"\.reliable\s*\(", rhs, re.I):
        out["reliability"] = "RELIABLE"
    elif re.search(r"\.best_effort\s*\(", rhs, re.I):
        out["reliability"] = "BEST_EFFORT"
    if re.search(r"\.durability_volatile\s*\(", rhs, re.I):
        out["durability"] = "VOLATILE"
    elif re.search(r"\.transient_local\s*\(", rhs, re.I):
        out["durability"] = "TRANSIENT_LOCAL"
    elif re.search(r"\.transient\s*\(", rhs, re.I):
        out["durability"] = "TRANSIENT"
    elif re.search(r"\.persistent\s*\(", rhs, re.I):
        out["durability"] = "PERSISTENT"
    rel_policy = re.search(
        r"\.reliability\s*\(\s*(?:rclcpp::)?ReliabilityPolicy\s*::\s*(\w+)",
        rhs, re.I,
    )
    if rel_policy:
        val = (rel_policy.group(1) or "").upper()
        out["reliability"] = "RELIABLE" if "RELIABLE" in val else "BEST_EFFORT"
    dur_policy = re.search(
        r"\.durability\s*\(\s*(?:rclcpp::)?DurabilityPolicy\s*::\s*(\w+)",
        rhs, re.I,
    )
    if dur_policy:
        val = (dur_policy.group(1) or "").upper()
        if "TRANSIENT_LOCAL" in val or "TRANSIENTLOCAL" in val:
            out["durability"] = "TRANSIENT_LOCAL"
        elif "TRANSIENT" in val:
            out["durability"] = "TRANSIENT"
        elif "PERSISTENT" in val:
            out["durability"] = "PERSISTENT"
        else:
            out["durability"] = "VOLATILE"

    return out


def build_qos_dictionary(
    package_path: Path,
    extra_paths: list[Path] | None = None,
) -> dict[str, dict[str, str]]:
    """
    Build Custom QoS Dictionary from package sources.

    1단계+2단계: Scan all .hpp, .h, .cpp, .py under package_path
    and extra_paths to collect QoS-producing symbols.

    Args:
        package_path: Package root to scan.
        extra_paths: Additional paths (e.g. workspace, deps).

    Returns:
        Dict mapping symbol name to QoS attributes.

    """
    roots: list[Path] = [Path(package_path).resolve()]
    if extra_paths:
        roots.extend(Path(p).resolve() for p in extra_paths if Path(p).is_dir())

    # ① Composition: rclcpp built-in profiles (SensorDataQoS, StandardTopicQoS 등)
    combined: dict[str, dict[str, str]] = dict(get_builtin_rclcpp_profiles())

    exts = ("*.cpp", "*.hpp", "*.h", "*.py")
    files: list[Path] = []
    for root in roots:
        if not root.is_dir():
            continue
        for ext in exts:
            files.extend(root.rglob(ext))
    files = sorted(set(p for p in files if p.is_file()))

    for fp in files:
        try:
            content = fp.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if fp.suffix in (".cpp", ".hpp", ".h"):
            symbols = _extract_cpp_qos_symbols(content, fp)
        else:
            symbols = _extract_py_qos_symbols(content, fp)
        for sym, qos in symbols.items():
            combined[sym] = qos

    return combined


def resolve_qos_from_expression(
    qos_expr: str, qos_dict: dict[str, dict[str, str]]
) -> dict[str, str] | None:
    """
    Return QoS from Custom QoS Dictionary for given expression.

    3단계 호출부 매핑. qos_expr이 "MyQoS()", "nav2::qos::LatchedPublisherQoS()"
    형식일 때 심볼명을 추출하여 사전에서 조회.
    """
    qos_expr = (qos_expr or "").strip()
    # 심볼명 추출: [\w:]+ 패턴으로 이름 전체 캡처
    #   my_project::QoS() -> my_project::QoS (전체), QoS (short)
    #   nav2::qos::LatchedPublisherQoS() -> nav2::qos::LatchedPublisherQoS, LatchedPublisherQoS
    m = re.search(r"([\w:]+)\s*[\（(]", qos_expr, re.I)
    if m:
        full_sym = m.group(1).strip()
        short_sym = full_sym.split("::")[-1] if "::" in full_sym else full_sym
        # 전체 이름 우선, 없으면 short 이름으로 폴백
        sym = full_sym if full_sym in qos_dict else short_sym
        if sym in qos_dict:
            base = dict(qos_dict[sym])
            # 인자로 depth 전달: LatchedPublisherQoS(5) -> history_depth=5
            depth_arg = re.search(r"\(\s*(\d+)\s*\)?", qos_expr)
            if depth_arg:
                base["history_depth"] = depth_arg.group(1)
            return base
    # 단순 식별자: qos_var
    m2 = re.match(r"^\s*([a-zA-Z_]\w*)\s*$", qos_expr)
    if m2 and m2.group(1) in qos_dict:
        return dict(qos_dict[m2.group(1)])
    return None
