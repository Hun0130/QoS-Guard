#!/usr/bin/env python3
"""
QoS 파라미터 YAML 로더.

② YAML 파라미터를 통한 QoS 결정.
config/*.yaml, params/*.yaml 등에서 reliability, durability, depth 추출.
"""
import re
from pathlib import Path


def _read_yaml_safe(path: Path) -> str:
    """Read YAML file without pyyaml dependency."""
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def _parse_yaml_scalar(content: str, key: str) -> str | None:
    """
    Single-file YAML에서 key: value 추출 (간단한 패턴).
    들여쓰기와 중첩 구조를 고려.
    """
    key_esc = re.escape(key)
    # key: value (단순)
    m = re.search(rf"^\s*{key_esc}\s*:\s*[\"']?([^\"'\s#]+)[\"']?\s*(?:#|$)", content, re.M | re.I)
    if m:
        return m.group(1).strip().strip('"\'')
    # key: "value"
    m = re.search(rf"^\s*{key_esc}\s*:\s*[\"']([^\"']*)[\"']", content, re.M | re.I)
    if m:
        return m.group(1).strip()
    return None


def _parse_yaml_node_params(content: str, node_name: str) -> dict[str, str]:
    """
    ros__parameters 아래 node_name.ros__parameters 에서 QoS 관련 추출.
    """
    out: dict[str, str] = {}
    # node_name: / ros__parameters: / **/ 로 시작하는 블록 찾기
    node_esc = re.escape(node_name)
    # Simplistic: 전체 content에서 key: value 검색 (노드 스코프 무시)
    for key in ("reliability", "durability", "depth", "qos_depth", "history_depth"):
        v = _parse_yaml_scalar(content, key)
        if v:
            if key == "depth" or key == "qos_depth" or key == "history_depth":
                if v.isdigit():
                    out["history_depth"] = v
            elif key == "reliability":
                out["reliability"] = "RELIABLE" if "reliable" in v.lower() else "BEST_EFFORT"
            elif key == "durability":
                vupper = v.upper()
                if "TRANSIENT_LOCAL" in vupper or "TRANSIENTLOCAL" in vupper:
                    out["durability"] = "TRANSIENT_LOCAL"
                elif "TRANSIENT" in vupper:
                    out["durability"] = "TRANSIENT"
                elif "PERSISTENT" in vupper:
                    out["durability"] = "PERSISTENT"
                else:
                    out["durability"] = "VOLATILE"
    return out


def load_qos_from_yaml_files(package_path: Path) -> dict[str, dict[str, str]]:
    """
    config/*.yaml, params/*.yaml 등에서 QoS 파라미터 추출.

    Returns:
        {"default": {"reliability": "RELIABLE", "history_depth": "10", ...}}
        code에서 빈 필드 보강 시 사용.
    """
    package_path = Path(package_path).resolve()
    if not package_path.is_dir():
        return {}

    patterns = [
        "config/**/*.yaml", "config/**/*.yml",
        "params/**/*.yaml", "params/**/*.yml",
    ]
    files: list[Path] = []
    for pat in patterns:
        try:
            files.extend(package_path.glob(pat))
        except (OSError, ValueError):
            pass
    files = sorted(set(p for p in files if p.is_file()))

    default: dict[str, str] = {}
    for fp in files:
        content = _read_yaml_safe(fp)
        if not content:
            continue
        qos = _parse_yaml_node_params(content, "")
        for k, v in qos.items():
            if v and k not in default:
                default[k] = v

    return {"default": default} if default else {}


def merge_yaml_qos_into(
    qos: dict[str, str],
    yaml_params: dict[str, dict[str, str]] | None,
    param_key: str | None = None,
) -> dict[str, str]:
    """
    YAML에서 읽은 QoS 파라미터를 code에서 추출한 qos에 병합.

    code에서 빈 필드일 때만 YAML 기본값으로 보강.
    """
    if not yaml_params:
        return qos
    out = dict(qos)
    src = yaml_params.get(param_key or "default") or yaml_params.get("default")
    if src:
        for k, v in src.items():
            if v and not (out.get(k) or "").strip():
                out[k] = v
    return out
