#!/usr/bin/env python3
"""
CLI 인자 파싱 모듈.
pub.xml, sub.xml, publish_period, rtt 를 파싱하여 반환합니다.
"""
import sys
from dataclasses import dataclass
from pathlib import Path


USAGE = (
    "Usage: qos_guard <pub.xml> <sub.xml> publish_period=<Nms> rtt=<Nms>\n"
    "   or: ros2 run qos_guard qos_guard <pub.xml> <sub.xml> publish_period=<Nms> rtt=<Nms>"
)


@dataclass
class CliArgs:
    """파싱된 CLI 인자."""

    pub_path: Path
    sub_path: Path
    publish_period_ms: int
    rtt_ns: int


def load_text(p: Path) -> str:
    """파일 경로에서 텍스트를 로드합니다."""
    if not p.exists():
        sys.exit(f"[ERROR] File not found: {p}")
    return p.read_text(encoding="utf-8", errors="ignore")


def parse_period(arg: str) -> int:
    """publish_period=<Nms> 형식에서 N을 파싱합니다."""
    if not arg.startswith("publish_period="):
        sys.exit("[ERROR] third argument must be publish_period=<Nms>")
    v = arg.split("=", 1)[1].strip().lower()
    if not v.endswith("ms") or not v[:-2].strip().isdigit():
        sys.exit("[ERROR] value must look like '40ms'")
    return int(v[:-2])


def parse_rtt(arg: str) -> int:
    """rtt=<Nms> 형식에서 N을 파싱하여 ns 단위로 반환합니다."""
    if not arg.startswith("rtt="):
        sys.exit("[ERROR] fourth argument must be rtt=<Nms>")
    v = arg.split("=", 1)[1].strip().lower()
    if not v.endswith("ms") or not v[:-2].strip().isdigit():
        sys.exit("[ERROR] rtt value must look like '50ms'")
    return int(v[:-2]) * 1_000_000


def parse_args(argv: list[str]) -> CliArgs:
    """
    sys.argv를 파싱하여 CliArgs를 반환합니다.
    인자 개수가 맞지 않으면 usage를 출력하고 종료합니다.
    """
    if len(argv) != 5:
        sys.exit(USAGE)

    pub_path = Path(argv[1])
    sub_path = Path(argv[2])
    publish_period_ms = parse_period(argv[3])
    rtt_ns = parse_rtt(argv[4])

    return CliArgs(
        pub_path=pub_path,
        sub_path=sub_path,
        publish_period_ms=publish_period_ms,
        rtt_ns=rtt_ns,
    )
