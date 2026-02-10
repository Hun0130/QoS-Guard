#!/usr/bin/env python3
"""
CLI 인자 파싱 모듈.

두 가지 모드 지원:
1. XML 페어 모드: pub.xml sub.xml <dds> <ros_version> [publish_period=<Nms>] [rtt=<Nms>]
2. 패키지 모드: <package_path> <dds> <ros_version> [publish_period=<Nms>] [rtt=<Nms>]

dds: fast | cyclone | connext
ros_version: humble | jazzy | kilted
"""
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Union


DDS_CHOICES = ("fast", "cyclone", "connext")
ROS_CHOICES = ("humble", "jazzy", "kilted")

USAGE = f"""Usage:
  XML pair mode:
    qos_guard <pub.xml> <sub.xml> <dds> <ros_version> [publish_period=<Nms>] [rtt=<Nms>]

  Package mode:
    qos_guard <package_path> <dds> <ros_version> [publish_period=<Nms>] [rtt=<Nms>]

  XML list mode (list all XML files in package):
    qos_guard list <package_path>

  dds: {', '.join(DDS_CHOICES)}
  ros_version: {', '.join(ROS_CHOICES)}

Examples:
  qos_guard pub.xml sub.xml fast humble publish_period=40ms rtt=50ms
  qos_guard /path/to/ros2_package fast humble
  qos_guard list /home/hoon/navigation2
"""

DEFAULT_PUBLISH_PERIOD_MS = 40
DEFAULT_RTT_MS = 50


@dataclass
class XmlPairArgs:
    """XML 페어 모드 인자."""

    pub_path: Path
    sub_path: Path
    dds: str
    ros_version: str
    publish_period_ms: int
    rtt_ns: int
    mode: Literal["xml_pair"] = "xml_pair"


@dataclass
class PackageArgs:
    """패키지 모드 인자."""

    package_path: Path
    dds: str
    ros_version: str
    publish_period_ms: int = DEFAULT_PUBLISH_PERIOD_MS
    rtt_ns: int = DEFAULT_RTT_MS * 1_000_000
    mode: Literal["package"] = "package"


@dataclass
class ListXmlArgs:
    """XML 목록 모드 인자."""

    package_path: Path
    mode: Literal["list"] = "list"


CliArgs = Union[XmlPairArgs, PackageArgs]


def parse_list_args(argv: list[str]) -> ListXmlArgs | None:
    """Return ListXmlArgs for list mode, otherwise None."""
    if len(argv) >= 2 and argv[1].strip().lower() == "list":
        if len(argv) < 3:
            sys.exit("[ERROR] list mode requires package_path.\n" + USAGE)
        path = Path(argv[2])
        if not path.exists():
            sys.exit(f"[ERROR] Path not found: {path}")
        return ListXmlArgs(package_path=path)
    return None


def load_text(p: Path) -> str:
    """파일 경로에서 텍스트를 로드합니다."""
    if not p.exists():
        sys.exit(f"[ERROR] File not found: {p}")
    return p.read_text(encoding="utf-8", errors="ignore")


def _parse_period(arg: str) -> int:
    """publish_period=<Nms> 형식에서 N을 파싱합니다."""
    if not arg.startswith("publish_period="):
        raise ValueError("must be publish_period=<Nms>")
    v = arg.split("=", 1)[1].strip().lower()
    if not v.endswith("ms") or not v[:-2].strip().isdigit():
        raise ValueError("value must look like '40ms'")
    return int(v[:-2])


def _parse_rtt(arg: str) -> int:
    """rtt=<Nms> 형식에서 N을 파싱하여 ns 단위로 반환합니다."""
    if not arg.startswith("rtt="):
        raise ValueError("must be rtt=<Nms>")
    v = arg.split("=", 1)[1].strip().lower()
    if not v.endswith("ms") or not v[:-2].strip().isdigit():
        raise ValueError("rtt value must look like '50ms'")
    return int(v[:-2]) * 1_000_000


def _is_kv_arg(s: str) -> bool:
    """publish_period= 또는 rtt= 형식인지 확인."""
    return s.startswith("publish_period=") or s.startswith("rtt=")


def _parse_dds(s: str) -> str:
    v = s.strip().lower()
    if v not in DDS_CHOICES:
        sys.exit(f"[ERROR] dds must be one of {{{', '.join(DDS_CHOICES)}}}, got: {s}")
    return v


def _parse_ros_version(s: str) -> str:
    v = s.strip().lower()
    if v not in ROS_CHOICES:
        sys.exit(f"[ERROR] ros_version must be one of {{{', '.join(ROS_CHOICES)}}}, got: {s}")
    return v


def parse_args(argv: list[str]) -> CliArgs | ListXmlArgs:
    """
    sys.argv를 파싱하여 CliArgs 또는 ListXmlArgs를 반환합니다.

    - list <path>: XML 목록 모드
    - 6개 이상 + 3·4번째가 dds, ros_version: xml_pair 모드
    - 4개 이상 + 2·3번째가 dds, ros_version: package 모드
    """
    list_args = parse_list_args(argv)
    if list_args is not None:
        return list_args

    if len(argv) < 4:
        sys.exit(USAGE)

    # XML pair mode: 6+ args, 3rd/4th are dds, ros_version
    if len(argv) >= 6:
        a3 = argv[3].strip().lower() in DDS_CHOICES
        a4 = argv[4].strip().lower() in ROS_CHOICES
        if a3 and a4:
            pub_path = Path(argv[1])
            sub_path = Path(argv[2])
            dds = _parse_dds(argv[3])
            ros_version = _parse_ros_version(argv[4])

            publish_period_ms = DEFAULT_PUBLISH_PERIOD_MS
            rtt_ns = DEFAULT_RTT_MS * 1_000_000

            for arg in argv[5:]:
                if not _is_kv_arg(arg):
                    sys.exit(f"[ERROR] Invalid argument: {arg}\n{USAGE}")
                try:
                    if arg.startswith("publish_period="):
                        publish_period_ms = _parse_period(arg)
                    else:
                        rtt_ns = _parse_rtt(arg)
                except ValueError as e:
                    sys.exit(f"[ERROR] {e}")

            return XmlPairArgs(
                pub_path=pub_path,
                sub_path=sub_path,
                dds=dds,
                ros_version=ros_version,
                publish_period_ms=publish_period_ms,
                rtt_ns=rtt_ns,
            )

    # Package mode: 4+ args, 2nd/3rd are dds, ros_version
    if len(argv) >= 4:
        b2 = argv[2].strip().lower() in DDS_CHOICES
        b3 = argv[3].strip().lower() in ROS_CHOICES
        if b2 and b3:
            path = Path(argv[1])
            if not path.exists():
                sys.exit(f"[ERROR] Path not found: {path}")

            dds = _parse_dds(argv[2])
            ros_version = _parse_ros_version(argv[3])

            publish_period_ms = DEFAULT_PUBLISH_PERIOD_MS
            rtt_ns = DEFAULT_RTT_MS * 1_000_000

            for arg in argv[4:]:
                if not _is_kv_arg(arg):
                    sys.exit(f"[ERROR] Invalid argument: {arg}\n{USAGE}")
                try:
                    if arg.startswith("publish_period="):
                        publish_period_ms = _parse_period(arg)
                    else:
                        rtt_ns = _parse_rtt(arg)
                except ValueError as e:
                    sys.exit(f"[ERROR] {e}")

            return PackageArgs(
                package_path=path,
                dds=dds,
                ros_version=ros_version,
                publish_period_ms=publish_period_ms,
                rtt_ns=rtt_ns,
            )

    sys.exit(USAGE)
