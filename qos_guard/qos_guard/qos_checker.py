#!/usr/bin/env python3
"""
QoS Guard - DDS QoS 프로파일 검증 도구.

CLI 인자 파싱, XML 파싱, 규칙 검사, 출력을 각 모듈에 위임합니다.
"""
import sys

from . import cli
from . import output
from . import rules
from . import xml_parser


def main() -> None:
    """메인 진입점."""
    args = cli.parse_args(sys.argv)

    pub_xml = cli.load_text(args.pub_path)
    sub_xml = cli.load_text(args.sub_path)

    pub_q = xml_parser.parse_profile(pub_xml)
    sub_q = xml_parser.parse_profile(sub_xml)

    ctx = rules.CheckContext(
        publish_period_ms=args.publish_period_ms,
        rtt_ns=args.rtt_ns,
    )

    warnings = rules.run_checks(pub_xml, sub_xml, pub_q, sub_q, ctx)

    if warnings:
        output.print_warnings(warnings)
        sys.exit(0)
    output.print_success()


if __name__ == "__main__":
    main()
