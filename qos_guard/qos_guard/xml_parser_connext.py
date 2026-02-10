#!/usr/bin/env python3
"""
Connext DDS QoS 프로파일 XML 파서 (스텁).

RTI Connext DDS XML 스키마 파싱은 향후 구현 예정입니다.
"""
from typing import Dict


def parse_profile(xml: str) -> Dict[str, str | list[str]]:
    """XML 문자열에서 QoS 프로파일을 파싱하여 딕셔너리로 반환합니다."""
    raise NotImplementedError(
        "Connext DDS XML parser is not yet implemented. "
        "Use dds=fast for Fast DDS profiles."
    )
