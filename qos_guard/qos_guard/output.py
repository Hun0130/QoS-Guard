#!/usr/bin/env python3
"""
출력 모듈.
검사 결과를 터미널에 포맷팅하여 출력합니다.
"""
from typing import List, Tuple

# ────────── ANSI 색 코드 ──────────
RED = "\033[31m"
BLUE = "\033[34m"
YELLOW = "\033[33m"
PURPLE = "\033[35m"
RESET = "\033[0m"

SEVERITY_COLOR = {
    "Critical": RED,
    "Conditional": YELLOW,
    "Incidental": PURPLE,
    "Warn": "\033[37m",
}


def color(txt: str, c: str) -> str:
    """텍스트에 ANSI 색 코드를 적용합니다."""
    return f"{c}{txt}{RESET}"


def format_warning(severity: str, side: str, msg: str) -> str:
    """단일 경고를 포맷팅된 문자열로 반환합니다."""
    tag = f"[{severity.upper()}]"
    color_tag = color(tag, SEVERITY_COLOR.get(severity, RED))
    if side == "CROSS":
        return f"{color_tag} {msg}"
    side_tag = color(f"[{side}]", BLUE)
    return f"{color_tag} {side_tag} {msg}"


def print_warnings(warnings: List[Tuple[str, str, str]]) -> None:
    """경고 목록을 출력합니다."""
    for severity, side, msg in warnings:
        print(format_warning(severity, side, msg))


def print_success() -> None:
    """모든 규칙 통과 메시지를 출력합니다."""
    print("✅  All QoS constraints satisfied.")
