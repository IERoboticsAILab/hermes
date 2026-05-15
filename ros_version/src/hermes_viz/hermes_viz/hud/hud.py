"""HUD state + HTML render. Pure template substitution. No Vuer specifics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional


_TEMPLATE_PATH = Path(__file__).parent / "hud_template.html"


@dataclass(frozen=True)
class HudState:
    deadman: bool
    latency_p50: Optional[float]
    latency_p95: Optional[float]
    rate_left: float
    rate_right: float
    gesture: Optional[str]           # human-readable (e.g. command_key)
    command_id: str                  # namespaced (e.g. "gesture.fist"); "" when unknown
    motors: tuple[int, int, int, int, int, int]
    trails_on: bool
    trail_n: int


def _latency_line(s: HudState) -> str:
    if s.latency_p50 is None or s.latency_p95 is None:
        return "<span style='color:#999'>unavailable</span>"
    return f"p50 {int(s.latency_p50)} ms · p95 {int(s.latency_p95)} ms"


def _motor_dots(motors: tuple[int, ...]) -> str:
    out: list[str] = []
    for m in motors:
        # Map 0..255 to 0.15..1.0 opacity (so off motors are still slightly visible).
        opacity = max(0.15, min(1.0, m / 255.0))
        out.append(
            f"<div class='motor-dot' "
            f"style='width:24px;height:24px;border-radius:50%;"
            f"background:#0c8a8e;opacity:{opacity:.2f}'></div>"
        )
    return "".join(out)


def _trail_text(state: HudState) -> str:
    if not state.trails_on:
        return "off"
    return f"on · N={state.trail_n}"


def render_hud(s: HudState) -> str:
    template = _TEMPLATE_PATH.read_text()
    return (
        template
        .replace("{{DEADMAN_TEXT}}", "LIVE" if s.deadman else "STOPPED")
        .replace("{{DEADMAN_BG}}", "#2a8c4a" if s.deadman else "#a83232")
        .replace("{{LATENCY_LINE}}", _latency_line(s))
        .replace("{{RATE_L}}", f"{s.rate_left:.0f}")
        .replace("{{RATE_R}}", f"{s.rate_right:.0f}")
        .replace("{{GESTURE_LABEL}}", s.gesture or "—")
        .replace("{{COMMAND_ID}}", s.command_id or "")
        .replace("{{MOTOR_DOTS}}", _motor_dots(s.motors))
        .replace("{{TRAIL_TEXT}}", _trail_text(s))
    )
