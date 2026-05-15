"""Pure JSON -> dataclass transforms for every H.E.R.M.E.S topic the viz consumes.

No ROS, no Vuer, no IO. Every parse function returns None on malformed input
and never raises. This is the only module that knows topic JSON shapes.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Optional


@dataclass(frozen=True)
class GloveFlex:
    index: float
    middle: float
    ring: float
    pinky: float


@dataclass(frozen=True)
class GloveFsr:
    index: bool
    middle: bool
    ring: bool
    pinky: bool


@dataclass(frozen=True)
class LeftImu:
    pitch: float
    roll: float
    yaw: float
    ax: float
    ay: float
    az: float


@dataclass(frozen=True)
class GloveState:
    flex: GloveFlex
    fsr_pressed: GloveFsr
    imu: Optional[LeftImu]  # right glove always None; left glove always populated when fresh


@dataclass(frozen=True)
class FusedSample:
    time_ms: int
    left_glove_fresh: bool   # True iff imu.L was present (fresh left glove sample)
    left: Optional[GloveState]
    right: Optional[GloveState]


def _f(v: Any) -> float:
    f = float(v)
    if math.isnan(f) or math.isinf(f):
        raise ValueError("nan/inf disallowed")
    return f


def _parse_flex(d: Any) -> Optional[GloveFlex]:
    if not isinstance(d, dict):
        raise ValueError("flex entry must be object")
    return GloveFlex(
        index=_f(d["index"]), middle=_f(d["middle"]),
        ring=_f(d["ring"]),   pinky=_f(d["pinky"]),
    )


def _parse_fsr(d: Any) -> Optional[GloveFsr]:
    if not isinstance(d, dict):
        raise ValueError("fsr_pressed entry must be object")
    return GloveFsr(
        index=bool(d["INDEX"]), middle=bool(d["MIDDLE"]),
        ring=bool(d["RING"]),   pinky=bool(d["PINKY"]),
    )


def _parse_imu(d: Any) -> LeftImu:
    if not isinstance(d, dict):
        raise ValueError("imu entry must be object")
    return LeftImu(
        pitch=_f(d["PITCH"]), roll=_f(d["ROLL"]), yaw=_f(d["YAW"]),
        ax=_f(d["AX"]), ay=_f(d["AY"]), az=_f(d["AZ"]),
    )


def parse_raw_input(raw: str) -> Optional[FusedSample]:
    try:
        d = json.loads(raw)
        if not isinstance(d, dict):
            return None
        time_ms = int(d["time_ms"])
        flex = d["flex"]
        fsr = d["fsr_pressed"]
        imu = d["imu"]
        if not (isinstance(flex, dict) and isinstance(fsr, dict) and isinstance(imu, dict)):
            return None

        left_fresh = "L" in imu  # presence of imu.L = left glove fresh
        left: Optional[GloveState] = None
        right: Optional[GloveState] = None
        if "L" in flex and "L" in fsr and left_fresh:
            left = GloveState(
                flex=_parse_flex(flex["L"]),
                fsr_pressed=_parse_fsr(fsr["L"]),
                imu=_parse_imu(imu["L"]),
            )
        if "R" in flex and "R" in fsr:
            right = GloveState(
                flex=_parse_flex(flex["R"]),
                fsr_pressed=_parse_fsr(fsr["R"]),
                imu=None,  # right glove has no IMU
            )
        return FusedSample(time_ms=time_ms, left_glove_fresh=left_fresh,
                           left=left, right=right)
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        return None


@dataclass(frozen=True)
class RobotPose:
    robot_id: str
    x: float
    y: float
    yaw: float
    stamp_ms: int


def parse_robot_state_beacon(raw: str) -> Optional[RobotPose]:
    try:
        d = json.loads(raw)
        if not isinstance(d, dict):
            return None
        return RobotPose(
            robot_id=str(d["robot_id"]),
            x=_f(d["x"]),
            y=_f(d["y"]),
            yaw=_f(d["yaw"]),
            stamp_ms=int(d["stamp_ms"]),
        )
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        return None


@dataclass(frozen=True)
class VestMotors:
    seq: int
    levels: tuple[int, int, int, int, int, int]


@dataclass(frozen=True)
class SwarmIntent:
    mode: str
    deadman_active: bool
    active_formation_type: str
    stamp_ms: int


@dataclass(frozen=True)
class CommandPacket:
    domain: str
    command_id: str
    command_key: str


def parse_vest_motors(raw: str) -> Optional[VestMotors]:
    parts = raw.strip().split(",")
    if len(parts) != 8 or parts[0] != "V1":
        return None
    try:
        seq = int(parts[1])
        levels = tuple(int(p) for p in parts[2:8])
        return VestMotors(seq=seq, levels=levels)  # type: ignore[arg-type]
    except (ValueError, TypeError):
        return None


def parse_swarm_intent(raw: str) -> Optional[SwarmIntent]:
    try:
        d = json.loads(raw)
        if not isinstance(d, dict):
            return None
        return SwarmIntent(
            mode=str(d["mode"]),
            deadman_active=bool(d["deadman_active"]),
            active_formation_type=str(d.get("active_formation_type", "")),
            stamp_ms=int(d["stamp_ms"]),
        )
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        return None


def parse_command_packet(raw: str) -> Optional[CommandPacket]:
    try:
        d = json.loads(raw)
        if not isinstance(d, dict):
            return None
        cmd_id = d["command_id"]
        if not isinstance(cmd_id, str):
            return None
        return CommandPacket(
            domain=str(d.get("domain", "")),
            command_id=cmd_id,
            command_key=str(d.get("command_key", "")),
        )
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        return None
