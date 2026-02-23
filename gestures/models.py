# gesture/models.py
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List


@dataclass
class FsrEvent:
    hand: str                  # "L" or "R"
    finger: str                # "INDEX" | "MIDDLE" | "RING" | "PINKY"
    action: str                # "TAP" | "DOUBLE_TAP" | "HOLD" | "PRESS" | "RELEASE"
    is_pressed: bool = False
    duration_ms: int = 0       # valid for HOLD
    timestamp_ms: int = 0


@dataclass
class GestureEvent:
    L_posture: Optional[str] = None
    R_posture: Optional[str] = None
    fsr: Optional[FsrEvent] = None
    fsr_sequence: Optional[List[FsrEvent]] = None
    imu: Optional[Dict[str, float]] = None  # {"PITCH":..., "ROLL":..., "YAW":...}
    hold_ms: int = 0


@dataclass
class GestureState:
    mode: Optional[str] = None
    deadman_active: bool = False
    modifiers: Dict[str, Any] = field(default_factory=dict)
    params: Dict[str, Any] = field(default_factory=lambda: {
        "speed_level": 2,
        "spacing_level": 2,
        "aggression_level": 2,
        "yaw_mode": "steer",
        "precision_drive": False,
        "paused": False
    })
    selection_op: str = "replace"