"""Floating hand scene nodes. MVP: pose + curl-scaled width + grip tint.

The left glove publishes Euler angles (PITCH, ROLL, YAW); use `euler_to_quat`
in the bridge to convert before calling `update_hand`. The right glove has
no IMU — call with the identity quaternion (0, 0, 0, 1).
"""

from __future__ import annotations

import math
from typing import Literal

from vuer.schemas import Box

from .types import KeyedNode


_SKIN_COLOR = "#d49a6a"
_GRIP_COLOR = "#b04a3a"

# Hand mesh dimensions when curl=0 (fully open). Y axis is the palm thickness.
_HAND_W = 0.12   # x: lateral width (shrinks with curl)
_HAND_H = 0.04   # y: palm thickness (constant)
_HAND_D = 0.18   # z: finger length (constant)


Side = Literal["left", "right"]


def euler_to_quat(*, pitch: float, roll: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert ZYX-order Euler (yaw-pitch-roll) to a quaternion (x, y, z, w).

    Convention: yaw = rotation about +Y, pitch = rotation about +X, roll = rotation
    about +Z. This matches the order used by most IMU drivers and is what Three.js
    expects when given quaternion components.
    """
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    # ZYX order Tait-Bryan (yaw-Y, pitch-X, roll-Z) -> quaternion
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * sp * cr + sy * cp * sr
    qy = sy * cp * cr - cy * sp * sr
    qz = cy * cp * sr - sy * sp * cr
    return (qx, qy, qz, qw)


class HandsScene:
    """Mutable state for the two floating hand meshes."""

    def __init__(self) -> None:
        self._state: dict[str, dict] = {
            "left":  {"position": (0.0, 1.0, 0.0), "quat": (0.0, 0.0, 0.0, 1.0),
                      "curl": 0.0, "grip": False},
            "right": {"position": (0.0, 1.0, 0.0), "quat": (0.0, 0.0, 0.0, 1.0),
                      "curl": 0.0, "grip": False},
        }

    def update_hand(self, *,
                    side: Side,
                    position: tuple[float, float, float],
                    quat: tuple[float, float, float, float],
                    curl: float,
                    grip: bool) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right' (got {side!r})")
        s = self._state[side]
        s["position"] = tuple(float(v) for v in position)
        s["quat"] = tuple(float(v) for v in quat)
        s["curl"] = max(0.0, min(1.0, float(curl)))
        s["grip"] = bool(grip)

    def snapshot_nodes(self) -> list[KeyedNode]:
        nodes: list[KeyedNode] = []
        for side in ("left", "right"):
            s = self._state[side]
            sx = _HAND_W - 0.06 * s["curl"]  # width shrinks as curl goes 0 -> 1
            color = _GRIP_COLOR if s["grip"] else _SKIN_COLOR
            box = Box(
                args=[sx, _HAND_H, _HAND_D],
                position=list(s["position"]),
                quaternion=list(s["quat"]),
                material={"color": color},
                key=f"hand_{side}",
            )
            nodes.append(KeyedNode(f"hand_{side}", box))
        return nodes
