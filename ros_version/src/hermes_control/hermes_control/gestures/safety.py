# gestures/safety.py
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

from hermes_control.gestures.models import GestureEvent, GestureState


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _axis(accel: Dict[str, float], primary: str, fallback: str) -> float:
    return float(accel.get(primary, accel.get(fallback, 0.0)))


@dataclass
class DeadmanImuLatch:
    last_gate: Optional[bool] = None  # True means motion allowed
    palm_up_since_ms: Optional[int] = None
    palm_down_since_ms: Optional[int] = None


class SafetyEvaluator:
    """
    Evaluates safety commands (like IMU deadman) defined in
    COMMAND_REGISTRY["commands"]["safety"]. Emits packets only when gate changes.
    """

    def __init__(self):
        self.deadman_latch = DeadmanImuLatch()

    def tick(
        self,
        event: GestureEvent,
        state: GestureState,
        registry: Dict[str, Any],
        now_ms: int,
    ) -> Optional[Dict[str, Any]]:
        safety_cmds = registry["commands"]["safety"]

        cmd = safety_cmds.get("DEADMAN_IMU")
        if not cmd:
            return None

        spec = cmd.get("gesture", {}).get("L_accel_palm_up")
        if spec is None:
            return None

        # Missing accel input is fail-safe OFF.
        if not event.accel_L:
            return self._set_gate(False, cmd, state)

        ax = _axis(event.accel_L, "AX", "X")
        ay = _axis(event.accel_L, "AY", "Y")
        az = _axis(event.accel_L, "AZ", "Z")

        n = _norm3(ax, ay, az)
        if n < 1e-6:
            return self._set_gate(False, cmd, state)

        azn = az / n
        threshold = float(spec.get("threshold_g", 0.80))
        hyst = float(spec.get("hysteresis_g", 0.10))
        sign = int(spec.get("palm_up_az_sign", -1))
        debounce_ms = int(spec.get("debounce_ms", 120))

        palm_up_now = (azn * sign) >= threshold
        palm_down_now = (azn * sign) <= (threshold - hyst)

        gate_motion = state.deadman_active if self.deadman_latch.last_gate is not None else True

        if palm_up_now:
            if self.deadman_latch.palm_up_since_ms is None:
                self.deadman_latch.palm_up_since_ms = now_ms
            if (now_ms - self.deadman_latch.palm_up_since_ms) >= debounce_ms:
                gate_motion = False
            self.deadman_latch.palm_down_since_ms = None

        elif palm_down_now:
            if self.deadman_latch.palm_down_since_ms is None:
                self.deadman_latch.palm_down_since_ms = now_ms
            if (now_ms - self.deadman_latch.palm_down_since_ms) >= debounce_ms:
                gate_motion = True
            self.deadman_latch.palm_up_since_ms = None

        return self._set_gate(gate_motion, cmd, state)

    def _set_gate(
        self,
        gate_motion: bool,
        cmd: Dict[str, Any],
        state: GestureState,
    ) -> Optional[Dict[str, Any]]:
        if self.deadman_latch.last_gate is not None and self.deadman_latch.last_gate == gate_motion:
            state.deadman_active = gate_motion
            return None

        self.deadman_latch.last_gate = gate_motion
        state.deadman_active = gate_motion

        return {
            "domain": "safety",
            "command_key": "DEADMAN_IMU",
            "command_id": cmd["id"],
            "effect": {"type": "gate_motion", "value": gate_motion},
            "resolved": {"source": "L_accel_palm_up"},
        }
