# gestures/safety.py
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

from gestures.models import GestureEvent, GestureState


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _axis(accel: Dict[str, float], primary: str, fallback: str) -> float:
    return float(accel.get(primary, accel.get(fallback, 0.0)))


@dataclass
class DeadmanImuLatch:
    last_gate: Optional[bool] = None  # True means motion allowed
    palm_up_since_ms: Optional[int] = None
    palm_down_since_ms: Optional[int] = None


@dataclass
class ShakeEStopLatch:
    above_since_ms: Optional[int] = None
    fired: bool = False


class SafetyEvaluator:
    """
    Evaluates safety commands (like IMU deadman) defined in
    COMMAND_REGISTRY["commands"]["safety"]. Emits packets only when gate changes.
    """

    def __init__(self):
        self.deadman_latch = DeadmanImuLatch()
        self.shake_estop_latch = ShakeEStopLatch()

    def tick(
        self,
        event: GestureEvent,
        state: GestureState,
        registry: Dict[str, Any],
        now_ms: int,
        force_deadman_true: bool = False,
    ) -> Optional[Dict[str, Any]]:
        safety_cmds = registry["commands"]["safety"]

        estop_packet = self._tick_shake_estop(event, safety_cmds, now_ms)
        if estop_packet:
            return estop_packet

        cmd = safety_cmds.get("DEADMAN_IMU")
        if not cmd:
            return None

        if force_deadman_true:
            self.deadman_latch.palm_up_since_ms = None
            self.deadman_latch.palm_down_since_ms = None
            return self._set_gate(True, cmd, state, resolved_source="override.deadman_always_true")

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

    def _tick_shake_estop(
        self,
        event: GestureEvent,
        safety_cmds: Dict[str, Any],
        now_ms: int,
    ) -> Optional[Dict[str, Any]]:
        cmd = safety_cmds.get("ESTOP")
        if not cmd:
            return None

        spec = cmd.get("gesture", {}).get("L_ACCEL_SHAKE")
        if spec is None:
            return None

        if not event.accel_L:
            self.shake_estop_latch.above_since_ms = None
            self.shake_estop_latch.fired = False
            return None

        threshold_g = float(spec.get("threshold_g", 0.75))
        release_threshold_g = float(spec.get("release_threshold_g", threshold_g * 0.6))
        hold_ms = int(spec.get("hold_ms", 220))

        l_dyn = abs(
            _norm3(
                _axis(event.accel_L, "AX", "X"),
                _axis(event.accel_L, "AY", "Y"),
                _axis(event.accel_L, "AZ", "Z"),
            )
            - 1.0
        )

        if l_dyn >= threshold_g:
            if self.shake_estop_latch.above_since_ms is None:
                self.shake_estop_latch.above_since_ms = now_ms

            dwell_ms = now_ms - self.shake_estop_latch.above_since_ms
            if (not self.shake_estop_latch.fired) and dwell_ms >= hold_ms:
                self.shake_estop_latch.fired = True
                return {
                    "domain": "safety",
                    "command_key": "ESTOP",
                    "command_id": cmd["id"],
                    "effect": cmd["effect"],
                    "resolved": {
                        "source": "L_ACCEL_SHAKE",
                        "left_dyn_g": round(l_dyn, 3),
                        "threshold_g": threshold_g,
                        "hold_ms": hold_ms,
                        "dwell_ms": dwell_ms,
                    },
                }
        else:
            self.shake_estop_latch.above_since_ms = None
            if l_dyn < release_threshold_g:
                self.shake_estop_latch.fired = False

        return None

    def _set_gate(
        self,
        gate_motion: bool,
        cmd: Dict[str, Any],
        state: GestureState,
        resolved_source: str = "L_accel_palm_up",
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
            "resolved": {"source": resolved_source},
        }
