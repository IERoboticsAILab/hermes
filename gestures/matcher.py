# gestures/matcher.py
from typing import Any, Dict, Optional, Tuple

from gestures.models import FsrEvent, GestureEvent, GestureState


def _fsr_tuple(f: FsrEvent) -> Tuple[str, str, str]:
    return (f.hand, f.finger, f.action)


def _inputs_available(cmd: Dict[str, Any], event: GestureEvent) -> bool:
    inputs = cmd.get("inputs")
    if not inputs:
        return True

    imu_axes = inputs.get("imu")
    if imu_axes:
        if not event.imu_R:
            return False
        for axis in imu_axes:
            if axis not in event.imu_R:
                return False

    return True


def _match_simple_gesture(spec: Dict[str, Any], event: GestureEvent) -> bool:
    # Reject unknown keys to avoid accidental always-true matches.
    allowed_keys = {
        "L_posture",
        "R_posture",
        "hold_ms",
        "min_hold_ms",
        "L_fsr",
        "R_fsr",
        "R_fsr_sequence",
        "max_gap_ms",
    }
    if any(k not in allowed_keys for k in spec.keys()):
        return False

    if "L_posture" in spec and spec["L_posture"] != event.L_posture:
        return False
    if "R_posture" in spec and spec["R_posture"] != event.R_posture:
        return False

    min_hold_ms = spec.get("hold_ms", spec.get("min_hold_ms"))
    if min_hold_ms is not None and event.hold_ms < int(min_hold_ms):
        return False

    if "L_fsr" in spec:
        if not event.fsr or event.fsr.hand != "L":
            return False
        if spec["L_fsr"]["finger"] != event.fsr.finger:
            return False
        if spec["L_fsr"]["action"] != event.fsr.action:
            return False

    if "R_fsr" in spec:
        if not event.fsr or event.fsr.hand != "R":
            return False
        if spec["R_fsr"]["finger"] != event.fsr.finger:
            return False
        if spec["R_fsr"]["action"] != event.fsr.action:
            return False

    if "R_fsr_sequence" in spec:
        if not event.fsr_sequence:
            return False
        seq_spec = spec["R_fsr_sequence"]
        if len(event.fsr_sequence) != len(seq_spec):
            return False
        for i, s in enumerate(seq_spec):
            if event.fsr_sequence[i].hand != "R":
                return False
            if event.fsr_sequence[i].finger != s["finger"]:
                return False
            if event.fsr_sequence[i].action != s["action"]:
                return False

    return True


def _mode_from_left_posture(registry: Dict[str, Any], L_posture: Optional[str]) -> Optional[str]:
    if not L_posture:
        return None
    for mode_name, mode_def in registry["meta"]["modes"].items():
        if mode_def["selector"].get("L_posture") == L_posture:
            return mode_name
    return None


def _resolve_cmd_vel(cmd: Dict[str, Any], state: GestureState, event: GestureEvent) -> Dict[str, float]:
    imu = event.imu_R or {}
    mapping = cmd.get("mapping", {})

    out: Dict[str, float] = {}
    for axis, axis_cfg in mapping.items():
        raw = float(imu.get(axis, 0.0))
        scale = float(axis_cfg.get("scale", 1.0))
        value = raw * scale
        target = axis_cfg.get("to")

        if target == "vy_or_steer":
            yaw_mode = state.params.get("yaw_mode", "steer")
            if yaw_mode == "steer":
                out["steer"] = value
            else:
                out["vy"] = value
        elif target:
            out[target] = value

    if state.params.get("precision_drive", False):
        for key in ("vx", "vy", "omega", "steer"):
            if key in out:
                out[key] *= 0.35

    return out


def _resolve_stream_value(cmd: Dict[str, Any], event: GestureEvent) -> Optional[float]:
    imu_axes = cmd.get("inputs", {}).get("imu", [])
    if not imu_axes or not event.imu_R:
        return None

    axis = imu_axes[0]
    raw = float(event.imu_R.get(axis, 0.0))

    key = cmd.get("effect", {}).get("key")
    if key == "formation_spacing":
        # Map roughly [-1, 1] pitch into [0.6, 2.2]m spacing.
        raw = max(-1.0, min(1.0, raw))
        return 1.4 + (raw * 0.8)

    return raw


def match_gesture(event: GestureEvent, state: GestureState, registry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    # Update mode from left-hand posture.
    mode = _mode_from_left_posture(registry, event.L_posture)
    if mode:
        state.mode = mode

    # Safety override for discrete gestures only.
    safety = registry["commands"]["safety"]
    safety_items = sorted(safety.items(), key=lambda kv: kv[1].get("priority", 0), reverse=True)
    for cmd_key, cmd in safety_items:
        gesture_spec = cmd.get("gesture", {})

        # Some safety gestures are handled by SafetyEvaluator, not by matcher.
        if "L_accel_palm_up" in gesture_spec or "BOTH_ACCEL_SHAKE" in gesture_spec:
            continue

        if _match_simple_gesture(gesture_spec, event):
            return {
                "domain": "safety",
                "command_key": cmd_key,
                "command_id": cmd["id"],
                "effect": cmd["effect"],
                "resolved": {},
            }

    # Auto-cancel group edit session as soon as the left-hand posture leaves POINT.
    if bool(state.modifiers.get("group_edit_active", False)) and state.mode != "SELECTION":
        return {
            "domain": "selection",
            "command_key": "AUTO_CANCEL_GROUP_ASSIGNMENT",
            "command_id": "select.cancel_group_assignment_auto",
            "effect": {"type": "cancel_group_assignment"},
            "resolved": {"reason": "left_posture_left_point"},
        }

    mode_to_domain = {
        "DRIVE": "drive",
        "SELECTION": "selection",
        "FORMATION": "formation",
        "BEHAVIOR": "behavior",
        "PARAMS": "params",
    }
    domain = mode_to_domain.get(state.mode or "")
    if not domain:
        return None

    commands = registry["commands"][domain]
    group_edit_active = bool(state.modifiers.get("group_edit_active", False)) if domain == "selection" else False

    for cmd_key, cmd in commands.items():
        if domain == "selection":
            cmd_id = cmd.get("id")
            # Selection mode is two-phase:
            # 1) choose group slot, 2) choose robots, 3) confirm.
            if cmd_id == "select.group" and group_edit_active:
                continue
            if cmd_id == "select.robot" and not group_edit_active:
                continue
            if cmd_id == "select.confirm_group_assignment" and not group_edit_active:
                continue

        req = cmd.get("requires", {})
        if req.get("mode") and req["mode"] != state.mode:
            continue
        if req.get("L_posture") and req["L_posture"] != event.L_posture:
            continue
        if req.get("deadman") and not state.deadman_active:
            continue
        if not _inputs_available(cmd, event):
            continue

        resolved: Dict[str, Any] = {}

        # Support "set_while_held" release edge by emitting value=False on RELEASE.
        if cmd.get("effect", {}).get("type") == "set_while_held" and event.fsr:
            gesture_spec = cmd.get("gesture", {})
            target = None
            target_hand = None
            if "R_fsr" in gesture_spec:
                target = gesture_spec["R_fsr"]
                target_hand = "R"
            elif "L_fsr" in gesture_spec:
                target = gesture_spec["L_fsr"]
                target_hand = "L"
            if target and event.fsr.hand == target_hand and event.fsr.finger == target.get("finger"):
                if event.fsr.action == "RELEASE":
                    effect = dict(cmd["effect"])
                    effect["value"] = False
                    return {
                        "domain": domain,
                        "command_key": cmd_key,
                        "command_id": cmd["id"],
                        "effect": effect,
                        "resolved": resolved,
                    }

        if "gesture_map" in cmd:
            if "gesture" in cmd and not _match_simple_gesture(cmd["gesture"], event):
                continue
            if not event.fsr:
                continue
            key = _fsr_tuple(event.fsr)
            binding = cmd["gesture_map"].get(key)
            if binding is None:
                continue
            resolved["binding"] = binding

            if cmd["effect"]["type"] == "set_state":
                return {
                    "domain": domain,
                    "command_key": cmd_key,
                    "command_id": cmd["id"],
                    "effect": {
                        "type": "set_state",
                        "key": cmd["effect"]["key"],
                        "value": binding,
                    },
                    "resolved": resolved,
                }

        elif "gesture" in cmd:
            if not _match_simple_gesture(cmd["gesture"], event):
                continue
        elif "inputs" in cmd:
            # Inputs-only commands are continuous streams (e.g. manual drive).
            pass
        else:
            continue

        if cmd["effect"]["type"] == "cmd_vel_stream":
            resolved["cmd_vel"] = _resolve_cmd_vel(cmd, state, event)

        if cmd["effect"]["type"] == "set_param_stream":
            stream_value = _resolve_stream_value(cmd, event)
            if stream_value is None:
                continue
            resolved["stream_value"] = stream_value

        return {
            "domain": domain,
            "command_key": cmd_key,
            "command_id": cmd["id"],
            "effect": cmd["effect"],
            "resolved": resolved,
        }

    return None
