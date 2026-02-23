# gesture/matcher.py
from typing import Dict, Any, Optional, Tuple, List
from gesture.models import GestureEvent, GestureState, FsrEvent


def _fsr_tuple(f: FsrEvent) -> Tuple[str, str, str]:
    return (f.hand, f.finger, f.action)


def _match_simple_gesture(spec: Dict[str, Any], event: GestureEvent) -> bool:
    if "L_posture" in spec and spec["L_posture"] != event.L_posture:
        return False
    if "R_posture" in spec and spec["R_posture"] != event.R_posture:
        return False
    if "hold_ms" in spec and event.hold_ms < spec["hold_ms"]:
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

    if "inputs" in spec and "imu" in spec["inputs"]:
        if event.imu is None:
            return False

    return True


def _mode_from_left_posture(registry: Dict[str, Any], L_posture: Optional[str]) -> Optional[str]:
    if not L_posture:
        return None
    for mode_name, mode_def in registry["meta"]["modes"].items():
        if mode_def["selector"].get("L_posture") == L_posture:
            return mode_name
    return None


def match_gesture(event: GestureEvent, state: GestureState, registry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    # Update mode
    mode = _mode_from_left_posture(registry, event.L_posture)
    if mode:
        state.mode = mode

    # Safety override
    safety = registry["commands"]["safety"]
    safety_items = sorted(safety.items(), key=lambda kv: kv[1].get("priority", 0), reverse=True)
    for cmd_key, cmd in safety_items:
        if _match_simple_gesture(cmd["gesture"], event):
            return {"domain": "safety", "command_key": cmd_key, "command_id": cmd["id"],
                    "effect": cmd["effect"], "resolved": {}}

    # Deadman update (if you prefer stateful deadman via packets)
    if event.fsr and event.fsr.hand == "R" and event.fsr.finger == "INDEX" and event.fsr.action == "HOLD":
        state.deadman_active = True
    elif event.fsr and event.fsr.hand == "R" and event.fsr.finger == "INDEX" and event.fsr.action == "RELEASE":
        state.deadman_active = False

    # Mode to domain
    mode_to_domain = {
        "DRIVE": "drive",
        "SELECTION": "selection",
        "FORMATION": "formation",
        "BEHAVIOR": "behavior",
        "PARAMS": "params"
    }
    domain = mode_to_domain.get(state.mode or "", None)
    if not domain:
        return None
    commands = registry["commands"][domain]

    # Modifier example: selection toggle modifier
    if domain == "selection":
        mod = commands.get("TOGGLE_MEMBERSHIP_MODIFIER")
        if mod and event.fsr and _match_simple_gesture(mod["gesture"], event):
            state.selection_op = "toggle"
        else:
            state.selection_op = "replace"

    # Match domain commands
    for cmd_key, cmd in commands.items():
        req = cmd.get("requires", {})
        if req.get("mode") and req["mode"] != state.mode:
            continue
        if req.get("L_posture") and req["L_posture"] != event.L_posture:
            continue
        if req.get("deadman") and not state.deadman_active:
            continue

        if "gesture_map" in cmd:
            if not event.fsr:
                continue
            key = _fsr_tuple(event.fsr)
            resolved = cmd["gesture_map"].get(key)
            if resolved is None:
                continue

            # If this is a set_state command, pass the chosen binding as value
            if cmd["effect"]["type"] == "set_state":
                return {
                    "domain": domain,
                    "command_key": cmd_key,
                    "command_id": cmd["id"],
                    "effect": {"type": "set_state", "key": cmd["effect"]["key"], "value": resolved},
                    "resolved": {"binding": resolved}
                }

            extra = {}
            if cmd["id"] == "select.group":
                extra["selection_op"] = state.selection_op

            return {"domain": domain, "command_key": cmd_key, "command_id": cmd["id"],
                    "effect": cmd["effect"], "resolved": {"binding": resolved, **extra}}

        if "gesture" in cmd and _match_simple_gesture(cmd["gesture"], event):
            return {"domain": domain, "command_key": cmd_key, "command_id": cmd["id"],
                    "effect": cmd["effect"], "resolved": {}}

    return None