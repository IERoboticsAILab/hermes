# -----------------------------
# Gesture / Command Data Model
# -----------------------------

# Consistent naming (use these everywhere):
# - Hands: "L" or "R"
# - Postures: OPEN, FIST, POINT, TWO, THREE
# - FSR: INDEX, MIDDLE, RING, PINKY
# - FSR actions: TAP, DOUBLE_TAP, HOLD
# - IMU channels: PITCH, ROLL, YAW
#
# "requires" expresses biomechanical constraints:
# e.g. "L_posture": "OPEN" means left must be open; any FSR press must then be on the right.

POSTURES = {
    "OPEN":   {"flex": {"index": "EXT", "middle": "EXT", "ring": "EXT", "pinky": "EXT"}},
    "FIST":   {"flex": {"index": "CURL", "middle": "CURL", "ring": "CURL", "pinky": "CURL"}},
    "POINT":  {"flex": {"index": "EXT", "middle": "CURL", "ring": "CURL", "pinky": "CURL"}},
    "TWO":    {"flex": {"index": "EXT", "middle": "EXT", "ring": "CURL", "pinky": "CURL"}},
    "THREE":  {"flex": {"index": "EXT", "middle": "EXT", "ring": "EXT", "pinky": "CURL"}},
}

FSR_FINGERS = ["INDEX", "MIDDLE", "RING", "PINKY"]
FSR_ACTIONS  = ["TAP", "DOUBLE_TAP", "HOLD"]

IMU_AXES = {
    "PITCH": {"desc": "tilt forward/back"},
    "ROLL":  {"desc": "tilt left/right"},
    "YAW":   {"desc": "wrist rotation"},
}

# -----------------------------
# Modes (Left-hand posture selects mode)
# -----------------------------
MODES = {
    "DRIVE":      {"selector": {"L_posture": "OPEN"},  "desc": "manual driving"},
    "SELECTION":  {"selector": {"L_posture": "POINT"}, "desc": "select robots/subgroups"},
    "FORMATION":  {"selector": {"L_posture": "FIST"},  "desc": "set formation"},
    "BEHAVIOR":   {"selector": {"L_posture": "TWO"},   "desc": "high-level behaviors"},
    "PARAMS":     {"selector": {"L_posture": "THREE"}, "desc": "speed/spacing/aggression"},
}

# -----------------------------
# Groups (A–H mapping, no scrolling)
# -----------------------------
GROUP_BINDINGS = {
    # Single tap => A-D
    ("R", "INDEX",  "TAP"):        "A",
    ("R", "MIDDLE", "TAP"):        "B",
    ("R", "RING",   "TAP"):        "C",
    ("R", "PINKY",  "TAP"):        "D",
    # Double tap => E-H
    ("R", "INDEX",  "DOUBLE_TAP"): "E",
    ("R", "MIDDLE", "DOUBLE_TAP"): "F",
    ("R", "RING",   "DOUBLE_TAP"): "G",
    ("R", "PINKY",  "DOUBLE_TAP"): "H",
}

# -----------------------------
# System-level Safety Commands (work across modes)
# -----------------------------
SAFETY_COMMANDS = {
    "DEADMAN": {
        "id": "safety.deadman",
        "desc": "motion enabled only while held",
        "gesture": {"R_fsr": {"finger": "INDEX", "action": "HOLD"}},
        "effect": {"type": "gate_motion", "value": True},
        "priority": 100,
    },
    "ESTOP": {
        "id": "safety.estop",
        "desc": "immediate stop, cancel tasks",
        "gesture": {"L_posture": "FIST", "R_posture": "FIST", "hold_ms": 1000},
        "effect": {"type": "emergency_stop"},
        "priority": 1000,
    },
    "SOFT_STOP": {
        "id": "safety.soft_stop",
        "desc": "stop movement, preserve task state",
        "gesture": {"L_posture": "OPEN", "R_fsr": {"finger": "MIDDLE", "action": "TAP"}},
        "effect": {"type": "pause"},
        "priority": 500,
    },
    "RESUME": {
        "id": "safety.resume",
        "desc": "resume last task/behavior",
        "gesture": {"L_posture": "OPEN", "R_fsr": {"finger": "MIDDLE", "action": "DOUBLE_TAP"}},
        "effect": {"type": "resume"},
        "priority": 500,
    },
}

# -----------------------------
# Drive Mode commands
# -----------------------------
DRIVE_COMMANDS = {
    "MANUAL_DRIVE": {
        "id": "drive.manual",
        "desc": "drive selected set with right-hand IMU while deadman held",
        "requires": {"mode": "DRIVE", "L_posture": "OPEN", "deadman": True},
        "inputs": {"imu": ["PITCH", "ROLL", "YAW"]},
        "mapping": {
            "PITCH": {"to": "vx", "scale": 1.0},
            "ROLL":  {"to": "vy_or_steer", "scale": 1.0},
            "YAW":   {"to": "omega", "scale": 1.0},
        },
        "effect": {"type": "cmd_vel_stream"},
    },
    "YAW_MODE_TOGGLE": {
        "id": "drive.yaw_toggle",
        "desc": "toggle yaw meaning (rotate-in-place vs steer)",
        "requires": {"mode": "DRIVE", "L_posture": "OPEN"},
        "gesture": {"R_fsr": {"finger": "RING", "action": "TAP"}},
        "effect": {"type": "toggle", "key": "yaw_mode"},
    },
    "PRECISION_DRIVE": {
        "id": "drive.precision",
        "desc": "reduced speed + more smoothing while held",
        "requires": {"mode": "DRIVE", "L_posture": "OPEN"},
        "gesture": {"R_fsr": {"finger": "RING", "action": "HOLD"}},
        "effect": {"type": "set_while_held", "key": "precision_drive", "value": True},
    },
}

# -----------------------------
# Selection Mode commands
# -----------------------------
SELECTION_COMMANDS = {
    "SELECT_GROUP": {
        "id": "select.group",
        "desc": "replace current selection with group A–H via fixed bindings",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        "gesture_map": GROUP_BINDINGS,
        "effect": {"type": "select_group", "op": "replace"},
    },
    "TOGGLE_MEMBERSHIP_MODIFIER": {
        "id": "select.toggle_modifier",
        "desc": "while held, group taps toggle membership instead of replace",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        "gesture": {"L_fsr": {"finger": "INDEX", "action": "HOLD"}},
        "effect": {"type": "modifier", "key": "selection_op", "value": "toggle"},
    },
    "SELECT_ALL": {
        "id": "select.all",
        "desc": "select all robots",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        # sequential chord, not scrolling
        "gesture": {"R_fsr_sequence": [
            {"finger": "INDEX", "action": "TAP"},
            {"finger": "MIDDLE", "action": "TAP"},
        ], "max_gap_ms": 400},
        "effect": {"type": "select_all"},
    },
    "SELECT_NONE": {
        "id": "select.none",
        "desc": "clear selection",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        "gesture": {"R_fsr_sequence": [
            {"finger": "RING", "action": "TAP"},
            {"finger": "PINKY", "action": "TAP"},
        ], "max_gap_ms": 400},
        "effect": {"type": "select_none"},
    },
    "SELECT_NEAREST_IN_DIRECTION": {
        "id": "select.nearest_direction",
        "desc": "select nearest robot in direction of right-hand pointing",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        "gesture": {"R_fsr": {"finger": "INDEX", "action": "HOLD"}},
        "inputs": {"imu": ["YAW", "PITCH"]},
        "effect": {"type": "select_by_cone", "cone_deg": 25, "range_m": 999},
    },
    "LASSO_SELECT_SWEEP": {
        "id": "select.lasso_sweep",
        "desc": "select robots in swept arc while holding",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        "gesture": {"R_fsr": {"finger": "MIDDLE", "action": "HOLD"}},
        "inputs": {"imu": ["YAW"]},
        "effect": {"type": "select_by_sweep_arc", "min_arc_deg": 15},
    },
    "SAVE_SELECTION_TO_GROUP": {
        "id": "select.save_group",
        "desc": "save current selection into a group slot A–H",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        "gesture": {"L_fsr": {"finger": "RING", "action": "HOLD"}},
        "gesture_map": GROUP_BINDINGS,  # right-hand tap chooses target group slot
        "effect": {"type": "save_selection_to_group"},
    },
    "RECALL_LAST_SELECTION": {
        "id": "select.recall_last",
        "desc": "recall last selection set",
        "requires": {"mode": "SELECTION", "L_posture": "POINT"},
        "gesture": {"R_fsr": {"finger": "PINKY", "action": "HOLD"}},
        "effect": {"type": "recall_last_selection"},
    },
}

# -----------------------------
# Formation Mode commands
# -----------------------------
FORMATION_TYPES = {
    "LINE":        {"desc": "line abreast"},
    "COLUMN":      {"desc": "single file"},
    "WEDGE":       {"desc": "V / wedge"},
    "CIRCLE":      {"desc": "perimeter circle"},
    "ECHELON_L":   {"desc": "echelon left"},
    "ECHELON_R":   {"desc": "echelon right"},
    "GRID":        {"desc": "grid"},
    "DIAMOND":     {"desc": "diamond"},
}

FORMATION_BINDINGS = {
    ("R", "INDEX",  "TAP"):        "LINE",
    ("R", "MIDDLE", "TAP"):        "COLUMN",
    ("R", "RING",   "TAP"):        "WEDGE",
    ("R", "PINKY",  "TAP"):        "CIRCLE",
    ("R", "INDEX",  "DOUBLE_TAP"): "ECHELON_L",
    ("R", "MIDDLE", "DOUBLE_TAP"): "ECHELON_R",
    ("R", "RING",   "DOUBLE_TAP"): "GRID",
    ("R", "PINKY",  "DOUBLE_TAP"): "DIAMOND",
}

FORMATION_COMMANDS = {
    "SET_FORMATION_TYPE": {
        "id": "formation.set_type",
        "desc": "choose formation type",
        "requires": {"mode": "FORMATION", "L_posture": "FIST"},
        "gesture_map": FORMATION_BINDINGS,
        "effect": {"type": "set_state", "key": "pending_formation_type", "value": resolved}
    },
    "SET_FORMATION_ORIENTATION": {
        "id": "formation.set_orientation",
        "desc": "rotate formation heading while held",
        "requires": {"mode": "FORMATION", "L_posture": "FIST"},
        "gesture": {"R_fsr": {"finger": "INDEX", "action": "HOLD"}},
        "inputs": {"imu": ["YAW"]},
        "effect": {"type": "set_param_stream", "key": "formation_heading", "value": resolved},
    },
    "SET_SPACING_CONTINUOUS": {
        "id": "formation.set_spacing_cont",
        "desc": "adjust spacing while held using pitch",
        "requires": {"mode": "FORMATION", "L_posture": "FIST"},
        "gesture": {"R_fsr": {"finger": "RING", "action": "HOLD"}},
        "inputs": {"imu": ["PITCH"]},
        "effect": {"type": "set_param_stream", "key": "formation_spacing", "value": resolved},
    },
    "APPLY_FORMATION": {
        "id": "formation.apply",
        "desc": "apply pending formation to current selection",
        "requires": {"mode": "FORMATION", "L_posture": "FIST"},
        "gesture": {"R_fsr": {"finger": "MIDDLE", "action": "HOLD"}, "min_hold_ms": 500},
        "effect": {"type": "apply_formation", "value": resolved},
    },
    "BREAK_FORMATION": {
        "id": "formation.break",
        "desc": "break formation but keep selection",
        "requires": {"mode": "FORMATION", "L_posture": "FIST"},
        "gesture": {"R_fsr": {"finger": "MIDDLE", "action": "DOUBLE_TAP"}},
        "effect": {"type": "break_formation", "value": resolved},
    },
}

# -----------------------------
# Behavior Mode commands
# -----------------------------
BEHAVIOR_BINDINGS = {
    ("R", "INDEX",  "TAP"):  "PATROL",
    ("R", "INDEX",  "DOUBLE_TAP"): "PATROL_PERIMETER",
    ("R", "MIDDLE", "TAP"):  "FOLLOW_PATH",
    ("R", "MIDDLE", "DOUBLE_TAP"): "FOLLOW_PATH_LOOP",
    ("R", "RING",   "TAP"):  "HOLD_ANCHOR",
    ("R", "PINKY",  "TAP"):  "RETURN_HOME",
    ("R", "RING",   "HOLD"): "FOLLOW_ME_TOGGLE",
    ("R", "PINKY",  "HOLD"): "DISPERSE_SCAN",
}

BEHAVIOR_DEFS = {
    "PATROL": {"desc": "patrol area"},
    "PATROL_PERIMETER": {"desc": "patrol perimeter (esp. circle formation)"},
    "FOLLOW_PATH": {"desc": "follow predefined waypoint path"},
    "FOLLOW_PATH_LOOP": {"desc": "loop waypoint path"},
    "HOLD_ANCHOR": {"desc": "hold position / maintain spacing"},
    "RETURN_HOME": {"desc": "return to rally/home point"},
    "FOLLOW_ME_TOGGLE": {"desc": "follow operator/leader beacon toggle"},
    "DISPERSE_SCAN": {"desc": "fan out and scan/explore"},
}

BEHAVIOR_COMMANDS = {
    "START_BEHAVIOR": {
        "id": "behavior.start",
        "desc": "start a behavior based on fixed bindings",
        "requires": {"mode": "BEHAVIOR", "L_posture": "TWO"},
        "gesture_map": BEHAVIOR_BINDINGS,
        "effect": {"type": "start_behavior"},
    },
}

# -----------------------------
# Params Mode commands (speed/spacing/aggression)
# -----------------------------
PARAM_COMMANDS = {
    "SPEED_DOWN_STEP": {
        "id": "params.speed_down",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "INDEX", "action": "TAP"}},
        "effect": {"type": "step_param", "key": "speed_level", "delta": -1, "min": 1, "max": 4},
    },
    "SPEED_UP_STEP": {
        "id": "params.speed_up",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "MIDDLE", "action": "TAP"}},
        "effect": {"type": "step_param", "key": "speed_level", "delta": +1, "min": 1, "max": 4},
    },
    "SPEED_SET_MIN": {
        "id": "params.speed_min",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "INDEX", "action": "HOLD"}},
        "effect": {"type": "set_param", "key": "speed_level", "value": 1},
    },
    "SPEED_SET_MAX": {
        "id": "params.speed_max",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "MIDDLE", "action": "HOLD"}},
        "effect": {"type": "set_param", "key": "speed_level", "value": 4},
    },

    "SPACING_DOWN_STEP": {
        "id": "params.spacing_down",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "RING", "action": "TAP"}},
        "effect": {"type": "step_param", "key": "spacing_level", "delta": -1, "min": 1, "max": 4},
    },
    "SPACING_UP_STEP": {
        "id": "params.spacing_up",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "PINKY", "action": "TAP"}},
        "effect": {"type": "step_param", "key": "spacing_level", "delta": +1, "min": 1, "max": 4},
    },
    "SPACING_SET_MIN": {
        "id": "params.spacing_min",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "RING", "action": "HOLD"}},
        "effect": {"type": "set_param", "key": "spacing_level", "value": 1},
    },
    "SPACING_SET_MAX": {
        "id": "params.spacing_max",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "PINKY", "action": "HOLD"}},
        "effect": {"type": "set_param", "key": "spacing_level", "value": 4},
    },

    "AGGRESSION_DOWN": {
        "id": "params.aggression_down",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "RING", "action": "DOUBLE_TAP"}},
        "effect": {"type": "step_param", "key": "aggression_level", "delta": -1, "min": 1, "max": 4},
    },
    "AGGRESSION_UP": {
        "id": "params.aggression_up",
        "requires": {"mode": "PARAMS", "L_posture": "THREE"},
        "gesture": {"R_fsr": {"finger": "PINKY", "action": "DOUBLE_TAP"}},
        "effect": {"type": "step_param", "key": "aggression_level", "delta": +1, "min": 1, "max": 4},
    },
}

# -----------------------------
# Registry (single place to look things up)
# -----------------------------
COMMAND_REGISTRY = {
    "meta": {
        "postures": POSTURES,
        "modes": MODES,
        "imu_axes": IMU_AXES,
        "formation_types": FORMATION_TYPES,
        "behavior_defs": BEHAVIOR_DEFS,
        "group_bindings": GROUP_BINDINGS,
        "formation_bindings": FORMATION_BINDINGS,
        "behavior_bindings": BEHAVIOR_BINDINGS,
    },
    "commands": {
        "safety": SAFETY_COMMANDS,
        "drive": DRIVE_COMMANDS,
        "selection": SELECTION_COMMANDS,
        "formation": FORMATION_COMMANDS,
        "behavior": BEHAVIOR_COMMANDS,
        "params": PARAM_COMMANDS,
    },
}

__all__ = ["COMMAND_REGISTRY"]