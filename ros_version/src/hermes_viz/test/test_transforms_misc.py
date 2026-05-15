from hermes_viz.transforms import (
    parse_vest_motors, parse_swarm_intent, parse_command_packet,
    VestMotors, SwarmIntent, CommandPacket,
)


# --- vest motors ---

def test_parse_vest_motors_valid():
    assert parse_vest_motors("V1,42,200,0,0,255,128,0") == VestMotors(seq=42, levels=(200, 0, 0, 255, 128, 0))


def test_parse_vest_motors_trailing_newline_ok():
    assert parse_vest_motors("V1,7,1,2,3,4,5,6\n") == VestMotors(seq=7, levels=(1, 2, 3, 4, 5, 6))


def test_parse_vest_motors_wrong_prefix_returns_none():
    assert parse_vest_motors("X1,42,0,0,0,0,0,0") is None


def test_parse_vest_motors_wrong_arity_returns_none():
    assert parse_vest_motors("V1,42,0,0,0") is None
    assert parse_vest_motors("V1,42,0,0,0,0,0,0,0") is None  # too many


def test_parse_vest_motors_non_int_returns_none():
    assert parse_vest_motors("V1,seq,0,0,0,0,0,0") is None
    assert parse_vest_motors("V1,42,abc,0,0,0,0,0") is None


# --- swarm intent ---

def test_parse_swarm_intent_valid():
    raw = ('{"type": "SWARM_INTENT", "schema": "hermes.swarm_intent.v1", "seq": 1,'
           ' "stamp_ms": 5000, "mode": "ACTIVE", "deadman_active": true, "paused": false,'
           ' "selection": ["r1"], "robot_ids": ["r1","r2"],'
           ' "centroid": {"x": 0.0, "y": 0.0}, "active_formation_type": "V",'
           ' "formation_heading": 0.0, "formation_spacing": 0.5,'
           ' "active_behavior": "follow", "behavior_params": {},'
           ' "home_xy": {"x":0,"y":0}, "path_waypoints": [], "drive_cmd_vel": [],'
           ' "groups": {}}')
    s = parse_swarm_intent(raw)
    assert s == SwarmIntent(mode="ACTIVE", deadman_active=True,
                            active_formation_type="V", stamp_ms=5000)


def test_parse_swarm_intent_missing_formation_defaults_empty_string():
    raw = '{"stamp_ms": 1, "mode": "IDLE", "deadman_active": false}'
    s = parse_swarm_intent(raw)
    assert s == SwarmIntent(mode="IDLE", deadman_active=False,
                            active_formation_type="", stamp_ms=1)


def test_parse_swarm_intent_malformed():
    assert parse_swarm_intent("nope") is None
    assert parse_swarm_intent('{"deadman_active": true}') is None  # missing mode/stamp_ms
    assert parse_swarm_intent('{"stamp_ms": "bad", "mode": "X", "deadman_active": true}') is None


# --- command packet ---

def test_parse_command_packet_valid():
    raw = ('{"domain": "teleop", "command_id": "teleop.mode.follow",'
           ' "command_key": "MODE_FOLLOW", "effect": {"type": "set_mode"},'
           ' "resolved": {}}')
    c = parse_command_packet(raw)
    assert c == CommandPacket(domain="teleop", command_id="teleop.mode.follow",
                              command_key="MODE_FOLLOW")


def test_parse_command_packet_minimal():
    # command_id alone is enough to identify a packet; missing pieces filled with empty strings.
    raw = '{"command_id": "gesture.fist"}'
    c = parse_command_packet(raw)
    assert c == CommandPacket(domain="", command_id="gesture.fist", command_key="")


def test_parse_command_packet_malformed():
    assert parse_command_packet("not json") is None
    assert parse_command_packet('{}') is None              # no command_id
    assert parse_command_packet('{"command_id": 42}') is None  # wrong type
