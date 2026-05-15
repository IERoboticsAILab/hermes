from hermes_viz.transforms import parse_robot_state_beacon, RobotPose


def test_parse_robot_state_beacon_valid():
    raw = ('{"schema": "hermes.robot_state_beacon.v1", "stamp_ms": 12345,'
           ' "robot_id": "r2", "frame_id": "map",'
           ' "x": 1.5, "y": -0.25, "yaw": 1.57, "vx": 0.0, "vy": 0.0}')
    p = parse_robot_state_beacon(raw)
    assert p == RobotPose(robot_id="r2", x=1.5, y=-0.25, yaw=1.57, stamp_ms=12345)


def test_parse_robot_state_beacon_ignores_extra_fields():
    # frame_id, vx, vy, schema are not part of RobotPose; they should be ignored, not rejected.
    raw = ('{"schema": "hermes.robot_state_beacon.v1", "stamp_ms": 1, "robot_id": "r1",'
           ' "frame_id": "world", "x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.1, "vy": 0.2,'
           ' "extra_field_we_dont_care_about": 42}')
    p = parse_robot_state_beacon(raw)
    assert p is not None and p.robot_id == "r1"


def test_parse_robot_state_beacon_unknown_id_accepted():
    # Bridge filters unknown IDs at a higher layer; the transform is dumb on purpose.
    raw = '{"stamp_ms": 0, "robot_id": "r99", "x": 0, "y": 0, "yaw": 0}'
    p = parse_robot_state_beacon(raw)
    assert p is not None and p.robot_id == "r99"


def test_parse_robot_state_beacon_malformed_returns_none():
    assert parse_robot_state_beacon("nope") is None
    assert parse_robot_state_beacon('{"robot_id": "r1"}') is None        # missing fields
    assert parse_robot_state_beacon('{"stamp_ms": 0, "robot_id": "r1", "x": "oops", "y": 0, "yaw": 0}') is None
    assert parse_robot_state_beacon('{"stamp_ms": 0, "robot_id": "r1", "x": 0, "y": 0, "yaw": null}') is None
