import pytest
from hermes_viz.transforms import (
    parse_raw_input,
    GloveFlex, GloveFsr, LeftImu, GloveState, FusedSample,
)


def test_parse_raw_input_both_gloves_present():
    raw = (
        '{"time_ms": 1000,'
        ' "flex": {"L": {"index": 0.1, "middle": 0.2, "ring": 0.3, "pinky": 0.4},'
        '          "R": {"index": 0.5, "middle": 0.6, "ring": 0.7, "pinky": 0.8}},'
        ' "fsr_pressed": {"L": {"INDEX": true,  "MIDDLE": false, "RING": false, "PINKY": false},'
        '                 "R": {"INDEX": false, "MIDDLE": true,  "RING": false, "PINKY": false}},'
        ' "imu": {"L": {"PITCH": 0.0, "ROLL": 0.0, "YAW": 1.57, "AX": 0.0, "AY": 0.0, "AZ": 9.8}}}'
    )
    s = parse_raw_input(raw)
    assert isinstance(s, FusedSample)
    assert s.time_ms == 1000
    assert s.left_glove_fresh is True
    assert s.left is not None
    assert s.left.flex == GloveFlex(index=0.1, middle=0.2, ring=0.3, pinky=0.4)
    assert s.left.fsr_pressed == GloveFsr(index=True, middle=False, ring=False, pinky=False)
    assert s.left.imu == LeftImu(pitch=0.0, roll=0.0, yaw=1.57, ax=0.0, ay=0.0, az=9.8)
    assert s.right is not None
    assert s.right.flex == GloveFlex(index=0.5, middle=0.6, ring=0.7, pinky=0.8)
    assert s.right.fsr_pressed == GloveFsr(index=False, middle=True, ring=False, pinky=False)
    assert s.right.imu is None  # right glove has no IMU


def test_parse_raw_input_left_only():
    raw = (
        '{"time_ms": 2000,'
        ' "flex": {"L": {"index": 0.1, "middle": 0.2, "ring": 0.3, "pinky": 0.4}},'
        ' "fsr_pressed": {"L": {"INDEX": false, "MIDDLE": false, "RING": false, "PINKY": false}},'
        ' "imu": {"L": {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0, "AX": 0.0, "AY": 0.0, "AZ": 9.8}}}'
    )
    s = parse_raw_input(raw)
    assert s.left_glove_fresh is True
    assert s.left is not None
    assert s.right is None


def test_parse_raw_input_deadman_off_empty_dicts():
    raw = '{"time_ms": 3000, "flex": {}, "fsr_pressed": {}, "imu": {}}'
    s = parse_raw_input(raw)
    assert s is not None
    assert s.time_ms == 3000
    assert s.left_glove_fresh is False
    assert s.left is None
    assert s.right is None


def test_parse_raw_input_right_only_treated_as_deadman_off():
    # Without left glove fresh (no imu.L), nothing should be marked fresh,
    # but the right glove flex/fsr_pressed are still parseable for HUD use if desired.
    # Per spec: empty imu.L => deadman off. We still return a FusedSample for HUD/latency tracking.
    raw = (
        '{"time_ms": 4000,'
        ' "flex": {"R": {"index": 0.5, "middle": 0.6, "ring": 0.7, "pinky": 0.8}},'
        ' "fsr_pressed": {"R": {"INDEX": false, "MIDDLE": false, "RING": false, "PINKY": false}},'
        ' "imu": {}}'
    )
    s = parse_raw_input(raw)
    assert s is not None
    assert s.left_glove_fresh is False  # no imu.L
    assert s.left is None
    assert s.right is not None  # right data still available even when left is stale
    assert s.right.imu is None


def test_parse_raw_input_malformed_returns_none():
    assert parse_raw_input("not json") is None
    assert parse_raw_input('{"flex": {}}') is None  # missing time_ms / fsr_pressed / imu
    assert parse_raw_input('{"time_ms": 0, "flex": "bad", "fsr_pressed": {}, "imu": {}}') is None
    assert parse_raw_input('{"time_ms": "nope", "flex": {}, "fsr_pressed": {}, "imu": {}}') is None
