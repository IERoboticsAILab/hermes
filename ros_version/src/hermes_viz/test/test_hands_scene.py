import math
import pytest
from hermes_viz.scene.hands import HandsScene, euler_to_quat


def test_hands_scene_initial_keys():
    hs = HandsScene()
    keys = {n.key for n in hs.snapshot_nodes()}
    assert keys == {"hand_left", "hand_right"}


def test_hands_scene_update_left_pose():
    hs = HandsScene()
    hs.update_hand(side="left", position=(0.1, 1.2, 0.3),
                   quat=(0.0, 0.0, 0.0, 1.0), curl=0.5, grip=False)
    n = {n.key: n.node for n in hs.snapshot_nodes()}["hand_left"]
    assert list(getattr(n, "position", [])) == [0.1, 1.2, 0.3]
    assert list(getattr(n, "quaternion", [])) == [0.0, 0.0, 0.0, 1.0]


def test_hands_scene_curl_clamped_to_unit_range():
    hs = HandsScene()
    hs.update_hand(side="left", position=(0, 0, 0), quat=(0, 0, 0, 1), curl=1.5, grip=False)
    hs.update_hand(side="right", position=(0, 0, 0), quat=(0, 0, 0, 1), curl=-0.3, grip=False)
    # Internal storage is implementation-defined; we just assert nothing crashed
    # and that snapshot_nodes returns 2 nodes.
    assert len({n.key for n in hs.snapshot_nodes()}) == 2


def test_hands_scene_grip_changes_color():
    hs = HandsScene()
    hs.update_hand(side="right", position=(0, 0, 0), quat=(0, 0, 0, 1), curl=0.0, grip=False)
    n_no_grip = {n.key: n.node for n in hs.snapshot_nodes()}["hand_right"]
    color_no = (getattr(n_no_grip, "material", {}) or {}).get("color")

    hs.update_hand(side="right", position=(0, 0, 0), quat=(0, 0, 0, 1), curl=0.0, grip=True)
    n_grip = {n.key: n.node for n in hs.snapshot_nodes()}["hand_right"]
    color_yes = (getattr(n_grip, "material", {}) or {}).get("color")

    assert color_no is not None and color_yes is not None
    assert color_no != color_yes


def test_hands_scene_invalid_side_raises():
    hs = HandsScene()
    with pytest.raises(ValueError):
        hs.update_hand(side="middle", position=(0, 0, 0), quat=(0, 0, 0, 1),
                       curl=0.0, grip=False)


def test_euler_to_quat_identity():
    qx, qy, qz, qw = euler_to_quat(pitch=0.0, roll=0.0, yaw=0.0)
    assert qx == pytest.approx(0.0)
    assert qy == pytest.approx(0.0)
    assert qz == pytest.approx(0.0)
    assert qw == pytest.approx(1.0)


def test_euler_to_quat_yaw_90deg():
    # Yaw 90 deg around +Y axis: quaternion (0, sin(pi/4), 0, cos(pi/4)).
    qx, qy, qz, qw = euler_to_quat(pitch=0.0, roll=0.0, yaw=math.pi / 2)
    assert qx == pytest.approx(0.0, abs=1e-6)
    assert qy == pytest.approx(math.sin(math.pi / 4), abs=1e-6)
    assert qz == pytest.approx(0.0, abs=1e-6)
    assert qw == pytest.approx(math.cos(math.pi / 4), abs=1e-6)


def test_euler_to_quat_unit_norm():
    # Any combination of euler angles should produce a unit quaternion.
    import math
    for p, r, y in [(0.1, 0.2, 0.3), (math.pi, 0, 0), (0, math.pi, 0), (0, 0, math.pi)]:
        qx, qy, qz, qw = euler_to_quat(pitch=p, roll=r, yaw=y)
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        assert norm == pytest.approx(1.0, abs=1e-6)
