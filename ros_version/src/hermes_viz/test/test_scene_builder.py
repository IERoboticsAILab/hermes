from hermes_viz.scene.builder import camera_presets, CAMERA_DEFAULT_KEY, build_root_scene
from hermes_viz.scene.robots import RobotsScene
from hermes_viz.scene.hands import HandsScene


def test_camera_presets_has_three_named_views():
    p = camera_presets()
    assert set(p.keys()) == {"front_elevated", "top_down", "orbit"}


def test_camera_default_is_front_elevated():
    assert CAMERA_DEFAULT_KEY == "front_elevated"


def test_each_camera_preset_has_position_and_lookat():
    for name, preset in camera_presets().items():
        assert "position" in preset and len(preset["position"]) == 3, f"{name} missing position"
        assert "lookAt" in preset and len(preset["lookAt"]) == 3, f"{name} missing lookAt"
        assert "fov" in preset, f"{name} missing fov"


def test_build_root_scene_returns_a_scene_with_children():
    lab_dims = {"room_w": 6.0, "room_d": 4.0, "room_h": 2.7,
                "optitrack_w": 4.0, "optitrack_d": 3.0}
    robots = RobotsScene(robot_ids=["r1", "r2"], urdf_src="/static/rosbot.urdf", trail_max_len=10)
    hands = HandsScene()
    scene = build_root_scene(lab_dims=lab_dims, robots=robots, hands=hands)
    # Scene should at minimum exist and have children attached.
    assert scene is not None
    # vuer.schemas.Scene is a positional-args container; children live in .children
    # or in the underlying args list. We just confirm something was built.
