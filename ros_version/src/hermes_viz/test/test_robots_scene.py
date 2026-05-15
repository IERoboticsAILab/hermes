import pytest
from hermes_viz.scene.robots import RobotsScene, BOT_COLORS


def test_robots_scene_initial_keys():
    rs = RobotsScene(robot_ids=["r1", "r2"], urdf_src="/static/rosbot.urdf", trail_max_len=10)
    keys = {n.key for n in rs.snapshot_nodes()}
    assert {"robot_r1", "robot_r2", "trail_r1", "trail_r2"}.issubset(keys)


def test_robots_scene_update_pose_sets_world_position():
    rs = RobotsScene(robot_ids=["r1"], urdf_src="/static/rosbot.urdf", trail_max_len=10)
    rs.update_pose("r1", x=2.0, y=1.0, yaw=0.5)
    nodes = {n.key: n.node for n in rs.snapshot_nodes()}
    bot = nodes["robot_r1"]
    # world.x = opti.x, world.y = 0, world.z = opti.y
    assert getattr(bot, "position", None) == [2.0, 0.0, 1.0]
    # rotation around Y axis carries yaw
    assert getattr(bot, "rotation", None) == [0.0, 0.5, 0.0]


def test_robots_scene_update_pose_appends_trail_in_world_coords():
    rs = RobotsScene(robot_ids=["r1"], urdf_src="/static/rosbot.urdf", trail_max_len=10)
    rs.update_pose("r1", x=1.0, y=0.0, yaw=0.0)
    rs.update_pose("r1", x=1.5, y=0.0, yaw=0.0)
    assert rs.trail_points("r1") == [(1.0, 0.0), (1.5, 0.0)]


def test_robots_scene_unknown_id_ignored():
    rs = RobotsScene(robot_ids=["r1"], urdf_src="/static/rosbot.urdf", trail_max_len=10)
    rs.update_pose("r99", x=0, y=0, yaw=0)  # must not raise
    assert rs.trail_points("r1") == []


def test_robots_scene_bot_colors_have_4_entries():
    # MVP supports r1..r4. Sanity check the palette.
    for rid in ("r1", "r2", "r3", "r4"):
        assert rid in BOT_COLORS, f"missing color for {rid}"
        assert BOT_COLORS[rid].startswith("#")


def test_robots_scene_resize_trails_affects_all_robots():
    rs = RobotsScene(robot_ids=["r1", "r2"], urdf_src="/static/rosbot.urdf", trail_max_len=10)
    for i in range(15):
        rs.update_pose("r1", x=float(i), y=0, yaw=0)
        rs.update_pose("r2", x=float(i), y=0, yaw=0)
    rs.resize_trails(3)
    assert len(rs.trail_points("r1")) == 3
    assert len(rs.trail_points("r2")) == 3
