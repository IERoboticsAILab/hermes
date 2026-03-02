import pytest

from gestures.models import GestureState
from swarm.behavior_engine import execute_behavior
from swarm.swarm_controller import SwarmController


BEHAVIORS = [
    "PATROL",
    "PATROL_PERIMETER",
    "FOLLOW_PATH",
    "FOLLOW_PATH_LOOP",
    "HOLD_ANCHOR",
    "RETURN_HOME",
    "FOLLOW_ME_TOGGLE",
    "DISPERSE_SCAN",
]


@pytest.mark.parametrize("behavior_name", BEHAVIORS)
def test_behavior_executor_generates_targets_for_selected_robots(behavior_name: str) -> None:
    robot_ids = ["r1", "r2", "r3", "r4"]
    params = {
        "spacing_m": 1.0,
        "speed_scale": 0.55,
        "aggression_scale": 1.0,
        "follow_me_enabled": True,
        "path_waypoints": [[-2.0, 0.0], [0.0, 0.0], [2.0, 0.0]],
    }

    result = execute_behavior(
        behavior_name=behavior_name,
        robot_ids=robot_ids,
        centroid_xy=(1.0, 2.0),
        behavior_params=params,
        heading_rad=0.25,
        home_xy=(0.0, 0.0),
    )

    assert result.metadata.get("status") == "ok"
    assert set(result.targets.keys()) == set(robot_ids)


def test_follow_me_executor_returns_disabled_when_flag_off() -> None:
    result = execute_behavior(
        behavior_name="FOLLOW_ME_TOGGLE",
        robot_ids=["r1", "r2"],
        centroid_xy=(0.0, 0.0),
        behavior_params={
            "spacing_m": 1.0,
            "speed_scale": 0.55,
            "aggression_scale": 1.0,
            "follow_me_enabled": False,
        },
    )

    assert result.targets == {}
    assert result.metadata.get("status") == "disabled"
    assert result.metadata.get("follow_me_enabled") is False


def test_controller_follow_me_is_true_toggle() -> None:
    state = GestureState()
    swarm = SwarmController(robot_ids=["r1", "r2", "r3"])
    swarm.selection = {"r1", "r2"}

    packet = {
        "effect": {"type": "start_behavior"},
        "resolved": {"binding": "FOLLOW_ME_TOGGLE"},
    }

    swarm.handle_packet(packet, state, centroid_xy=(1.0, 1.0))
    assert swarm.follow_me_enabled is True
    assert swarm.active_behavior == "FOLLOW_ME_TOGGLE"
    assert set(swarm.last_targets.keys()) == {"r1", "r2"}

    swarm.handle_packet(packet, state, centroid_xy=(1.0, 1.0))
    assert swarm.follow_me_enabled is False
    assert swarm.active_behavior is None
    assert swarm.last_targets == {}
    assert swarm.behavior_params.get("status") == "disabled"


def test_controller_exposes_runtime_home_and_path_params() -> None:
    state = GestureState()
    swarm = SwarmController(robot_ids=["r1", "r2", "r3"])
    swarm.selection = {"r1", "r2", "r3"}
    swarm.home_xy = (5.0, -3.0)
    swarm.path_waypoints = [(-1.0, 0.0), (0.0, 0.0), (2.0, 1.0)]

    swarm.handle_packet(
        {"effect": {"type": "start_behavior"}, "resolved": {"binding": "FOLLOW_PATH"}},
        state,
        centroid_xy=(0.0, 0.0),
    )
    assert swarm.behavior_params.get("path_waypoints") == [
        {"x": -1.0, "y": 0.0},
        {"x": 0.0, "y": 0.0},
        {"x": 2.0, "y": 1.0},
    ]
    assert swarm.behavior_params.get("home_xy") == {"x": 5.0, "y": -3.0}
