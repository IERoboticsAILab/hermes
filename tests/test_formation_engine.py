import pytest

from swarm.formation_engine import FormationParams, compute_formation_targets


FORMATIONS = [
    "LINE",
    "COLUMN",
    "WEDGE",
    "CIRCLE",
    "GRID",
    "DIAMOND",
    "ECHELON_L",
    "ECHELON_R",
    "UNKNOWN_FORMATION",
]


@pytest.mark.parametrize("formation_type", FORMATIONS)
@pytest.mark.parametrize("n", range(1, 13))
def test_targets_are_centered_on_requested_centroid(formation_type: str, n: int) -> None:
    robot_ids = [f"r{i}" for i in range(1, n + 1)]
    centroid = (3.25, -1.75)
    heading = 1.17
    spacing = 1.2

    out = compute_formation_targets(
        formation_type,
        robot_ids,
        centroid_xy=centroid,
        params=FormationParams(spacing_m=spacing, heading_rad=heading),
    )

    xs = [pose[0] for pose in out.values()]
    ys = [pose[1] for pose in out.values()]
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)

    assert mx == pytest.approx(centroid[0], abs=1e-9)
    assert my == pytest.approx(centroid[1], abs=1e-9)
    assert set(out.keys()) == set(robot_ids)


def test_all_target_headings_match_requested_heading() -> None:
    heading = 0.42
    out = compute_formation_targets(
        "GRID",
        ["r3", "r1", "r2"],
        centroid_xy=(0.0, 0.0),
        params=FormationParams(spacing_m=1.0, heading_rad=heading),
    )

    for _, _, yaw in out.values():
        assert yaw == pytest.approx(heading, abs=1e-12)
