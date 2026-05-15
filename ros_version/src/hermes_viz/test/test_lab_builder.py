import pytest
from hermes_viz.scene.lab import build_lab_nodes


def test_build_lab_nodes_returns_expected_keys():
    nodes = build_lab_nodes(room_w=6.0, room_d=4.0, room_h=2.7,
                            optitrack_w=4.0, optitrack_d=3.0)
    keys = {n.key for n in nodes}
    expected = {"floor", "wall_north", "wall_south", "wall_east", "wall_west", "optitrack_volume"}
    assert expected.issubset(keys), f"missing: {expected - keys}"


def test_build_lab_nodes_default_values():
    nodes = build_lab_nodes()  # all defaults
    keys = {n.key for n in nodes}
    assert "floor" in keys
    assert "optitrack_volume" in keys


def test_build_lab_nodes_rejects_nonpositive_dims():
    with pytest.raises(ValueError):
        build_lab_nodes(room_w=0, room_d=4, room_h=2.7, optitrack_w=4, optitrack_d=3)
    with pytest.raises(ValueError):
        build_lab_nodes(room_w=6, room_d=4, room_h=2.7, optitrack_w=4, optitrack_d=-1)


def test_build_lab_nodes_optitrack_must_fit_in_room():
    # The OptiTrack volume cannot exceed the room footprint.
    with pytest.raises(ValueError):
        build_lab_nodes(room_w=4.0, room_d=4.0, room_h=2.7,
                        optitrack_w=10.0, optitrack_d=3.0)
