import pytest
from hermes_viz.scene.trail import Trail


def test_trail_starts_empty():
    t = Trail(max_len=5)
    assert t.points() == []


def test_trail_appends_in_order():
    t = Trail(max_len=5)
    t.push(1.0, 2.0)
    t.push(3.0, 4.0)
    assert t.points() == [(1.0, 2.0), (3.0, 4.0)]


def test_trail_evicts_oldest_when_full():
    t = Trail(max_len=3)
    for i in range(5):
        t.push(float(i), float(i))
    assert t.points() == [(2.0, 2.0), (3.0, 3.0), (4.0, 4.0)]


def test_trail_resize_truncates_oldest():
    t = Trail(max_len=5)
    for i in range(5):
        t.push(float(i), 0.0)
    t.resize(2)
    assert t.points() == [(3.0, 0.0), (4.0, 0.0)]


def test_trail_resize_grow_keeps_existing():
    t = Trail(max_len=2)
    t.push(1.0, 0.0)
    t.push(2.0, 0.0)
    t.resize(5)
    t.push(3.0, 0.0)
    assert t.points() == [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]


def test_trail_clear():
    t = Trail(max_len=5)
    t.push(1.0, 1.0)
    t.clear()
    assert t.points() == []


def test_trail_rejects_nonpositive_max_len():
    with pytest.raises(ValueError):
        Trail(max_len=0)
    with pytest.raises(ValueError):
        Trail(max_len=-1)


def test_trail_resize_rejects_nonpositive():
    t = Trail(max_len=3)
    with pytest.raises(ValueError):
        t.resize(0)
