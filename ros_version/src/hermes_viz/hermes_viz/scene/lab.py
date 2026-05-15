"""Lab geometry: 4 inward-facing wall planes + floor + OptiTrack-volume overlay.

Coordinate convention: Three.js right-handed, +Y up. Floor sits on y=0.
Walls are placed at +/- room_w/2 (east/west) and +/- room_d/2 (north/south).
"""

from __future__ import annotations

import math

from vuer.schemas import Plane

from .types import KeyedNode


_WALL_COLOR = "#d6d3cc"   # warm gray
_FLOOR_COLOR = "#bdb9b1"
_OPTITRACK_COLOR = "#c97d4a"


def build_lab_nodes(
    *,
    room_w: float = 6.0,
    room_d: float = 4.0,
    room_h: float = 2.7,
    optitrack_w: float = 4.0,
    optitrack_d: float = 3.0,
) -> list[KeyedNode]:
    for name, v in (("room_w", room_w), ("room_d", room_d), ("room_h", room_h),
                    ("optitrack_w", optitrack_w), ("optitrack_d", optitrack_d)):
        if v <= 0:
            raise ValueError(f"{name} must be > 0 (got {v})")
    if optitrack_w > room_w or optitrack_d > room_d:
        raise ValueError("optitrack volume must fit within the room footprint")

    half_pi = math.pi / 2

    floor = Plane(
        args=[room_w, room_d],
        position=[0, 0, 0],
        rotation=[-half_pi, 0, 0],
        material={"color": _FLOOR_COLOR},
        key="floor",
    )

    wall_north = Plane(
        args=[room_w, room_h],
        position=[0, room_h / 2, -room_d / 2],
        material={"color": _WALL_COLOR},
        key="wall_north",
    )
    wall_south = Plane(
        args=[room_w, room_h],
        position=[0, room_h / 2, room_d / 2],
        rotation=[0, math.pi, 0],
        material={"color": _WALL_COLOR},
        key="wall_south",
    )
    wall_east = Plane(
        args=[room_d, room_h],
        position=[room_w / 2, room_h / 2, 0],
        rotation=[0, -half_pi, 0],
        material={"color": _WALL_COLOR},
        key="wall_east",
    )
    wall_west = Plane(
        args=[room_d, room_h],
        position=[-room_w / 2, room_h / 2, 0],
        rotation=[0, half_pi, 0],
        material={"color": _WALL_COLOR},
        key="wall_west",
    )

    optitrack_volume = Plane(
        args=[optitrack_w, optitrack_d],
        position=[0, 0.001, 0],  # tiny y-offset to avoid z-fighting with floor
        rotation=[-half_pi, 0, 0],
        material={"color": _OPTITRACK_COLOR, "transparent": True, "opacity": 0.15},
        key="optitrack_volume",
    )

    return [
        KeyedNode("floor", floor),
        KeyedNode("wall_north", wall_north),
        KeyedNode("wall_south", wall_south),
        KeyedNode("wall_east", wall_east),
        KeyedNode("wall_west", wall_west),
        KeyedNode("optitrack_volume", optitrack_volume),
    ]
