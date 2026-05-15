"""ROSbot scene state: 4 URDF instances + per-bot motion trails.

Coordinate mapping: OptiTrack (x, y, yaw) -> Vuer world (x, 0, y) with
rotation around +Y carrying yaw. Robots sit on the floor (world.y = 0).
"""

from __future__ import annotations

from typing import Iterable

from vuer.schemas import Urdf, Line

from .types import KeyedNode
from .trail import Trail


# Per-bot accent colors used by the trail Line.
BOT_COLORS: dict[str, str] = {
    "r1": "#0c8a8e",  # teal
    "r2": "#d49a3c",  # amber
    "r3": "#d05a6e",  # rose
    "r4": "#3f3f9e",  # indigo
}


_FLOOR_Y = 0.0
_TRAIL_Y = 0.02   # slight lift to avoid z-fighting with the floor plane


class RobotsScene:
    """Mutable scene state for the swarm. snapshot_nodes() returns the current Vuer nodes."""

    def __init__(self, *, robot_ids: Iterable[str], urdf_src: str, trail_max_len: int):
        if trail_max_len < 1:
            raise ValueError("trail_max_len must be >= 1")
        self._ids: list[str] = list(robot_ids)
        if not self._ids:
            raise ValueError("at least one robot_id required")
        self._urdf_src = urdf_src
        self._poses: dict[str, tuple[float, float, float]] = {rid: (0.0, 0.0, 0.0) for rid in self._ids}
        self._trails: dict[str, Trail] = {rid: Trail(max_len=trail_max_len) for rid in self._ids}

    def update_pose(self, robot_id: str, x: float, y: float, yaw: float) -> None:
        if robot_id not in self._poses:
            return
        self._poses[robot_id] = (float(x), float(y), float(yaw))
        self._trails[robot_id].push(x, y)

    def trail_points(self, robot_id: str) -> list[tuple[float, float]]:
        return self._trails[robot_id].points() if robot_id in self._trails else []

    def resize_trails(self, max_len: int) -> None:
        for t in self._trails.values():
            t.resize(max_len)

    def snapshot_nodes(self) -> list[KeyedNode]:
        nodes: list[KeyedNode] = []
        for rid in self._ids:
            x, y, yaw = self._poses[rid]
            bot = Urdf(
                src=self._urdf_src,
                position=[x, _FLOOR_Y, y],
                rotation=[0.0, yaw, 0.0],
                key=f"robot_{rid}",
            )
            nodes.append(KeyedNode(f"robot_{rid}", bot))

            color = BOT_COLORS.get(rid, "#888888")
            pts_world = [[px, _TRAIL_Y, py] for px, py in self._trails[rid].points()]
            if len(pts_world) >= 2:
                line = Line(points=pts_world, color=color, key=f"trail_{rid}")
            else:
                # Empty / single-point trail: a zero-length stub far below the floor,
                # invisible (opacity=0). Keeps the key present so upsert() always finds it.
                line = Line(points=[[0.0, -1.0, 0.0], [0.0, -1.0, 0.0]],
                            color=color, opacity=0.0, key=f"trail_{rid}")
            nodes.append(KeyedNode(f"trail_{rid}", line))
        return nodes
