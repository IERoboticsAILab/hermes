import math
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Sequence, Tuple

from swarm.formation_engine import FormationParams, compute_formation_targets

TargetMap = Dict[str, Tuple[float, float, float]]


@dataclass
class BehaviorResult:
    targets: TargetMap
    metadata: Dict[str, Any]


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _sorted_ids(robot_ids: Sequence[str]) -> List[str]:
    return sorted(str(rid) for rid in robot_ids)


def _rotate_local(local_xy: Tuple[float, float], heading_rad: float) -> Tuple[float, float]:
    x, y = local_xy
    ch = math.cos(heading_rad)
    sh = math.sin(heading_rad)
    return (ch * x - sh * y, sh * x + ch * y)


def _world_from_local(
    local_points: List[Tuple[float, float]],
    centroid_xy: Tuple[float, float],
    heading_rad: float,
) -> List[Tuple[float, float]]:
    cx, cy = centroid_xy
    world: List[Tuple[float, float]] = []
    for p in local_points:
        rx, ry = _rotate_local(p, heading_rad)
        world.append((cx + rx, cy + ry))
    return world


def _heading_to(src_xy: Tuple[float, float], dst_xy: Tuple[float, float], default: float) -> float:
    dx = dst_xy[0] - src_xy[0]
    dy = dst_xy[1] - src_xy[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return default
    return math.atan2(dy, dx)


def _waypoints_from_param(value: Any) -> List[Tuple[float, float]]:
    if not isinstance(value, list):
        return []

    out: List[Tuple[float, float]] = []
    for item in value:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            out.append((_as_float(item[0], 0.0), _as_float(item[1], 0.0)))
            continue
        if isinstance(item, Mapping):
            if "x" in item and "y" in item:
                out.append((_as_float(item["x"], 0.0), _as_float(item["y"], 0.0)))
            elif "X" in item and "Y" in item:
                out.append((_as_float(item["X"], 0.0), _as_float(item["Y"], 0.0)))
    return out


def _serialize_waypoints(waypoints: List[Tuple[float, float]]) -> List[Dict[str, float]]:
    return [{"x": float(x), "y": float(y)} for x, y in waypoints]


def execute_behavior(
    behavior_name: str,
    robot_ids: Sequence[str],
    centroid_xy: Tuple[float, float],
    behavior_params: Mapping[str, Any],
    *,
    heading_rad: float = 0.0,
    home_xy: Tuple[float, float] = (0.0, 0.0),
    previous_targets: Mapping[str, Tuple[float, float, float]] | None = None,
    active_formation_type: str | None = None,
) -> BehaviorResult:
    ids = _sorted_ids(robot_ids)
    if not ids:
        return BehaviorResult(
            targets={},
            metadata={
                "executor": "none",
                "status": "no_selection",
            },
        )

    spacing_m = max(0.2, _as_float(behavior_params.get("spacing_m"), 1.0))
    speed_scale = max(0.1, _as_float(behavior_params.get("speed_scale"), 0.55))
    aggression_scale = max(0.1, _as_float(behavior_params.get("aggression_scale"), 1.0))

    n = len(ids)
    cx, cy = centroid_xy
    name = str(behavior_name or "").upper()

    if name == "PATROL":
        forward_span = spacing_m * max(2.0, n * 0.75)
        lateral_span = spacing_m * max(1.0, n * 0.35)
        local_waypoints = [
            (forward_span, lateral_span),
            (forward_span, -lateral_span),
            (-forward_span, -lateral_span),
            (-forward_span, lateral_span),
        ]
        waypoints = _world_from_local(local_waypoints, centroid_xy, heading_rad)

        targets: TargetMap = {}
        for i, rid in enumerate(ids):
            wp_idx = i % len(waypoints)
            cur_wp = waypoints[wp_idx]
            next_wp = waypoints[(wp_idx + 1) % len(waypoints)]
            targets[rid] = (cur_wp[0], cur_wp[1], _heading_to(cur_wp, next_wp, heading_rad))

        return BehaviorResult(
            targets=targets,
            metadata={
                "executor": "patrol_rect",
                "status": "ok",
                "loop": True,
                "path_waypoints": _serialize_waypoints(waypoints),
            },
        )

    if name == "PATROL_PERIMETER":
        params = FormationParams(spacing_m=spacing_m, heading_rad=heading_rad)
        base = compute_formation_targets("CIRCLE", ids, centroid_xy, params)
        targets: TargetMap = {}
        radii: List[float] = []
        for rid, (x, y, _) in base.items():
            ang = math.atan2(y - cy, x - cx)
            radii.append(math.hypot(x - cx, y - cy))
            targets[rid] = (x, y, ang + (math.pi / 2.0))
        avg_radius = (sum(radii) / len(radii)) if radii else spacing_m

        return BehaviorResult(
            targets=targets,
            metadata={
                "executor": "patrol_perimeter",
                "status": "ok",
                "loop": True,
                "radius_m": float(avg_radius),
                "formation_hint": active_formation_type or "CIRCLE",
            },
        )

    if name in {"FOLLOW_PATH", "FOLLOW_PATH_LOOP"}:
        waypoints = _waypoints_from_param(behavior_params.get("path_waypoints"))
        if len(waypoints) < 2:
            path_step = spacing_m * (2.0 + (1.0 - min(1.0, speed_scale)))
            local = [
                (-path_step, 0.0),
                (0.0, 0.0),
                (path_step, 0.0),
                (2.0 * path_step, 0.0),
            ]
            waypoints = _world_from_local(local, centroid_xy, heading_rad)

        path_heading = _heading_to(waypoints[0], waypoints[1], heading_rad)
        stage_center = waypoints[0]
        targets = compute_formation_targets(
            "LINE",
            ids,
            centroid_xy=stage_center,
            params=FormationParams(spacing_m=spacing_m, heading_rad=path_heading),
        )

        return BehaviorResult(
            targets=targets,
            metadata={
                "executor": "follow_path",
                "status": "ok",
                "loop": name == "FOLLOW_PATH_LOOP",
                "path_waypoints": _serialize_waypoints(waypoints),
            },
        )

    if name == "HOLD_ANCHOR":
        targets: TargetMap = {}
        if previous_targets:
            for rid in ids:
                if rid in previous_targets:
                    x, y, yaw = previous_targets[rid]
                    targets[rid] = (float(x), float(y), float(yaw))

        if len(targets) < n:
            missing_ids = [rid for rid in ids if rid not in targets]
            missing = compute_formation_targets(
                "LINE",
                missing_ids,
                centroid_xy=centroid_xy,
                params=FormationParams(spacing_m=spacing_m, heading_rad=heading_rad),
            )
            targets.update(missing)

        return BehaviorResult(
            targets=targets,
            metadata={
                "executor": "hold_anchor",
                "status": "ok",
                "anchor_source": "previous_targets" if previous_targets else "generated",
            },
        )

    if name == "RETURN_HOME":
        formation = "CIRCLE" if n >= 3 else "COLUMN"
        targets = compute_formation_targets(
            formation,
            ids,
            centroid_xy=home_xy,
            params=FormationParams(spacing_m=spacing_m, heading_rad=heading_rad),
        )
        return BehaviorResult(
            targets=targets,
            metadata={
                "executor": "return_home",
                "status": "ok",
                "home_xy": {"x": float(home_xy[0]), "y": float(home_xy[1])},
                "home_formation": formation,
            },
        )

    if name == "FOLLOW_ME_TOGGLE":
        if not bool(behavior_params.get("follow_me_enabled", True)):
            return BehaviorResult(
                targets={},
                metadata={
                    "executor": "follow_me",
                    "status": "disabled",
                    "follow_me_enabled": False,
                },
            )
        base = compute_formation_targets(
            "WEDGE",
            ids,
            centroid_xy=centroid_xy,
            params=FormationParams(spacing_m=spacing_m, heading_rad=heading_rad),
        )
        backoff = 1.5 * spacing_m
        dx = -backoff * math.cos(heading_rad)
        dy = -backoff * math.sin(heading_rad)
        targets = {
            rid: (x + dx, y + dy, heading_rad)
            for rid, (x, y, _) in base.items()
        }
        return BehaviorResult(
            targets=targets,
            metadata={
                "executor": "follow_me",
                "status": "ok",
                "follow_me_enabled": True,
                "leader_xy": {"x": float(cx), "y": float(cy)},
            },
        )

    if name == "DISPERSE_SCAN":
        scan_radius = spacing_m * (1.8 + aggression_scale)
        targets: TargetMap = {}
        for i, rid in enumerate(ids):
            ang = heading_rad + ((2.0 * math.pi * i) / n)
            x = cx + (scan_radius * math.cos(ang))
            y = cy + (scan_radius * math.sin(ang))
            targets[rid] = (x, y, ang)

        return BehaviorResult(
            targets=targets,
            metadata={
                "executor": "disperse_scan",
                "status": "ok",
                "scan_radius_m": float(scan_radius),
            },
        )

    return BehaviorResult(
        targets={},
        metadata={
            "executor": "unknown",
            "status": "unknown_behavior",
            "behavior_name": name,
        },
    )
