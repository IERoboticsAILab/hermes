# swarm/formation_engine.py
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class FormationParams:
    spacing_m: float = 1.0
    heading_rad: float = 0.0  # rotation of formation in global frame


def _rotate(x: float, y: float, heading: float) -> Tuple[float, float]:
    ch = math.cos(heading)
    sh = math.sin(heading)
    return (ch * x - sh * y, sh * x + ch * y)


def _center_offsets(offsets: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if not offsets:
        return []
    mx = sum(x for x, _ in offsets) / float(len(offsets))
    my = sum(y for _, y in offsets) / float(len(offsets))
    return [(x - mx, y - my) for x, y in offsets]


def compute_formation_offsets(formation_type: str, n: int, spacing: float) -> List[Tuple[float, float]]:
    """
    Returns offsets (x,y) in formation local frame (x forward, y left).
    """
    if n <= 0:
        return []

    offsets: List[Tuple[float, float]]

    if formation_type == "LINE":
        # abreast line centered at origin along y axis
        # positions: y = ... spaced, x = 0
        start = -(n - 1) / 2.0
        offsets = [(0.0, (start + i) * spacing) for i in range(n)]

    elif formation_type == "COLUMN":
        # single file along x axis
        start = -(n - 1) / 2.0
        offsets = [((start + i) * spacing, 0.0) for i in range(n)]

    elif formation_type == "WEDGE":
        # V shape: leader at front, pairs behind
        offsets = [(0.0, 0.0)]
        layer = 1
        placed = 1
        while placed < n:
            # place left then right in each layer
            x = -layer * spacing
            y = layer * spacing
            if placed < n:
                offsets.append((x, y))
                placed += 1
            if placed < n:
                offsets.append((x, -y))
                placed += 1
            layer += 1
    elif formation_type == "CIRCLE":
        # equally spaced around circle radius chosen so arc spacing approx == spacing
        # circumference ≈ n*spacing => r ≈ (n*spacing)/(2π)
        r = max(spacing, (n * spacing) / (2.0 * math.pi))
        offsets = []
        for i in range(n):
            ang = (2.0 * math.pi * i) / n
            # x forward, y left: use cos for x, sin for y
            offsets.append((r * math.cos(ang), r * math.sin(ang)))
        # rotate so one is "front"

    elif formation_type == "GRID":
        # near-square grid
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)
        # center grid around origin
        offsets = []
        for idx in range(n):
            r = idx // cols
            c = idx % cols
            x = (r - (rows - 1) / 2.0) * spacing
            y = (c - (cols - 1) / 2.0) * spacing
            offsets.append((x, y))

    elif formation_type == "DIAMOND":
        # diamond: like a rotated square-ish layout
        # build layers expanding then contracting around center
        offsets = [(0.0, 0.0)]
        layer = 1
        placed = 1
        while placed < n:
            # front/back/left/right points for each layer
            candidates = [
                ( layer * spacing, 0.0),
                (-layer * spacing, 0.0),
                (0.0,  layer * spacing),
                (0.0, -layer * spacing),
            ]
            for xy in candidates:
                if placed >= n:
                    break
                offsets.append(xy)
                placed += 1
            layer += 1

    elif formation_type == "ECHELON_L":
        # diagonal line slanting left (y increases) going backward in x
        start = -(n - 1) / 2.0
        offsets = [((start + i) * spacing, (start + i) * spacing) for i in range(n)]

    elif formation_type == "ECHELON_R":
        # diagonal line slanting right (y decreases) going backward in x
        start = -(n - 1) / 2.0
        offsets = [((start + i) * spacing, -(start + i) * spacing) for i in range(n)]

    else:
        # fallback: COLUMN
        start = -(n - 1) / 2.0
        offsets = [((start + i) * spacing, 0.0) for i in range(n)]

    # Keep formation centered around origin for all types.
    return _center_offsets(offsets)


def compute_formation_targets(
    formation_type: str,
    robot_ids: List[str],
    centroid_xy: Tuple[float, float],
    params: FormationParams
) -> Dict[str, Tuple[float, float, float]]:
    """
    Returns {robot_id: (target_x, target_y, target_heading_rad)}

    - robot_ids order is stabilized (sorted) to avoid reshuffling.
    - centroid_xy is where the formation is centered in global coordinates.
    """
    ids = sorted(robot_ids)
    n = len(ids)
    offsets = compute_formation_offsets(formation_type, n, params.spacing_m)

    cx, cy = centroid_xy
    out: Dict[str, Tuple[float, float, float]] = {}

    for rid, (ox, oy) in zip(ids, offsets):
        rx, ry = _rotate(ox, oy, params.heading_rad)
        out[rid] = (cx + rx, cy + ry, params.heading_rad)

    return out
