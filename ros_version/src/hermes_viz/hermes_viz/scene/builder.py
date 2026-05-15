"""Top-level scene assembly + named camera presets.

Coordinate convention (Vuer/Three.js): +X right, +Y up, +Z toward audience.
OptiTrack frame is mapped to world via scene.robots and scene.hands.
"""

from __future__ import annotations

from typing import Any

from vuer.schemas import Scene, AmbientLight, DirectionalLight

from .lab import build_lab_nodes
from .robots import RobotsScene
from .hands import HandsScene


CAMERA_DEFAULT_KEY = "front_elevated"


def camera_presets() -> dict[str, dict[str, Any]]:
    """Three named camera viewpoints. Reachable via hotkeys 1/2/3 in the viewer."""
    return {
        "front_elevated": {
            "position": [0.0, 2.0, 3.0],
            "lookAt":   [0.0, 0.3, 0.0],
            "fov": 50,
        },
        "top_down": {
            "position": [0.0, 6.0, 0.001],  # tiny non-zero z so lookAt direction is well-defined
            "lookAt":   [0.0, 0.0, 0.0],
            "fov": 60,
        },
        "orbit": {
            "position": [4.0, 1.8, 0.0],
            "lookAt":   [0.0, 0.3, 0.0],
            "fov": 50,
            # The sink can read this to drive auto-orbit motion.
            "orbit": {"radius": 4.0, "height": 1.8, "period_s": 30.0},
        },
    }


def build_root_scene(*, lab_dims: dict, robots: RobotsScene, hands: HandsScene) -> Scene:
    """Compose the initial root scene: lab + bots + hands + lights.

    `lab_dims` is forwarded to `build_lab_nodes` and accepts any subset of its kwargs.
    """
    children: list = []
    for kn in build_lab_nodes(**lab_dims):
        children.append(kn.node)
    for kn in robots.snapshot_nodes():
        children.append(kn.node)
    for kn in hands.snapshot_nodes():
        children.append(kn.node)
    children.append(AmbientLight(intensity=0.4))
    children.append(DirectionalLight(intensity=0.8, position=[2, 4, 2]))
    return Scene(*children)
