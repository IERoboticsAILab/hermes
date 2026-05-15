"""Shared scene-level types used by lab/robots/hands builders."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class KeyedNode:
    """A Vuer scene node paired with a stable key for upsert/update semantics."""
    key: str
    node: Any  # Vuer schema node (Plane, Box, Glb, Urdf, Line, Html, ...)
