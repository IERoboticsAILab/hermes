# swarm/swarm_controller.py
from dataclasses import dataclass, field
from typing import Dict, Set, List, Optional, Any, Tuple

from swarm.formation_engine import compute_formation_targets, FormationParams


@dataclass
class SwarmController:
    robot_ids: List[str]
    selection: Set[str] = field(default_factory=set)
    groups: Dict[str, Set[str]] = field(default_factory=lambda: {k: set() for k in "ABCDEFGH"})
    last_selection: Set[str] = field(default_factory=set)

    pending_formation_type: Optional[str] = None
    active_formation_type: Optional[str] = None
    formation_heading: float = 0.0
    formation_spacing: float = 1.0

    active_behavior: Optional[str] = None
    behavior_params: Dict[str, Any] = field(default_factory=dict)

    paused: bool = False

    # You can store last computed targets for debugging / publishing
    last_targets: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)

    def handle_packet(self, packet: Dict[str, Any], gesture_state: Any, centroid_xy: Tuple[float, float]) -> None:
        effect = packet.get("effect", {})
        etype = effect.get("type")
        resolved = packet.get("resolved", {})

        if etype == "pause":
            self.paused = True
            return
        if etype == "resume":
            self.paused = False
            return
        if etype == "emergency_stop":
            self._stop_all()
            return

        # Selection
        if etype == "select_group":
            group_name = resolved.get("binding")
            op = resolved.get("selection_op", "replace")
            self._select_group(group_name, op=op)
            return

        if etype == "select_all":
            self._set_selection(set(self.robot_ids))
            return

        if etype == "select_none":
            self._set_selection(set())
            return

        if etype == "recall_last_selection":
            self._set_selection(set(self.last_selection))
            return

        if etype == "save_selection_to_group":
            group_name = resolved.get("binding")
            if group_name:
                self.groups[group_name] = set(self.selection)
            return

        # Formation type selection comes in as set_state with value
        if etype == "set_state" and effect.get("key") == "pending_formation_type":
            self.pending_formation_type = effect.get("value")
            return

        if etype == "apply_formation":
            if not self.selection:
                return
            if self.pending_formation_type:
                self.active_formation_type = self.pending_formation_type

            if not self.active_formation_type:
                return

            # spacing level from gesture_state can override
            spacing_level = int(gesture_state.params.get("spacing_level", 2))
            # Example mapping 1..4 -> meters (tweak to taste)
            spacing_map = {1: 0.6, 2: 1.0, 3: 1.5, 4: 2.2}
            self.formation_spacing = spacing_map.get(spacing_level, 1.0)

            params = FormationParams(spacing_m=self.formation_spacing, heading_rad=self.formation_heading)
            self.last_targets = compute_formation_targets(
                self.active_formation_type,
                list(self.selection),
                centroid_xy=centroid_xy,
                params=params
            )
            # Your transport layer should publish these targets
            # publish_formation_targets(self.last_targets)
            return

        if etype == "break_formation":
            self.active_formation_type = None
            self.last_targets = {}
            return

        # Behavior
        if etype == "start_behavior":
            behavior_name = resolved.get("binding")
            if behavior_name:
                self.active_behavior = behavior_name
            return

    def set_formation_heading(self, heading_rad: float) -> None:
        self.formation_heading = float(heading_rad)

    def _set_selection(self, new_sel: Set[str]) -> None:
        self.last_selection = set(self.selection)
        self.selection = set(new_sel)

    def _select_group(self, group_name: Optional[str], op: str = "replace") -> None:
        if not group_name:
            return
        group = self.groups.get(group_name, set())
        if op == "toggle":
            sel = set(self.selection)
            for rid in group:
                if rid in sel:
                    sel.remove(rid)
                else:
                    sel.add(rid)
            self._set_selection(sel)
        else:
            self._set_selection(set(group))

    def _stop_all(self) -> None:
        self.paused = False
        self.active_behavior = None
        self.active_formation_type = None
        self.pending_formation_type = None
        self.last_targets = {}