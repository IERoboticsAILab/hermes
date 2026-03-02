# swarm/swarm_controller.py
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

from hermes_control.swarm.formation_engine import FormationParams, compute_formation_targets


@dataclass
class SwarmController:
    robot_ids: List[str]
    selection: Set[str] = field(default_factory=set)
    groups: Dict[str, Set[str]] = field(default_factory=lambda: {k: set() for k in "ABCDEFGH"})
    last_selection: Set[str] = field(default_factory=set)
    active_group_edit: Optional[str] = None
    pending_group_members: Set[str] = field(default_factory=set)

    pending_formation_type: Optional[str] = None
    active_formation_type: Optional[str] = None
    formation_heading: float = 0.0
    formation_spacing: float = 1.0

    active_behavior: Optional[str] = None
    behavior_params: Dict[str, Any] = field(default_factory=dict)

    paused: bool = False
    last_targets: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    last_cmd_vel: Dict[str, float] = field(default_factory=dict)

    def handle_packet(
        self,
        packet: Dict[str, Any],
        gesture_state: Any,
        centroid_xy: Optional[Tuple[float, float]] = None,
    ) -> None:
        effect = packet.get("effect", {})
        etype = effect.get("type")
        resolved = packet.get("resolved", {})

        if etype == "gate_motion":
            gesture_state.deadman_active = bool(effect.get("value", True))
            return

        if etype == "pause":
            self.paused = True
            gesture_state.params["paused"] = True
            return

        if etype == "resume":
            self.paused = False
            gesture_state.params["paused"] = False
            return

        if etype == "emergency_stop":
            self._stop_all()
            gesture_state.deadman_active = False
            gesture_state.modifiers["group_edit_active"] = False
            gesture_state.modifiers.pop("group_edit_name", None)
            return

        if etype == "toggle":
            key = effect.get("key")
            if not key:
                return
            if key == "yaw_mode":
                current = str(gesture_state.params.get("yaw_mode", "steer"))
                gesture_state.params["yaw_mode"] = "rotate_in_place" if current == "steer" else "steer"
            else:
                gesture_state.params[key] = not bool(gesture_state.params.get(key, False))
            return

        if etype == "set_while_held":
            key = effect.get("key")
            if not key:
                return
            gesture_state.params[key] = bool(effect.get("value", True))
            return

        if etype == "modifier":
            key = effect.get("key")
            if not key:
                return
            gesture_state.modifiers[key] = effect.get("value")
            return

        if etype == "set_param":
            key = effect.get("key")
            if not key:
                return
            gesture_state.params[key] = effect.get("value")
            return

        if etype == "step_param":
            key = effect.get("key")
            if not key:
                return
            cur = int(gesture_state.params.get(key, 0))
            delta = int(effect.get("delta", 0))
            nxt = cur + delta
            if "min" in effect:
                nxt = max(int(effect["min"]), nxt)
            if "max" in effect:
                nxt = min(int(effect["max"]), nxt)
            gesture_state.params[key] = nxt
            return

        if etype == "cmd_vel_stream":
            self.last_cmd_vel = dict(resolved.get("cmd_vel", {}))
            return

        # Selection
        if etype == "select_robot":
            rid = self._normalize_robot_id(resolved.get("binding"))
            if rid is None:
                return
            if self.active_group_edit:
                # While editing a group slot, robot gestures toggle membership.
                sel = set(self.pending_group_members)
                if rid in sel:
                    sel.remove(rid)
                else:
                    sel.add(rid)
                self.pending_group_members = sel
                self._set_selection(sel)
            else:
                # Outside group assignment workflow, keep robot selection simple.
                self._set_selection({rid})
            return

        if etype == "select_group":
            group_name = resolved.get("binding")
            self._start_group_assignment(group_name)
            if self.active_group_edit:
                gesture_state.modifiers["group_edit_active"] = True
                gesture_state.modifiers["group_edit_name"] = self.active_group_edit
            return

        if etype == "confirm_group_assignment":
            self._confirm_group_assignment()
            gesture_state.modifiers["group_edit_active"] = False
            gesture_state.modifiers.pop("group_edit_name", None)
            return

        if etype == "cancel_group_assignment":
            self._cancel_group_assignment()
            gesture_state.modifiers["group_edit_active"] = False
            gesture_state.modifiers.pop("group_edit_name", None)
            return

        # Formation
        if etype == "set_state" and effect.get("key") == "pending_formation_type":
            self.pending_formation_type = effect.get("value")
            return

        if etype == "set_param_stream":
            key = effect.get("key")
            value = resolved.get("stream_value")
            if value is None:
                return
            if key == "formation_heading":
                self.formation_heading = float(value)
            elif key == "formation_spacing":
                self.formation_spacing = max(0.2, float(value))
            return

        if etype == "apply_formation":
            if not self.selection:
                return
            if self.pending_formation_type:
                self.active_formation_type = self.pending_formation_type
            if not self.active_formation_type:
                return

            cx, cy = centroid_xy if centroid_xy is not None else (0.0, 0.0)
            params = FormationParams(
                spacing_m=float(self.formation_spacing),
                heading_rad=float(self.formation_heading),
            )
            self.last_targets = compute_formation_targets(
                self.active_formation_type,
                list(self.selection),
                centroid_xy=(cx, cy),
                params=params,
            )
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
        normalized = set()
        for rid in new_sel:
            norm = self._normalize_robot_id(rid)
            if norm is not None:
                normalized.add(norm)

        self.last_selection = set(self.selection)
        self.selection = normalized

    def _start_group_assignment(self, group_name: Optional[str]) -> None:
        if not group_name:
            return
        # Reselecting a group starts a fresh version of that group.
        self.active_group_edit = str(group_name)
        self.groups[self.active_group_edit] = set()
        self.pending_group_members = set()
        self._set_selection(set())

    def _confirm_group_assignment(self) -> None:
        if not self.active_group_edit:
            return
        self.groups[self.active_group_edit] = set(self.selection)
        self.pending_group_members = set()
        self.active_group_edit = None

    def _cancel_group_assignment(self) -> None:
        # Cancels group-edit mode while preserving the current robot selection.
        self.pending_group_members = set()
        self.active_group_edit = None

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

    def _normalize_robot_id(self, rid: Any) -> Optional[str]:
        if rid is None:
            return None
        candidate = str(rid).strip().lower()

        inventory = {r.lower() for r in self.robot_ids}
        if candidate in inventory:
            return candidate

        # Accept R1/r1 aliases only if present in inventory.
        if candidate.startswith("r") and candidate[1:].isdigit() and candidate in inventory:
            return candidate

        return None

    def _stop_all(self) -> None:
        self.paused = False
        self.active_behavior = None
        self.active_formation_type = None
        self.pending_formation_type = None
        self.last_targets = {}
        self.last_cmd_vel = {}
        self.active_group_edit = None
        self.pending_group_members = set()
