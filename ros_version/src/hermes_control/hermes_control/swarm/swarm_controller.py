# swarm/swarm_controller.py
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

from hermes_control.swarm.behavior_engine import execute_behavior
from hermes_control.swarm.formation_engine import FormationParams, compute_formation_targets


@dataclass
class SwarmController:
    robot_ids: List[str]
    selection: Set[str] = field(default_factory=set)
    groups: Dict[str, Set[str]] = field(default_factory=lambda: {k: set() for k in "ABCDEFG"})
    last_selection: Set[str] = field(default_factory=set)
    active_group_edit: Optional[str] = None
    pending_group_members: Set[str] = field(default_factory=set)

    pending_formation_type: Optional[str] = None
    active_formation_type: Optional[str] = None
    formation_heading: float = 0.0
    formation_spacing: float = 1.0
    home_xy: Tuple[float, float] = (0.0, 0.0)
    path_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    follow_me_enabled: bool = False

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
        target_centroid = centroid_xy if centroid_xy is not None else (0.0, 0.0)

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

        if etype == "set_mode":
            mode = str(effect.get("value", "")).strip().upper()
            if mode in {"DRIVE", "SELECTION", "FORMATION", "BEHAVIOR", "PARAMS"}:
                gesture_state.mode = mode
            return

        if etype == "set_selection":
            value = effect.get("value")
            if not isinstance(value, list):
                return
            self._set_selection(set(value))
            if self.active_behavior:
                self._refresh_behavior_params(gesture_state, target_centroid)
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
            if key in {"speed_level", "spacing_level", "aggression_level"}:
                gesture_state.params[key] = self._clamp_level(gesture_state.params.get(key, 2))
            if key == "spacing_level":
                self.formation_spacing = self._spacing_m_for_level(int(gesture_state.params[key]))
            if self.active_behavior and key in {"speed_level", "spacing_level", "aggression_level"}:
                self._refresh_behavior_params(gesture_state, target_centroid)
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
            if key in {"speed_level", "spacing_level", "aggression_level"}:
                gesture_state.params[key] = self._clamp_level(gesture_state.params.get(key, 2))
            if key == "spacing_level":
                self.formation_spacing = self._spacing_m_for_level(int(gesture_state.params[key]))
            if self.active_behavior and key in {"speed_level", "spacing_level", "aggression_level"}:
                self._refresh_behavior_params(gesture_state, target_centroid)
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
            if self.active_behavior:
                self._refresh_behavior_params(gesture_state, target_centroid)
            return

        if etype == "select_group":
            group_name = resolved.get("binding")
            self._start_group_assignment(group_name)
            if self.active_group_edit:
                gesture_state.modifiers["group_edit_active"] = True
                gesture_state.modifiers["group_edit_name"] = self.active_group_edit
            if self.active_behavior:
                self._refresh_behavior_params(gesture_state, target_centroid)
            return

        if etype == "confirm_group_assignment":
            self._confirm_group_assignment()
            gesture_state.modifiers["group_edit_active"] = False
            gesture_state.modifiers.pop("group_edit_name", None)
            if self.active_behavior:
                self._refresh_behavior_params(gesture_state, target_centroid)
            return

        if etype == "cancel_group_assignment":
            self._cancel_group_assignment()
            gesture_state.modifiers["group_edit_active"] = False
            gesture_state.modifiers.pop("group_edit_name", None)
            if self.active_behavior:
                self._refresh_behavior_params(gesture_state, target_centroid)
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
                gesture_state.params["spacing_level"] = self._spacing_level_from_m(self.formation_spacing)
            if self.active_behavior and key in {"formation_heading", "formation_spacing"}:
                self._refresh_behavior_params(gesture_state, target_centroid)
            return

        if etype == "apply_formation":
            if not self.selection:
                return
            if self.pending_formation_type:
                self.active_formation_type = self.pending_formation_type
            if not self.active_formation_type:
                return

            params = FormationParams(
                spacing_m=float(self.formation_spacing),
                heading_rad=float(self.formation_heading),
            )
            self.last_targets = compute_formation_targets(
                self.active_formation_type,
                list(self.selection),
                centroid_xy=target_centroid,
                params=params,
            )
            return

        if etype == "break_formation":
            self.active_formation_type = None
            self.last_targets = {}
            return

        # Behavior
        if etype == "start_behavior":
            behavior_name = str(resolved.get("binding") or "")
            if not behavior_name:
                return

            if behavior_name == "FOLLOW_ME_TOGGLE":
                self.follow_me_enabled = not self.follow_me_enabled
                if self.follow_me_enabled:
                    self.active_behavior = behavior_name
                    self._refresh_behavior_params(gesture_state, target_centroid)
                else:
                    if self.active_behavior == "FOLLOW_ME_TOGGLE":
                        self.active_behavior = None
                    self.last_targets = {}
                    self.behavior_params = {
                        "behavior": "FOLLOW_ME_TOGGLE",
                        "executor": "follow_me",
                        "status": "disabled",
                        "follow_me_enabled": False,
                    }
                return

            self.follow_me_enabled = False
            self.active_behavior = behavior_name
            self._refresh_behavior_params(gesture_state, target_centroid)
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

    @staticmethod
    def _clamp_level(value: Any, default: int = 2) -> int:
        try:
            level = int(value)
        except (TypeError, ValueError):
            return default
        return max(1, min(4, level))

    @staticmethod
    def _speed_scale_for_level(level: int) -> float:
        return {
            1: 0.35,
            2: 0.55,
            3: 0.75,
            4: 1.00,
        }.get(level, 0.55)

    @staticmethod
    def _aggression_scale_for_level(level: int) -> float:
        return {
            1: 0.75,
            2: 1.00,
            3: 1.20,
            4: 1.40,
        }.get(level, 1.00)

    @staticmethod
    def _spacing_m_for_level(level: int) -> float:
        return {
            1: 0.70,
            2: 1.00,
            3: 1.30,
            4: 1.60,
        }.get(level, 1.00)

    @classmethod
    def _spacing_level_from_m(cls, spacing_m: float) -> int:
        levels = [1, 2, 3, 4]
        return min(levels, key=lambda lvl: abs(cls._spacing_m_for_level(lvl) - float(spacing_m)))

    def _refresh_behavior_params(
        self,
        gesture_state: Any,
        centroid_xy: Optional[Tuple[float, float]] = None,
    ) -> None:
        speed_level = self._clamp_level(gesture_state.params.get("speed_level", 2))
        spacing_level = self._clamp_level(gesture_state.params.get("spacing_level", 2))
        aggression_level = self._clamp_level(gesture_state.params.get("aggression_level", 2))

        self.behavior_params = {
            "behavior": self.active_behavior,
            "speed_level": speed_level,
            "speed_scale": self._speed_scale_for_level(speed_level),
            "spacing_level": spacing_level,
            "spacing_m": float(self.formation_spacing),
            "formation_heading": float(self.formation_heading),
            "aggression_level": aggression_level,
            "aggression_scale": self._aggression_scale_for_level(aggression_level),
            "home_xy": {"x": float(self.home_xy[0]), "y": float(self.home_xy[1])},
            "path_waypoints": [list(pt) for pt in self.path_waypoints],
            "follow_me_enabled": bool(self.follow_me_enabled),
        }
        if self.active_behavior:
            self._execute_active_behavior(centroid_xy if centroid_xy is not None else (0.0, 0.0))

    def _execute_active_behavior(self, centroid_xy: Tuple[float, float]) -> None:
        result = execute_behavior(
            behavior_name=str(self.active_behavior),
            robot_ids=sorted(self.selection),
            centroid_xy=centroid_xy,
            behavior_params=self.behavior_params,
            heading_rad=float(self.formation_heading),
            home_xy=self.home_xy,
            previous_targets=self.last_targets,
            active_formation_type=self.active_formation_type,
        )
        self.last_targets = dict(result.targets)
        self.behavior_params.update(result.metadata)

    def _stop_all(self) -> None:
        self.paused = False
        self.active_behavior = None
        self.behavior_params = {}
        self.active_formation_type = None
        self.pending_formation_type = None
        self.last_targets = {}
        self.last_cmd_vel = {}
        self.active_group_edit = None
        self.pending_group_members = set()
        self.follow_me_enabled = False
