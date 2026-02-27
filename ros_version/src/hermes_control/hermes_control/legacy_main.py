# main.py
from typing import Any, Dict, Optional, Tuple

from hermes_control.gestures.matcher import match_gesture
from hermes_control.gestures.models import GestureState
from hermes_control.gestures.recognizer import GestureRecognizer
from hermes_control.gestures.registry import COMMAND_REGISTRY
from hermes_control.gestures.safety import SafetyEvaluator
from hermes_control.swarm.swarm_controller import SwarmController

state = GestureState()
recognizer = GestureRecognizer()
safety = SafetyEvaluator()
swarm = SwarmController(robot_ids=["r1", "r2", "r3", "r4", "r5"])


def loop_tick(raw: Dict[str, Any], centroid_xy: Tuple[float, float] = (0.0, 0.0)) -> Dict[str, Optional[Dict[str, Any]]]:
    ev = recognizer.recognize(raw)
    now_ms = int(raw["time_ms"])

    safety_packet = safety.tick(ev, state, COMMAND_REGISTRY, now_ms)
    if safety_packet:
        swarm.handle_packet(safety_packet, state, centroid_xy=centroid_xy)

    packet = match_gesture(ev, state, COMMAND_REGISTRY)
    if packet:
        swarm.handle_packet(packet, state, centroid_xy=centroid_xy)

    return {
        "safety_packet": safety_packet,
        "packet": packet,
    }
