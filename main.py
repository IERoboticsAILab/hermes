# main.py
from typing import Any, Dict, Optional, Tuple

from gestures.matcher import match_gesture
from gestures.models import GestureState
from gestures.recognizer import GestureRecognizer
from gestures.registry import COMMAND_REGISTRY
from gestures.safety import SafetyEvaluator
from swarm.swarm_controller import SwarmController

state = GestureState()
recognizer = GestureRecognizer()
safety = SafetyEvaluator()
swarm = SwarmController(robot_ids=["r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8"])


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
