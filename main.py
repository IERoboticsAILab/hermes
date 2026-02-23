# main.py
from gesture.models import GestureState
from gesture.recognizer import GestureRecognizer
from gesture.registry import COMMAND_REGISTRY
from gesture.matcher import match_gesture
from gesture.safety import SafetyEvaluator
from swarm.swarm_controller import SwarmController

state = GestureState()
recognizer = GestureRecognizer()
safety = SafetyEvaluator()
swarm = SwarmController(robot_ids=["r1","r2","r3","r4","r5"])

def loop_tick(raw):
    ev = recognizer.recognize(raw)
    now_ms = int(raw["time_ms"])

    # 1) Safety evaluation (may emit a packet when gate changes)
    safety_packet = safety.tick(ev, state, COMMAND_REGISTRY, now_ms)
    if safety_packet:
        # Optionally: if gate just turned OFF, publish stop immediately
        pass

    # 2) Normal gesture matching (ESTOP still works here too)
    packet = match_gesture(ev, state, COMMAND_REGISTRY)
    if packet:
        # swarm.handle_packet(packet, state, centroid_xy=...)
        pass

    # 3) Continuous drive publishing is still gated by state.deadman_active
    # if state.mode == "DRIVE" and state.deadman_active and ev.imu_R and not state.params.get("paused", False):
    #     publish_cmd_vel(...)