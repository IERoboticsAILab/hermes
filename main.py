# main.py
from gesture.models import GestureState
from gesture.recognizer import GestureRecognizer
from gesture.registry import COMMAND_REGISTRY
from gesture.matcher import match_gesture
from swarm.swarm_controller import SwarmController

# Example init
state = GestureState()
recognizer = GestureRecognizer()
swarm = SwarmController(robot_ids=["r1", "r2", "r3", "r4", "r5", "r6", "r7"])

def get_centroid_xy(selection):
    # Replace with real robot pose averaging.
    return (0.0, 0.0)

def loop_tick(raw):
    # 1) Recognize gestures
    ev = recognizer.recognize(raw)

    # 2) Match to command packet
    packet = match_gesture(ev, state, COMMAND_REGISTRY)

    # 3) Apply swarm semantics
    if packet:
        centroid = get_centroid_xy(swarm.selection if swarm.selection else swarm.robot_ids)
        swarm.handle_packet(packet, state, centroid_xy=centroid)

        # If formation applied, you now have targets in swarm.last_targets
        if swarm.last_targets:
            # publish targets to robots here
            pass

    # 4) Continuous drive (manual cmd_vel) happens here, not in swarm controller
    # if state.mode == "DRIVE" and state.deadman_active and ev.imu and not state.params.get("paused", False):
    #     publish_cmd_vel(selection=swarm.selection or set(swarm.robot_ids), imu=ev.imu, params=state.params)
