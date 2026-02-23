# gesture/recognizer.py
from dataclasses import dataclass
from typing import Dict, Any, Optional
from gesture.models import GestureEvent
from gesture.fsr_tracker import FsrTracker, FsrConfig
from gesture.posture_classifier import PostureClassifier, PostureCalibration
from gesture.imu_filter import ImuFilter, ImuFilterConfig


@dataclass
class RecognizerConfig:
    fsr: FsrConfig = FsrConfig()
    posture: PostureCalibration = PostureCalibration()
    imu: ImuFilterConfig = ImuFilterConfig()


class GestureRecognizer:
    """
    Expects raw input in a consistent structure:

    raw = {
      "time_ms": int,
      "flex": {
        "L": {"index":0..1,"middle":0..1,"ring":0..1,"pinky":0..1,"thumb":0..1(optional)},
        "R": {...}
      },
      "fsr_pressed": {
        "L": {"INDEX":bool,"MIDDLE":bool,"RING":bool,"PINKY":bool},
        "R": {"INDEX":bool,"MIDDLE":bool,"RING":bool,"PINKY":bool}
      },
      "imu": {
        "R": {"PITCH":float,"ROLL":float,"YAW":float},
        "L": {"PITCH":float,"ROLL":float,"YAW":float}  # optional
      }
    }
    """

    def __init__(self, config: Optional[RecognizerConfig] = None):
        self.cfg = config or RecognizerConfig()
        self.fsr = FsrTracker(self.cfg.fsr)
        self.posture = PostureClassifier(self.cfg.posture)
        self.imu_filter = ImuFilter(self.cfg.imu)

    def recognize(self, raw: Dict[str, Any]) -> GestureEvent:
        now_ms = int(raw["time_ms"])

        # Postures
        L_posture = None
        R_posture = None

        flex = raw.get("flex", {})
        if "L" in flex:
            L_posture = self.posture.classify(flex["L"])
        if "R" in flex:
            R_posture = self.posture.classify(flex["R"])

        # FSR events
        fsr_pressed = raw.get("fsr_pressed", {"L": {}, "R": {}})
        fsr_out = self.fsr.update(fsr_pressed, now_ms)

        single = fsr_out["single"]
        seq = fsr_out["sequence"]

        # IMU: default to right-hand IMU for driving
        # Right IMU (filtered) for drive if present
        imu_R_raw = (raw.get("imu", {}).get("R") or {})
        imu_R = self.imu_filter.update(imu_R_raw) if imu_R_raw else None

        # Left accel for deadman (MPU-6050)
        accel_L = (raw.get("imu", {}).get("L") or None)


        # Compose event
        ev = GestureEvent(
            L_posture=L_posture,
            R_posture=R_posture,
            fsr=single,
            fsr_sequence=seq,
            imu=imu
        )

        # Optionally provide hold_ms for two-fist estop etc.
        if single and single.action == "HOLD":
            ev.hold_ms = single.duration_ms

        return ev