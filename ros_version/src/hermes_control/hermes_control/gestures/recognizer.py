# gestures/recognizer.py
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from hermes_control.gestures.fsr_tracker import FsrConfig, FsrTracker
from hermes_control.gestures.imu_filter import ImuFilter, ImuFilterConfig
from hermes_control.gestures.models import GestureEvent
from hermes_control.gestures.posture_classifier import PostureCalibration, PostureClassifier


@dataclass
class RecognizerConfig:
    fsr: FsrConfig = field(default_factory=FsrConfig)
    posture: PostureCalibration = field(default_factory=PostureCalibration)
    imu: ImuFilterConfig = field(default_factory=ImuFilterConfig)


class GestureRecognizer:
    """
    Expects raw input like:

    raw = {
      "time_ms": int,
      "flex": {
        "L": {"index":0..1,"middle":0..1,"ring":0..1,"pinky":0..1},
        "R": {"index":0..1,"middle":0..1,"ring":0..1,"pinky":0..1}
      },
      "fsr_pressed": {
        "L": {"INDEX":bool,"MIDDLE":bool,"RING":bool,"PINKY":bool},
        "R": {"INDEX":bool,"MIDDLE":bool,"RING":bool,"PINKY":bool}
      },
      "imu": {
        # The active control IMU comes from the left glove only.
        "L": {"PITCH":float,"ROLL":float,"YAW":float,"AX":float,"AY":float,"AZ":float}
      }
    }
    """

    def __init__(self, config: Optional[RecognizerConfig] = None):
        self.cfg = config or RecognizerConfig()
        self.fsr = FsrTracker(self.cfg.fsr)
        self.posture = PostureClassifier(self.cfg.posture)
        self.imu_filter = ImuFilter(self.cfg.imu)

    def _coerce_left_accel(self, raw_imu: Dict[str, Any]) -> Optional[Dict[str, float]]:
        left = raw_imu.get("L_accel") or raw_imu.get("L_ACCEL") or raw_imu.get("L")
        if not isinstance(left, dict):
            return None

        if not any(k in left for k in ("AX", "AY", "AZ", "X", "Y", "Z")):
            return None

        return {
            "AX": float(left.get("AX", left.get("X", 0.0))),
            "AY": float(left.get("AY", left.get("Y", 0.0))),
            "AZ": float(left.get("AZ", left.get("Z", 0.0))),
        }

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

        imu_raw = raw.get("imu", {})
        control_imu_raw = imu_raw.get("L") or {}
        imu_R = self.imu_filter.update(control_imu_raw) if control_imu_raw else None
        accel_L = self._coerce_left_accel(imu_raw)

        ev = GestureEvent(
            L_posture=L_posture,
            R_posture=R_posture,
            fsr=single,
            fsr_sequence=seq,
            imu_R=imu_R,
            accel_L=accel_L,
        )

        # HOLD events carry duration information used by hold_ms constraints.
        if single and single.action == "HOLD":
            ev.hold_ms = single.duration_ms

        return ev
