# gestures/posture_classifier.py
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class PostureCalibration:
    # Current glove calibration treats values below ~0.9 as extended.
    # Values at/above ~0.9 are treated as curled.
    ext_max: float = 0.80
    curl_min: float = 0.80

    # Keep hysteresis off by default for this near-single-threshold calibration.
    hysteresis: float = 0.0


class PostureClassifier:
    """
    flex_input: {"index":0..1,"middle":0..1,"ring":0..1,"pinky":0..1}
    output: "OPEN"|"FIST"|"POINT"|"TWO"|"THREE"|None
    """

    def __init__(self, calib: Optional[PostureCalibration] = None):
        self.calib = calib or PostureCalibration()

    def classify(self, flex: Dict[str, float]) -> Optional[str]:
        # Expand EXT/CURL thresholds slightly by hysteresis margin to reduce jitter.
        h = max(0.0, float(self.calib.hysteresis))
        ext_limit = min(1.0, self.calib.ext_max + h)
        curl_limit = max(0.0, self.calib.curl_min - h)

        if ext_limit >= curl_limit:
            # Fall back to raw calibration if hysteresis collapses the deadband.
            ext_limit = self.calib.ext_max
            curl_limit = self.calib.curl_min

        def is_ext(v: float) -> bool:
            return v <= ext_limit

        def is_curl(v: float) -> bool:
            return v >= curl_limit

        idx, mid, ring, pinky = flex["index"], flex["middle"], flex["ring"], flex["pinky"]

        if is_ext(idx) and is_ext(mid) and is_ext(ring) and is_ext(pinky):
            return "OPEN"
        if is_curl(idx) and is_curl(mid) and is_curl(ring) and is_curl(pinky):
            return "FIST"
        if is_ext(idx) and is_curl(mid) and is_curl(ring) and is_curl(pinky):
            return "POINT"
        if is_ext(idx) and is_ext(mid) and is_curl(ring) and is_curl(pinky):
            return "TWO"
        if is_ext(idx) and is_ext(mid) and is_ext(ring) and is_curl(pinky):
            return "THREE"

        return None
