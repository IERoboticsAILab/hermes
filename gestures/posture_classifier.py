# gesture/posture_classifier.py
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class PostureCalibration:
    # normalized flex values assumed [0..1] where 0=extended, 1=fully curled
    ext_max: float = 0.35    # <= ext_max treated as extended
    curl_min: float = 0.65   # >= curl_min treated as curled

    # optional hysteresis margin to avoid jitter
    hysteresis: float = 0.05


class PostureClassifier:
    """
    flex_input: {"index":0..1,"middle":0..1,"ring":0..1,"pinky":0..1}
    output: "OPEN"|"FIST"|"POINT"|"TWO"|"THREE"|None (unknown)
    """

    def __init__(self, calib: Optional[PostureCalibration] = None):
        self.calib = calib or PostureCalibration()

    def classify(self, flex: Dict[str, float]) -> Optional[str]:
        e = self.calib.ext_max
        c = self.calib.curl_min

        def is_ext(v: float) -> bool:
            return v <= e

        def is_curl(v: float) -> bool:
            return v >= c

        idx, mid, ring, pinky = flex["index"], flex["middle"], flex["ring"], flex["pinky"]

        # OPEN: all extended
        if is_ext(idx) and is_ext(mid) and is_ext(ring) and is_ext(pinky):
            return "OPEN"

        # FIST: all curled
        if is_curl(idx) and is_curl(mid) and is_curl(ring) and is_curl(pinky):
            return "FIST"

        # POINT: index extended, others curled
        if is_ext(idx) and is_curl(mid) and is_curl(ring) and is_curl(pinky):
            return "POINT"

        # TWO: index+middle extended, ring+pinky curled
        if is_ext(idx) and is_ext(mid) and is_curl(ring) and is_curl(pinky):
            return "TWO"

        # THREE: index+middle+ring extended, pinky curled
        if is_ext(idx) and is_ext(mid) and is_ext(ring) and is_curl(pinky):
            return "THREE"

        return None