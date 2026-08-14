# gestures/posture_classifier.py
from dataclasses import dataclass, field
from typing import Dict, Optional


# Default per-finger boundary between EXTended and CURLed. The four flex
# sensors don't share identical resting voltages (different physical
# wiring + Velcro tension on the user's hand), so each finger gets its
# own threshold. Tuned per-finger against the current left-glove build.
_DEFAULT_THRESHOLDS: Dict[str, float] = {
    "index": 0.71,
    "middle": 0.76,
    "ring": 0.66,
    "pinky": 0.71,
}


@dataclass
class PostureCalibration:
    # ext_max and curl_min are per-finger dicts. A value below ext_max[f]
    # is EXTended; a value at/above curl_min[f] is CURLed. With a single-
    # threshold setup (ext == curl) the hysteresis field opens a STATEFUL
    # deadband around that threshold: a finger only flips from EXT to
    # CURL once it crosses ABOVE (threshold + hysteresis), and only flips
    # back to EXT once it crosses BELOW (threshold - hysteresis). This
    # eliminates the rapid SELECTION ↔ FORMATION flicker that occurred
    # when the user's flex values sat near a finger's threshold.
    ext_max: Dict[str, float] = field(default_factory=lambda: dict(_DEFAULT_THRESHOLDS))
    curl_min: Dict[str, float] = field(default_factory=lambda: dict(_DEFAULT_THRESHOLDS))
    hysteresis: float = 0.02


class PostureClassifier:
    """
    flex_input: {"index":0..1,"middle":0..1,"ring":0..1,"pinky":0..1}
    output: "OPEN"|"FIST"|"POINT"|"TWO"|"THREE"|None

    Stateful: each finger latches into EXT or CURL and only switches when
    the flex value crosses the threshold by `hysteresis`. The pattern
    matching is then performed on the latched states.
    """

    _FINGERS = ("index", "middle", "ring", "pinky")

    def __init__(self, calib: Optional[PostureCalibration] = None):
        self.calib = calib or PostureCalibration()
        # Per-finger latched state: "EXT" or "CURL". Default to EXT so a
        # cold start with rest values lands on the OPEN posture.
        self._finger_state: Dict[str, str] = {f: "EXT" for f in self._FINGERS}

    def _update_finger_state(self, finger: str, value: float) -> str:
        """Stateful EXT/CURL transition with hysteresis around the
        per-finger threshold. The boundaries collapse to the raw
        threshold if hysteresis is 0 (jitter-prone — set hysteresis > 0
        to suppress flicker)."""
        em = float(self.calib.ext_max[finger])
        cm = float(self.calib.curl_min[finger])
        h = max(0.0, float(self.calib.hysteresis))
        ext_to_curl_at = min(1.0, cm + h)   # must rise ABOVE this to flip
        curl_to_ext_at = max(0.0, em - h)   # must drop BELOW this to flip
        last = self._finger_state.get(finger, "EXT")
        if last == "EXT" and value > ext_to_curl_at:
            new_state = "CURL"
        elif last == "CURL" and value < curl_to_ext_at:
            new_state = "EXT"
        else:
            new_state = last
        self._finger_state[finger] = new_state
        return new_state

    def classify(self, flex: Dict[str, float]) -> Optional[str]:
        states: Dict[str, str] = {}
        for f in self._FINGERS:
            states[f] = self._update_finger_state(f, float(flex[f]))

        is_ext = {f: states[f] == "EXT" for f in self._FINGERS}
        is_curl = {f: states[f] == "CURL" for f in self._FINGERS}

        if all(is_ext[f] for f in self._FINGERS):
            return "OPEN"
        if all(is_curl[f] for f in self._FINGERS):
            return "FIST"
        if is_ext["index"] and is_curl["middle"] and is_curl["ring"] and is_curl["pinky"]:
            return "POINT"
        if is_ext["index"] and is_ext["middle"] and is_curl["ring"] and is_curl["pinky"]:
            return "TWO"
        if is_ext["index"] and is_ext["middle"] and is_ext["ring"] and is_curl["pinky"]:
            return "THREE"

        return None
