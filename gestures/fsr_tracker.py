# gesture/fsr_tracker.py
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple
from gesture.models import FsrEvent


@dataclass
class FsrConfig:
    debounce_ms: int = 30
    tap_max_ms: int = 200
    double_tap_gap_ms: int = 350
    hold_min_ms: int = 500
    chord_gap_ms: int = 400  # for sequences like INDEX then MIDDLE


@dataclass
class _ButtonState:
    pressed: bool = False
    last_change_ms: int = 0
    press_start_ms: int = 0
    last_release_ms: int = 0
    last_tap_ms: int = 0
    pending_single_tap: bool = False


class FsrTracker:
    """
    Raw input: dict like {"L": {"INDEX": bool, ...}, "R": {...}}
    Output:
      - single_event: FsrEvent (TAP/DOUBLE_TAP/HOLD/PRESS/RELEASE) or None
      - sequence_event: list[FsrEvent] if chord recognized (optional)
    """

    def __init__(self, config: Optional[FsrConfig] = None):
        self.cfg = config or FsrConfig()
        self.state: Dict[Tuple[str, str], _ButtonState] = {}
        self._seq_buffer: List[FsrEvent] = []
        self._seq_last_ms: int = 0

    def _get(self, hand: str, finger: str) -> _ButtonState:
        key = (hand, finger)
        if key not in self.state:
            self.state[key] = _ButtonState()
        return self.state[key]

    def update(self, raw_pressed: Dict[str, Dict[str, bool]], now_ms: int) -> Dict[str, Optional[object]]:
        emitted: Optional[FsrEvent] = None
        emitted_seq: Optional[List[FsrEvent]] = None

        # Process all buttons
        for hand, fingers in raw_pressed.items():
            for finger, is_down in fingers.items():
                st = self._get(hand, finger)

                # Debounce: only accept transitions after debounce_ms
                if is_down != st.pressed:
                    if now_ms - st.last_change_ms < self.cfg.debounce_ms:
                        continue

                    # Transition accepted
                    st.last_change_ms = now_ms
                    prev = st.pressed
                    st.pressed = is_down

                    if (not prev) and is_down:
                        # PRESS
                        st.press_start_ms = now_ms
                        emitted = FsrEvent(hand=hand, finger=finger, action="PRESS",
                                           is_pressed=True, duration_ms=0, timestamp_ms=now_ms)

                    elif prev and (not is_down):
                        # RELEASE
                        st.last_release_ms = now_ms
                        press_dur = now_ms - st.press_start_ms
                        emitted = FsrEvent(hand=hand, finger=finger, action="RELEASE",
                                           is_pressed=False, duration_ms=press_dur, timestamp_ms=now_ms)

                        # Decide TAP if short
                        if press_dur <= self.cfg.tap_max_ms:
                            # Double-tap detection
                            if st.pending_single_tap and (now_ms - st.last_tap_ms) <= self.cfg.double_tap_gap_ms:
                                st.pending_single_tap = False
                                emitted = FsrEvent(hand=hand, finger=finger, action="DOUBLE_TAP",
                                                   is_pressed=False, duration_ms=press_dur, timestamp_ms=now_ms)
                                self._push_seq(emitted)
                            else:
                                st.pending_single_tap = True
                                st.last_tap_ms = now_ms
                                # We'll finalize the single TAP after double_tap_gap_ms passes
                        else:
                            # long press that is not HOLD stream (HOLD is streamed while pressed)
                            st.pending_single_tap = False

        # Finalize any pending single taps whose double-tap window expired
        for (hand, finger), st in self.state.items():
            if st.pending_single_tap and (now_ms - st.last_tap_ms) > self.cfg.double_tap_gap_ms:
                st.pending_single_tap = False
                emitted = FsrEvent(hand=hand, finger=finger, action="TAP",
                                   is_pressed=False, duration_ms=0, timestamp_ms=st.last_tap_ms)
                self._push_seq(emitted)

        # Generate HOLD stream events for buttons currently held past threshold
        # (One HOLD per update tick at most, prioritizing earliest held; tweak if you want multiple)
        hold_candidate = self._best_hold(now_ms)
        if hold_candidate and emitted is None:
            emitted = hold_candidate

        # Recognize chord/sequence if buffer has 2 and timing ok
        emitted_seq = self._maybe_emit_sequence(now_ms)

        return {"single": emitted, "sequence": emitted_seq}

    def _best_hold(self, now_ms: int) -> Optional[FsrEvent]:
        best = None
        best_dur = 0
        for (hand, finger), st in self.state.items():
            if st.pressed:
                dur = now_ms - st.press_start_ms
                if dur >= self.cfg.hold_min_ms and dur > best_dur:
                    best_dur = dur
                    best = FsrEvent(hand=hand, finger=finger, action="HOLD",
                                    is_pressed=True, duration_ms=dur, timestamp_ms=now_ms)
        return best

    def _push_seq(self, ev: FsrEvent) -> None:
        # build simple 2-event sequences with gap limit
        if not self._seq_buffer:
            self._seq_buffer = [ev]
            self._seq_last_ms = ev.timestamp_ms
            return
        if ev.timestamp_ms - self._seq_last_ms <= self.cfg.chord_gap_ms:
            self._seq_buffer.append(ev)
            self._seq_last_ms = ev.timestamp_ms
        else:
            self._seq_buffer = [ev]
            self._seq_last_ms = ev.timestamp_ms

    def _maybe_emit_sequence(self, now_ms: int) -> Optional[List[FsrEvent]]:
        if len(self._seq_buffer) >= 2:
            # Emit exactly 2-event sequence and clear
            seq = self._seq_buffer[:2]
            self._seq_buffer = []
            return seq
        # Expire buffer if stale
        if self._seq_buffer and (now_ms - self._seq_last_ms) > self.cfg.chord_gap_ms:
            self._seq_buffer = []
        return None