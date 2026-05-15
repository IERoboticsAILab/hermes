"""Per-robot pose ring buffer for trail rendering. Pure stdlib."""

from __future__ import annotations

from collections import deque


class Trail:
    """Bounded FIFO of (x, y) positions. Newest at the right; oldest evicted."""

    def __init__(self, max_len: int):
        if max_len < 1:
            raise ValueError("max_len must be >= 1")
        self._buf: deque[tuple[float, float]] = deque(maxlen=max_len)

    def push(self, x: float, y: float) -> None:
        self._buf.append((float(x), float(y)))

    def resize(self, max_len: int) -> None:
        if max_len < 1:
            raise ValueError("max_len must be >= 1")
        # deque maxlen is immutable; rebuild keeping the most recent samples.
        kept = list(self._buf)[-max_len:]
        self._buf = deque(kept, maxlen=max_len)

    def clear(self) -> None:
        self._buf.clear()

    def points(self) -> list[tuple[float, float]]:
        return list(self._buf)
