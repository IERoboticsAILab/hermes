"""Standalone smoke test: spawn Vuer and load the staged ROSbot URDF.

Run:
    python ros_version/src/hermes_viz/scripts/smoke_vuer_urdf.py

Expected: server logs a URL on stdout; opening it in a browser shows the
ROSbot model. Ctrl-C to exit.

CAUTION: Vuer 0.1.x API note — Urdf constructor signature and static_root
behaviour should be verified against `python3 -c "from vuer.schemas import
Urdf; help(Urdf)"` before running. Adjust src/key kwargs if the API differs.
"""
from __future__ import annotations

import asyncio
from pathlib import Path

from vuer import Vuer, VuerSession
from vuer.schemas import DefaultScene, Urdf


ASSETS = Path(__file__).resolve().parents[1] / "hermes_viz" / "assets" / "rosbot_urdf"
URDF_PATH = ASSETS / "rosbot.urdf"


def main() -> None:
    if not URDF_PATH.exists():
        raise SystemExit(f"URDF not found at {URDF_PATH}. Did Task 9 staging run?")

    app = Vuer(static_root=str(ASSETS))  # serve assets dir as /static

    @app.spawn(start=True)
    async def boot(sess: VuerSession):
        # Vuer's Urdf schema; mesh paths are relative to the URDF's directory
        # via Vuer's static_root served at /static/.
        sess.set @ DefaultScene(
            Urdf(src="/static/rosbot.urdf", key="rosbot_demo"),
        )
        while True:
            await asyncio.sleep(1.0)


if __name__ == "__main__":
    main()
