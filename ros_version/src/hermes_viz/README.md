# hermes_viz

Browser-based 3D digital twin for H.E.R.M.E.S. The viz subscribes to existing
`/hermes/*` topics and renders the lab, the 4 ROSbots, and the operator's
floating hand meshes in real time. **Read-only** — the bridge publishes
nothing; killing it never disturbs the wearable stack.

## Run

```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hermes_viz
source install/setup.bash

ros2 launch hermes_viz viz.launch.py
```

Chromium auto-opens to `http://localhost:8012`. If it doesn't (or you're
running headless), open that URL in any browser on the Pi.

### Laptop fallback (better performance for the defense)

Run Vuer + the bridge on a laptop sharing the Pi's `ROS_DOMAIN_ID`:

```bash
ros2 launch hermes_viz viz.launch.py vuer_host:=0.0.0.0 autostart_browser:=false
# then on the laptop: open http://<pi-ip>:8012
```

## Camera presets

Hotkeys (with the browser focused):

- `1` — front-elevated (default, audience POV)
- `2` — top-down tactical
- `3` — cinematic auto-orbit
- `R` — reset to preset 1

## Replay a session

```bash
ros2 bag record -o sessions/$(date +%Y%m%d_%H%M%S) \
  /hermes/raw_input /hermes/robot_state_beacon \
  /hermes/swarm_intent /hermes/command_packets \
  /hermes/vest_serial_tx
# ...record, Ctrl-C when done.

# Replay against the running viz:
ros2 bag play sessions/<dirname>
```

Because the bridge listens to the same topics live or replayed, the viz
mirrors the recorded session with no extra config.

## Pre-defense smoke checklist

Run this from the actual defense Pi at least 48 hours before the defense.

- [ ] `colcon build --packages-select hermes_viz` finishes clean.
- [ ] `pytest src/hermes_viz/test/` is all green.
- [ ] `ros2 launch hermes_viz viz.launch.py` opens Chromium to a recognizable
      lab scene with 4 ROSbots, 2 floating hands, and the HUD on the right.
- [ ] Start the wearable stack:
      `ros2 launch hermes_control wearables_pi.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=921600`
      Verify in the viz:
      - Both hand meshes follow your real hand motion (left for IMU; right for finger curl/grip).
      - All 4 ROSbots track their OptiTrack poses.
      - HUD deadman badge toggles between LIVE (green) and STOPPED (red).
      - HUD latency, packet rate, gesture label, and motor dots update.
- [ ] Cycle camera presets: `1`, `2`, `3`, `R`.
- [ ] Toggle trail visibility from the HUD (off/on, change N).
- [ ] Kill the viz bridge with Ctrl-C — confirm the wearable stack continues
      running unaffected (`ros2 topic echo /hermes/swarm_intent` still ticking).
- [ ] Test on the actual projector at the defense room. Confirm:
      - The HUD is readable from 5 m away.
      - Colors don't wash out under the room lighting.
- [ ] Record a 30-second bag and replay it through the viz to confirm replay path.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Chromium opens but scene is empty | Vuer server not up yet | Wait 2-3s after launch; refresh the page |
| ROSbots all stuck at origin | No beacon messages | Check `ros2 topic hz /hermes/robot_state_beacon`; verify OptiTrack v2 is active |
| Hands floating in mid-air, no motion | Left glove not fresh | Check `ros2 topic echo /hermes/raw_input` — `imu` dict must contain key `"L"` |
| HUD latency shows "unavailable" | Glove timestamp missing | Same as above; the bridge can't compute end-to-end without a glove-side `time_ms` |
| Pi struggling at < 30 FPS | Pi GPU saturated | Switch to laptop fallback: `vuer_host:=0.0.0.0 autostart_browser:=false` and view from a laptop |

## Architecture quick-reference

```
[ROS Pi: existing wearable stack — unchanged]
   /hermes/raw_input, /hermes/robot_state_beacon,
   /hermes/swarm_intent, /hermes/command_packets, /hermes/vest_serial_tx
        ↓ DDS (localhost)
[hermes_viz_bridge — NEW] subscribes only, publishes nothing
        ↓ in-process
[Vuer server — NEW] :8012
        ↓ HTTP/WS
[Chromium kiosk]  →  HDMI / projector
```

## Phase 2 backlog

Work explicitly deferred until after the defense:

- Per-joint finger articulation from individual flex channels (MVP is binary open/close).
- Photo-textured lab walls (MVP is plain warm gray).
- OptiTrack camera meshes rendered in the scene.
- In-viz bag scrubbing UI (use `ros2 bag play` directly).
- Multi-viewer / VR / mobile-friendly modes.
- `r5` / `r6` ROSbots once their OptiTrack configs go live.
