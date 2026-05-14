# H.E.R.M.E.S 3D Visualization — Digital-Twin MVP

**Date:** 2026-05-14
**Author:** Saleh Abd-Elrahman (with Claude)
**Target ship date:** Thesis defense (~2026-05-25, ~1.5 weeks out)
**MVP priority:** Real-to-sim digital twin

## 1. Purpose

A browser-based 3D visualization of the H.E.R.M.E.S lab that mirrors the live state of the wearables and the ROSbot swarm in real time. The system serves three audiences:

1. **Thesis defense (primary)** — a recognizable, polished 3D scene that lets the audience watch the operator and the swarm in one frame while the defense narration runs.
2. **Development debugging (secondary)** — an HUD overlay that surfaces ESP-NOW latency, packet rate, gesture confidence, and vest motor activation so the system can be diagnosed at a glance during dev.
3. **Replay & analysis (free)** — ROS bag playback through the standard ROS 2 tooling republishes the same topics the visualization subscribes to, so any recorded session can be replayed in the same viewer with zero extra code.

Out of scope for this spec: simulation-driven control (the visualization is read-only; it does not feed commands back into the system).

## 2. Success criteria

A run is successful for the MVP when, on the lab Raspberry Pi 5 with the full wearable stack live:

1. Both gloves' position and orientation are visible as floating hand meshes that track operator motion with no perceptible lag (< 100 ms end-to-end latency from glove sample to scene update at p50).
2. All four ROSbots (`r1`–`r4`) appear in the scene at their OptiTrack poses and update at ≥ 10 Hz with no perceptible jitter.
3. A per-robot fading trail visualizes the last ~20 s of motion.
4. The debug HUD displays current values for: per-glove packet rate, end-to-end latency (p50/p95, rolling 5 s), current gesture label + confidence, six vest motor zone indicators, and the deadman state badge.
5. Camera presets `1` / `2` / `3` switch between front-elevated / top-down / cinematic orbit without dropping below 30 FPS in Chromium on the Pi 5.
6. A recorded `.bag` of `/hermes/*` topics played back via `ros2 bag play` reproduces the same visualization sequence.

## 3. Decisions locked in during brainstorming

| Decision | Choice |
|---|---|
| Tech stack | Vuer (Python-native browser 3D) |
| Runtime host | Pi 5 runs both ROS and Vuer; Chromium displays locally via HDMI |
| Wearable representation | Two floating low-poly hand meshes, no body; vest as HUD only |
| ROSbot count | 4 real bots (`r1`–`r4`); `r5`/`r6` deferred |
| Camera default | Front-elevated (audience perspective) |
| Lab geometry source | Photo-textured planes built by hand from user-provided rough dimensions |
| Robot mesh source | Existing ROSbot URDF converted once to GLB |
| MVP additions confirmed late | Per-robot motion trails (added after initial scoping) |

## 4. Architecture

### 4.1 Process topology

```
                       Pi 5 (existing services unchanged)
   ┌────────────────────────────────────────────────────────────┐
   │  vest_serial_bridge_node → gesture_pipeline_node           │
   │       ↓                                                     │
   │  /hermes/raw_input, /hermes/command_packets,                │
   │  /hermes/swarm_intent, /hermes/robot_state_beacon,          │
   │  /hermes/vest_serial_tx                                     │
   │       ↓ (DDS, localhost)                                    │
   │  ┌────────────────────────────────────────┐                 │
   │  │ hermes_viz_bridge (NEW, rclpy)         │                 │
   │  │  subscribes → transforms → pushes      │                 │
   │  │  state via Vuer Python API             │                 │
   │  └─────────────────┬──────────────────────┘                 │
   │                    ↓ (in-process / WebSocket)               │
   │  ┌────────────────────────────────────────┐                 │
   │  │ Vuer server (NEW, port 8012)           │                 │
   │  │  serves Three.js scene to Chromium     │                 │
   │  └─────────────────┬──────────────────────┘                 │
   │                    ↓ HTTP/WS                                │
   │  ┌────────────────────────────────────────┐                 │
   │  │ Chromium --kiosk http://localhost:8012 │  ───► HDMI / proj │
   │  └────────────────────────────────────────┘                 │
   └────────────────────────────────────────────────────────────┘
```

### 4.2 Why a separate bridge node instead of putting rclpy directly in the Vuer process

Vuer's scene loop is event-driven; rclpy's executor wants its own thread / spin loop. Keeping them in separate processes (or at least separate Python threads with explicit queueing) avoids head-of-line blocking when one side stutters. The bridge owns the ROS executor; Vuer owns the rendering. The two are connected by Vuer's Python API which is non-blocking.

### 4.3 The bridge is read-only

The bridge subscribes only. It publishes no topics, runs no services, exposes no parameters that affect runtime behavior of the wearable stack. If it crashes or is killed, the rest of the H.E.R.M.E.S system is unaffected. This is a hard invariant.

## 5. Scene composition

### 5.1 Lab geometry

Hand-modeled from rough lab dimensions (to be supplied — see §11). Four wall planes textured with photographic material (color/texture from the lab photos), a floor plane with a subdued grid, and a rectangular floor overlay highlighting the OptiTrack tracking volume.

**Polygon budget:** ≤ 200 tris for the lab box. The lab is backdrop, not the subject.

### 5.2 ROSbots

The existing ROSbot URDF is converted **once, offline**, to a single GLB via `urdf2gltf` (or equivalent — confirmed during implementation). The GLB is checked into `hermes_viz/assets/rosbot.glb`. Four instances are placed in the scene, one per OptiTrack-tracked robot.

**Polygon budget:** ≤ 10k tris per bot, ≤ 40k tris total for the swarm.

### 5.3 Floating hands

Two stylized low-poly hand meshes (left / right). Position and orientation driven by the glove IMU quaternion. Finger curl in MVP is binary per-hand (fully open vs. fully closed, blended from the flex sensor sum). Per-joint articulation is Phase 2.

**Polygon budget:** ≤ 5k tris per hand.

### 5.4 Per-robot motion trails

Each bot keeps a ring buffer of its last `N` (default 200) pose samples. The trail renders as a polyline whose alpha fades linearly from the bot's accent color at the head to transparent at the tail.

- Update on every pose update (~10 Hz × 4 = 40 segments/s aggregate).
- HUD toggle to enable/disable trails globally and adjust `N`.
- Trail color matches a per-bot accent palette (see §6.3).

### 5.5 Lighting

One directional light (warm, slightly above front), one low-intensity ambient. **No shadows** (Pi GPU budget). The lab photo textures already carry their own lighting cues.

## 6. Data bindings

### 6.1 Topic-to-scene map

| Topic | Type | Rate (typ.) | Bridge transform | Scene effect |
|---|---|---|---|---|
| `/hermes/raw_input` | `std_msgs/String` (JSON) | 20–50 Hz | parse JSON → split L/R glove fields | hand mesh pose + finger curl + FSR grip tint |
| `/hermes/robot_state_beacon` | `std_msgs/String` (JSON) | ~10 Hz per robot | parse JSON → `(robot_id, x, y, θ)` | bot pose + push to trail ring buffer |
| `/hermes/swarm_intent` | `std_msgs/String` | event | passthrough label | HUD: gesture label, formation goal marker on floor |
| `/hermes/command_packets` | `std_msgs/String` (JSON) | event | parse confidence | HUD: gesture confidence bar |
| `/hermes/vest_serial_tx` | `std_msgs/String` | event | parse `V1,<seq>,<m1..m6>` | HUD: six motor zone indicators light up |

### 6.2 Latency measurement

The bridge stamps a `bridge_recv_ts` on each `/hermes/raw_input` message. If the underlying JSON carries a `glove_sample_ts` from the ESP32 (it does today), end-to-end latency = `bridge_recv_ts - glove_sample_ts`. A rolling 5 s window holds the last samples; p50 and p95 are computed on every HUD tick (every 250 ms).

If the upstream glove timestamp is missing or unreliable for a given run, the HUD falls back to displaying packet inter-arrival jitter only and labels the latency line as "unavailable" — never silently shows a wrong number.

### 6.3 Color palette

Bot accents: `r1` teal, `r2` amber, `r3` rose, `r4` indigo. (Hex values picked during implementation to match the existing presentation palette in `presentation/`.) HUD chrome stays neutral (#1a1a1a panel, #f6f6f4 text) so the bot accent colors stay readable.

## 7. Debug HUD

A right-anchored panel, ~280 px wide, rendered as an HTML overlay above the Vuer canvas. Sections, top to bottom:

1. **Deadman badge** (large, ~80 px tall) — red `STOPPED` or green `LIVE`. Driven by `/hermes/raw_input` "deadman" field; goes red if the field is false **or** the topic is silent for > 500 ms.
2. **Latency** — p50 / p95 in ms, rolling 5 s.
3. **Packet rate** — Hz per glove (`L`, `R`), based on inter-arrival of `/hermes/raw_input` parsed sides.
4. **Gesture** — current label (large text) + horizontal confidence bar.
5. **Vest motors** — six labeled circles, fill when the corresponding `m1..m6` byte is non-zero. Fade-out animation over 200 ms after release.
6. **Trails** — checkbox + slider for `N` (50–1000).

The HUD is implemented as a Vuer `Html` overlay, styled with inline CSS in `hud/hud.css`. Independently testable: it accepts a state object via a single `update(state)` call.

## 8. Camera

Three presets baked into the scene:

| Key | Preset | Position (m, lab frame) | Look-at |
|---|---|---|---|
| `1` | Front-elevated (default) | (0, -3, 2) | (0, 0, 0.3) |
| `2` | Top-down | (0, 0, 6) | (0, 0, 0) |
| `3` | Orbit | auto-orbits radius 4 m, height 1.8 m, period 30 s | (0, 0, 0.3) |
| `R` | Reset to preset 1 | — | — |

Numbers above are placeholders pending real lab dimensions; final values calibrated during implementation against the chosen reference frame (assumed: OptiTrack origin, +X right, +Y forward, +Z up).

## 9. ROS bag handling

No bag-specific code in MVP. Standard tooling:

```bash
# Record
ros2 bag record -o sessions/$(date +%Y%m%d_%H%M%S) \
  /hermes/raw_input /hermes/robot_state_beacon \
  /hermes/swarm_intent /hermes/command_packets \
  /hermes/vest_serial_tx

# Replay (with the viz stack already running)
ros2 bag play sessions/<name>
```

The bridge listens to the same topics either way. Phase 2 stretch: a scrub/seek control inside the HUD that wraps `ros2 bag play` and exposes seek/rate controls.

## 10. Package layout

```
ros_version/src/hermes_viz/
├── hermes_viz/
│   ├── __init__.py
│   ├── viz_bridge_node.py        # rclpy node, owns subscriptions
│   ├── scene/
│   │   ├── __init__.py
│   │   ├── builder.py            # constructs Vuer scene graph
│   │   ├── lab.py                # lab walls / floor / OptiTrack volume
│   │   ├── robots.py             # 4 GLB instances + trails
│   │   └── hands.py              # 2 hand meshes + finger curl logic
│   ├── hud/
│   │   ├── hud.html              # HUD template
│   │   ├── hud.css               # styling
│   │   └── state.py              # state struct + update(state) entry point
│   ├── transforms.py             # JSON → scene state (pure, unit-tested)
│   └── assets/
│       ├── rosbot.glb
│       ├── hand_left.glb, hand_right.glb
│       └── lab/                  # wall textures, floor texture
├── launch/
│   └── viz.launch.py             # starts vuer + bridge + chromium
├── test/
│   ├── test_transforms.py        # pure transforms
│   ├── test_hud_state.py         # HUD state aggregation
│   └── test_bridge_integration.py # replay-bag-against-stub-renderer
├── package.xml
├── setup.py
└── README.md
```

Each file owns a single concept. Anything larger than ~250 lines should be split.

## 11. Inputs required from the user before implementation starts

These can be gathered in parallel with the writing-plans phase:

1. **Lab dimensions** — rough W × D × H, OptiTrack volume W × D, position of any landmark (e.g., workbench) that should appear in the scene. A hand-drawn floorplan with measurements is enough.
2. **Lab photos** — 3–5 photos covering all four walls and the floor, enough to sample wall color and any visible signage.
3. **ROSbot URDF path** — confirm the canonical URDF used by the lab today and the mesh it references.
4. **OptiTrack frame convention** — confirm axis orientation (assumed +X right / +Y forward / +Z up; if different we adjust scene coordinates).
5. **`/hermes/raw_input` JSON schema** — a sample message captured from a live run so the bridge parsing is wired against the real shape, not an assumed one.

## 12. Performance budget (Pi 5)

| Resource | Budget |
|---|---|
| Scene polygon total | ≤ 60k tris |
| Texture memory | ≤ 8 MB total |
| Lights | 1 directional + 1 ambient, no shadows |
| Target FPS | 30 FPS sustained in Chromium |
| ROS subscriber queue depth | 10 per topic (lossy — newest wins) |
| HUD update tick | 4 Hz (250 ms) |
| Trail buffer | 200 × 4 bots = 800 segments |

If Pi 5 fails to hit 30 FPS at MVP scene complexity, fallback is to keep the bridge on the Pi and move only the Vuer server + Chromium to a laptop on the same WiFi (no architectural change; one config flag).

## 13. Testing strategy

### 13.1 Unit (`pytest`, fast)

- `transforms.py`: JSON → scene state for every topic. Includes malformed-input cases (truncated JSON, missing fields, NaN values) → expect graceful no-op, not crash.
- `hud/state.py`: rolling-window latency math (insert N samples → expect known p50/p95), packet-rate decay.

### 13.2 Integration (replay-based, slower)

- Record a small `.bag` (≤ 30 s) covering: deadman on/off, all gestures, a formation change.
- Test plays the bag against a **stub Vuer client** that records every scene state mutation. Assert the bot pose timeline matches the bag, the HUD latency series stays bounded, no exceptions raised.

### 13.3 Manual smoke (pre-defense checklist)

1. Boot Pi, source workspace, run `ros2 launch hermes_viz viz.launch.py`.
2. Chromium opens kiosk; scene renders.
3. Start wearable stack; verify hands track, bots track, HUD updates, deadman toggles correctly.
4. Switch camera presets `1` / `2` / `3` / `R`.
5. Kill bridge; confirm wearable stack continues unaffected (read-only invariant).

## 14. Risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Pi 5 can't hit 30 FPS in Chromium | Medium | Demo stutters | Laptop fallback path is one config flag (§12). Test on Pi within the first 3 days of implementation, not the last 3. |
| URDF → GLB conversion produces broken meshes | Medium | Bots invisible | Convert and check into repo on day 1 of implementation, before any other work depends on it. Manual visual check in a GLB viewer before committing. |
| `/hermes/raw_input` JSON schema differs from assumption | Low | Bridge parsing wrong | Pull a real sample message first (§11.5). Schema-validate at startup; log + skip malformed messages. |
| WiFi/DDS issues if laptop fallback engaged | Low | Bot poses lag | Switch back to Pi-local rendering; document it as a known fallback degradation. |
| Hand IMU drift makes hand visualization look "off" | Medium | Audience distracted | Apply a slow drift correction (low-pass to identity quaternion on prolonged stationarity). If still bad, document and acknowledge in narration. |
| Defense projector reads colors differently from monitor | High | HUD readability suffers | Test on the actual projector at least 48 h before defense. Keep HUD palette high-contrast (dark panel, bright text). |

## 15. Out of scope (Phase 2)

- Per-joint finger articulation from flex sensor channels (MVP: binary open/close).
- Photoreal lab textures, ambient occlusion, shadows.
- OptiTrack camera meshes rendered in the scene.
- In-viz bag scrubbing UI (use `ros2 bag play` directly).
- Multi-viewer or VR (Vuer supports this, but it's not in the defense flow).
- `r5` / `r6` bots until those OptiTrack configs go live.
- Driving simulation forward when the wearables are offline (this is a read-only digital twin, not a simulator).

## 16. Open questions

None blocking. The five inputs in §11 are pre-implementation gathering, not unresolved design decisions.
