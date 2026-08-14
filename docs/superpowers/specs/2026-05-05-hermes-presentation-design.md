# H.E.R.M.E.S Presentation Design Spec

**Date:** 2026-05-05  
**Format:** Single self-contained HTML file  
**Duration:** ~8 minutes + live demo  
**Audience:** Mixed technical/non-technical

---

## Visual Design System

| Property | Value |
|---|---|
| Background | `#080d1a` (deep navy-black) |
| Primary accent | `#00BFFF` (electric blue) |
| Body text | `#ffffff` |
| Muted/subtitle | `#8892a4` |
| Glow effect | blue `box-shadow` on key elements |
| Body font | Inter (Google Fonts) |
| Code/label font | JetBrains Mono (Google Fonts) |
| Transitions | None (instant slide change) |
| Progress indicator | Thin blue bar at bottom of viewport |

---

## Navigation & Controls

| Key | Action |
|---|---|
| `→` / `←` | Next / previous slide |
| `S` | Toggle speaker notes panel (slides in from right) |
| `F` | Toggle fullscreen |
| Slide counter | Bottom-right, format `3 / 14` |

Speaker notes panel: dark overlay, right-side drawer, does not affect slide layout. Not visible on projected screen in dual-display mode.

---

## Slide Structure

### Slide 1 — Title
**Layout:** Centered hero  
- Large title: `H.E.R.M.E.S`
- Subtitle: `Human-Encoded Recognition and Motion for Embodied Swarms`
- Tagline: `Turning human motion into swarm intent`
- Presenter name + date bottom-right  
**Notes:** Opening line, introduce yourself, state the 8-minute talk structure.

---

### Slide 2 — Hook
**Layout:** Full-screen, big text only  
- Blue accent quote: *"What if commanding a swarm was as natural as pointing your hand — like a conductor?"*  
**Notes:** Pause here. Let the question land before advancing.

---

### Slide 3 — Problem
**Layout:** Two-column  
- Left: visual illustration of 1-human → N-robot breakdown (SVG/CSS diagram)
- Right: 3 bullets — cognitive load, switching between robots, no natural group commands  
**Notes:** "One-to-one control doesn't scale. As team size grows, the operator becomes the bottleneck."

---

### Slide 4 — Research Question
**Layout:** Centered, large italic text  
- *"How can wearable gestures and motion be used to control a multi-robot swarm in a way that is fast, selective, and understandable to the operator?"*  
**Notes:** One sentence. Don't explain it — let it breathe.

---

### Slide 5 — System Overview
**Layout:** Full-width architecture diagram  
- Data flow: `Left Glove + Right Glove → Vest ESP32 (ESP-NOW) → Raspberry Pi / ROS 2 → Swarm Intent → ROSbots → Haptic Feedback`
- Rendered as styled HTML/CSS flow diagram with blue connectors
- Label each node briefly  
**Notes:** "Keep this high level — no ROS graph detail yet. Mixed audience."

---

### Slide 6 — Wearable Interface
**Layout:** Three photo slots + captions  
- Slot 1: Left glove — *"mode selection, IMU, deadman safety"*
- Slot 2: Right glove — *"discrete commands via FSR"*
- Slot 3: Vest — *"ESP-NOW hub + haptic output (6 motors)"*
- Each slot: clearly marked `[ PHOTO ]` placeholder with caption below  
**Notes:** "The left glove handles mode state and continuous IMU data. The right glove sends discrete commands. The vest is both the communications hub and the feedback device."

---

### Slide 7 — Control Modes
**Layout:** Clean table, 4 rows  

| Gesture | Mode | Effect |
|---|---|---|
| Open hand | DRIVE | Move selected robots |
| Point / POI | SELECT | Choose robot subset |
| Fist | FORMATION | Command line/group shape |
| (Brief mention) | PARAMS | Behavior tuning |

**Notes:** "Only cover DRIVE, SELECT, FORMATION in depth — PARAMS only if time allows."

---

### Slide 8 — Swarm Intelligence Layer
**Layout:** Two-column  
- Left: vertical abstraction stack diagram — `Gesture → Command Packet → Swarm Intent → Local Robot Execution`
- Right: key insight text — *"This is not hardware wiring. Each robot independently executes its part of the intent."*  
**Notes:** "This is the important abstraction. The operator issues intent, not wheel commands."

---

### Slide 9 — Validation
**Layout:** Three-column card layout  

**Input Reliability**
- Gesture recognition accuracy
- Deadman safety response
- Command success rate

**System Timing**
- End-to-end latency
- Robot response time

**Swarm Behavior**
- Selection accuracy
- Movement to target points
- Line formation accuracy

**Notes:** "Summarize numbers briefly here — the demo is the proof."

---

### Slide 10 — Demo Setup
**Layout:** Spatial diagram  
- 4 ROSbots labeled R1–R4 at point A
- OptiTrack coverage area indicated
- Points A, B, C marked
- Legend: what each point represents  
**Notes:** "Orient the audience before the demo starts. Tell them what success looks like."

---

### Slide 11 — Live Demo
**Layout:** Step checklist — stays on screen during demo  

```
☐ All robots at point A
☐ Step 1: Move ALL → B  ("global swarm control")
☐ Step 2: Select HALF → C  ("selective control")
☐ Step 3: Reselect ALL → Line formation  ("group coordination")
```

**Notes:** Full demo script:  
"First I'll show global control — moving all four robots from A to B..."  
"Now selective control — I'll select half the swarm and move only those to C..."  
"Finally group coordination — I'll reselect all and command a line formation..."  
"One interface. Three different levels of intent."  
**Fallback:** "On live OptiTrack and wireless hardware, I prepared a recorded run of the same demo so evaluation doesn't depend on room conditions."

---

### Slide 12 — Conclusion
**Layout:** Large centered text, two beats  
- Beat 1: *"H.E.R.M.E.S shows that one human can command a robot swarm through intent, not individual control."*
- Beat 2 (blue accent, larger): *"The goal of H.E.R.M.E.S is not to replace human decision-making. It is to give one human a language that a swarm can understand."*  
**Notes:** Return to the hook. Pause after the final line.

---

### Appendix Slide A1 — Fallback Materials
**Layout:** Four slots  
- `[ HARDWARE PHOTO ]`
- `[ ROS NODES SCREENSHOT ]`
- `[ DEMO VIDEO LINK ]`
- Measured results summary (text, pulled from Slide 9)  
**Notes:** Use if any part of live demo fails. State it calmly and professionally.

---

### Appendix Slide A2 — Limitations
**Layout:** Three bullets  
- OptiTrack dependency — assumes external localization
- Latency increases beyond 4 robots
- Validation strongest on 4-robot lab setup  
**Notes:** "Honest and short. Don't apologize — every system has scope."

---

## File Output

- **Path:** `presentation/hermes-presentation.html`
- Single self-contained file (fonts loaded from Google Fonts CDN — requires internet; fallback to system fonts if offline)
- All slide content, CSS, and JS inline
- Photo slots: `<div class="photo-slot">` with visible label and dashed border

---

## Out of Scope

- PDF export (screenshot manually if needed)
- Animation/transitions between slides
- Embedded video (link only in appendix)
