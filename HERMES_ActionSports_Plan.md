# H.E.R.M.E.S Action-Sports Videography — Detailed Plan
### Working name: **SLIPSTREAM** — *direct the shot, not the drone*

*Companion to [HERMES_Startup_Ideas.md](./HERMES_Startup_Ideas.md) and [HERMES_Film_Tool_Plan.md](./HERMES_Film_Tool_Plan.md). Prepared 2026-06-29. Market/competitor facts cited inline; treat exact figures as directional.*

---

## 1. Thesis (one line)

Action-sports filming is stuck between **dumb autonomy** (DJI/HoverAir auto-follow that loses you the moment the action gets real) and **expert FPV piloting** (cinematic but needs a skilled pilot in goggles who isn't you). **SLIPSTREAM is the missing middle:** the athlete — or a spotter — *directs the shot by gesture*, hands-free and eyes-up, mid-action, with haptic feedback for where the drone is — and coordinates *multiple* drones for multi-angle captures in a single take.

This is arguably the **most natural consumer/prosumer fit for H.E.R.M.E.S**, because here the eyes-up/hands-busy constraint isn't a nice-to-have — it's physically mandatory. You cannot hold a controller while you ski, surf, or send a jump.

---

## 2. The problem (researched, not assumed)

Today there are two poles and nothing good in between:

**Pole 1 — Autonomous follow drones are "good enough for a Sunday jog," not for real action.**
The autonomous-follow *leader* — **Skydio — exited the consumer market in 2023** to chase defense/enterprise, leaving the category's best tracking tech gone ([UAV Coach](https://uavcoach.com/skydio-consumer-exit/)). DJI (ActiveTrack 360) and HoverAir X1 now lead consumer follow ([UAV Coach follow-me 2026](https://uavcoach.com/follow-me-drone/)) — but their tracking breaks on exactly what defines the sport. Per DJI's own guidance and independent testing:
- "**Sudden acceleration or unpredictable direction changes can throw off the drone's predictive algorithm — a classic issue when trying to track fast-action sports**" ([DJI support](https://support.dji.com/help/content?customId=en-us03400006813)).
- It "**can get stuck in trees**," misses "thin objects like small tree branches, power lines, or guy-wires," and the Mavic 4 Pro "**favors higher altitudes**" and flies "**way too cautious**" — backing off instead of getting the dramatic close shot ([DroneXL](https://dronexl.co/2025/06/02/dji-mavic-4-pro-activetrack-flight/), [DC Rainmaker Gauntlet test](https://www.dcrainmaker.com/2025/05/dji-mavic4-pro-activetrack-testing-review.html)).
- Framing is fixed and generic: it follows, it doesn't *direct*. No "orbit me on this turn," "drop low and chase," "pull wide for the cliff."

**Pole 2 — FPV piloting gets the epic shot, but you can't be the athlete.**
Cinematic chase FPV is now the gold standard — the **2026 Winter Olympics used FPV drones to follow skiers and snowboarders** ([Digital Camera World](https://www.digitalcameraworld.com/buying-guides/best-fpv-drone)). But it requires a highly skilled pilot flying via goggles, fully occupied. It can't be done solo, can't be done by the athlete, and the skill barrier is steep.

**The gap:** *directed creative control without piloting skill, operable while you're the one doing the sport, across multiple drones at once.* Nobody serves it.

---

## 3. Market

| Market | Size | Source |
|---|---|---|
| Consumer camera drones | **~$8.99B (2024) → ~$32.6B (2031), ~20.2% CAGR** | [Reanin](https://www.reanin.com/reports/consumer-camera-drones-market) |
| Action cameras | **~$7.3B (2025) → ~$19.6B (2034), ~11.6% CAGR** (Insta360 now #1 over GoPro) | [Fortune Business Insights](https://www.fortunebusinessinsights.com/action-camera-market-111731), [HDIN](https://www.hdinresearch.com/news/559) |
| 360° cameras | **~$2.24B (2025) → ~$5.98B (2031), ~17.8% CAGR** | [Mordor](https://www.mordorintelligence.com/industry-reports/360-degree-camera-market) |

**Honest caveat:** there is no clean public TAM for the *specific* "directed action-sports drone control" sub-niche — it doesn't exist as a category yet (which is the opportunity *and* the risk). The numbers above are the adjacent pools you'd draw from. The serviceable wedge is smaller and must be proven, not assumed from these headline figures.

---

## 4. The product — tiered by who's directing

One engine (gesture→intent + swarm coordination + haptic feedback), three go-to-market tiers:

| Tier | Who directs | Setup | Why it's compelling |
|---|---|---|---|
| **T1 — Pro multi-drone capture (B2B, lead with this)** | A single operator conducts a *fleet* of drones by gesture | Productions, events, resorts, broadcasters | One person choreographs 3–5 coordinated angles in one take — today that needs 3–5 FPV pilots. The swarm engine is the whole point |
| **T2 — Prosumer "athlete-director"** | The athlete or a spotter directs 1–2 drones by hand, mid-run | Serious creators, athlete teams, coaches | Better-than-auto-follow directed shots, no film crew, no piloting skill, hands-free |
| **T3 — Consumer (long-term)** | Solo athlete, simple gestures | Mass creator market | Huge, but gated by the platform problem (§7). Don't start here |

**The hero demo and the defensible core is T1:** *one operator gesture-conducting a coordinated drone camera crew.* No consumer product and almost no pro setup does **coordinated multi-drone follow** — it's the direct application of the H.E.R.M.E.S swarm engine, it's B2B (real budgets, not racing DJI on $400 hardware), and it's the shot nobody else can get.

---

## 5. MVP — prove the multi-drone gesture choreography (T1)

**MVP = one operator, wearing the glove + haptic vest, gesture-conducting 2–3 open-platform camera drones to capture a coordinated multi-angle action sequence (e.g., a single MTB or ski run filmed simultaneously wide + orbit + chase).**

- Gestures map to *shot intent per drone or per group*: assign roles ("you orbit, you chase, you hold wide"), adjust live ("tighten," "pull back," "switch sides").
- Haptic vest signals drone state without looking — out of frame, low battery, lost subject, approaching limit.
- The swarm engine keeps the drones coordinated and collision-aware relative to each other and the subject.

**Why this MVP:** it leads with the one thing that's genuinely unserved and maps directly to your existing swarm coordination + wearable stack. It's a B2B demo that sells itself in one clip, and it sidesteps the brutal consumer-hardware war with DJI.

---

## 6. How it maps to H.E.R.M.E.S

| SLIPSTREAM need | Existing component | Work required |
|---|---|---|
| Hands-free shot direction mid-action | Glove + IMU + gesture classification | Adapt to a rugged, glanceable, low-vocabulary "shot-intent" gesture set |
| Coordinate multiple drones | **Swarm auction + formation engine** | High reuse — drones are agents; add subject-relative framing geometry |
| Per-drone shot intent → flight | Gesture→intent layer | New: map intent to camera-relative flight paths (orbit/chase/dolly/crane) |
| Feedback without a screen | Haptic vest | Remap to frame/battery/tracking/limit alerts; must survive motion + weather |
| Talk to the drones | ESP-NOW/serial transport experience | New: integrate with an open drone autopilot (PX4/ArduPilot/MAVLink) |

Again, the hard/defensible parts (multi-agent coordination, wearable→intent loop) exist. The new work is subject-relative cinematography geometry + autopilot integration + ruggedization.

---

## 7. The make-or-break strategic constraint: the platform problem

**This decides whether the company is possible. Foreground it.**

DJI owns consumer/prosumer with cheap, vertically-integrated, *closed* drones — you **cannot** inject third-party control into a DJI flight controller. Competing with DJI on hardware is suicide. So SLIPSTREAM must be a **control layer**, and that forces a platform choice:

| Option | Pro | Con |
|---|---|---|
| **A. Build on open autopilots (PX4/ArduPilot/MAVLink)** | Full control; you own the stack | You inherit reliability/polish vs DJI; you (or a partner) must supply the airframe |
| **B. Partner with an open/SDK-friendly drone maker** | Skip airframe R&D | Dependency; few consumer options expose flight control |
| **C. Target FPV / custom-rig pros** | They already fly open, custom, multi-drone setups | Niche, but it's exactly the T1 buyer |

**Recommended:** start at **C + A** — the pro/FPV world already flies open, customizable, multi-drone rigs and already pays for crews and pilots. They are the T1 customer, they're not locked to DJI, and they value the differentiated shot. Win there, then assess whether a partnership (B) ever opens the prosumer tier. **Do not bet the company on cracking DJI's closed consumer ecosystem.**

---

## 8. Competitive landscape & honest differentiation

| Competitor | What they own | SLIPSTREAM edge |
|---|---|---|
| **DJI ActiveTrack** | Consumer auto-follow at scale, cheap | Directed (not just follow), close/aggressive framing, **multi-drone**, hands-free intent — and works on open platforms DJI's closed stack won't allow you to touch anyway |
| **HoverAir X1** | Effortless solo selfie-follow | Same — plus it's a toy-tier single drone; no creative direction, no coordination |
| **Skydio (gone from consumer)** | *Was* best autonomy; left in 2023 | The vacated "smart follow" high ground — but pursue the *directed/multi* axis they never did, not a re-fight on pure autonomy |
| **FPV pilots / chase crews** | The epic cinematic shot, full control | No goggles, no expert pilot, the athlete can direct, and **one operator runs many drones** instead of one-pilot-one-drone |
| **Cable cams / gimbals / wearable POV (GoPro/Insta360)** | Specific shot types | Complementary, not competing — SLIPSTREAM is aerial direction; POV cams ride along |

**Honest read:** for the *casual solo* user, DJI/HoverAir are "good enough" and free-with-the-drone — T3 is a hard, late fight. SLIPSTREAM's real, winnable edge is **directed multi-drone capture for people who care about the shot** (T1, then T2). That's where auto-follow visibly fails and FPV is expensive and unscalable.

---

## 9. Business model

- **T1 (B2B):** sell/lease the control system (wearable + software + integration) to production houses, resorts, event/broadcast crews; or operate it as a service (you bring the gesture-conducted drone crew). Project/day-rate or annual license.
- **T2 (prosumer):** wearable + app subscription, paired with supported open-platform drones; possibly a bundled rig.
- **Software-forward over time:** the durable asset is the *coordination + intent + framing* software; the wearable is the interface. Highest margin if it becomes the standard control layer for multi-drone cine.

---

## 10. Go-to-market — phased

1. **Phase 0 (0–4 mo):** T1 MVP. Film ONE breathtaking multi-angle single-take action clip (MTB/ski/surf) shot by one gesture-operator running 3 drones. That clip is the company's entire early marketing and fundraising.
2. **Phase 1 (4–12 mo):** Design partners — an action-sports production house + a ski resort or event (Red Bull–type, X Games, freeride media). Paid pilots; refine the gesture vocabulary and ruggedness in the field.
3. **Phase 2 (12–24 mo):** Productize T1; expand to T2 with a supported open-platform drone and athlete-director mode. Build the creator brand (athletes are the influencers).
4. **Phase 3 (24 mo+):** Evaluate T3/consumer only if a platform partnership (Option B) makes it viable.

---

## 11. Regulatory & policy

Lighter than defense/medical, real but navigable:
- **FAA Part 107** for commercial filming (the T1/T2 buyers are commercial). Since 2021, licensed pilots can fly **over people and over moving vehicles** without a waiver under conditions — notably **Category 1 micro-drones (<0.55 lb, no exposed rotors)** for over-people ([FAA Part 107](https://www.faa.gov/newsroom/small-unmanned-aircraft-systems-uas-regulations-part-107)). This pushes you toward small, prop-guarded drones for athlete-proximate shots.
- **Operating from a moving vehicle** is restricted to sparsely-populated areas — relevant for chase/follow scenarios; design around it.
- **Multiple drones / one operator** needs a **Part 107.35 waiver** (the same one drone shows use, ~90-day review) — central to the T1 multi-drone model; budget for it and lean on partners who hold one ([FAA waivers](https://www.faa.gov/uas/commercial_operators/part_107_waivers)).
- **BVLOS:** prohibited without waiver today; the proposed **Part 108 (NPRM Aug 2025)** may loosen this — a tailwind to watch ([Knobbe Martens](https://www.knobbe.com/blog/proposed-faa-rule-to-allow-drone-operations-beyond-visual-line-of-sight/)).
- **Safety/liability:** drones near fast-moving people is the core risk. Prop guards, geofencing, collision avoidance, and fail-safe behaviors aren't features — they're table stakes and a liability shield.

---

## 12. Risks — honest

| Risk | Severity | Mitigation |
|---|---|---|
| **Platform problem** — DJI closed, open platforms less polished | **High** | Lead with FPV/pro (open rigs) T1; don't fight DJI on consumer hardware |
| Auto-follow is "good enough" for casual users | High | Don't target casual; target directed/multi-drone where it visibly fails |
| Gesture reliability while the operator/athlete is in motion | High | Start with a *spotter*-operated T1 (operator is stationary), not athlete-in-motion; prove mid-action later |
| Safety incident near athletes | High | Micro-drones, guards, avoidance, fail-safes from day one |
| Niche/seasonal, project-based revenue | Med | B2B licensing + service; build creator brand for durability |
| You're a roboticist, not an action-sports/film insider | Med–High | Co-founder/advisor from action-sports media is near-mandatory |

---

## 13. Roadmap & milestones

| When | Milestone | Proves |
|---|---|---|
| Month 2 | Single drone driven by gesture intent (orbit/chase/wide) on an open platform | Core control loop works |
| Month 4 | 2–3 drones coordinated, gesture-conducted, one operator | The differentiator (swarm) works |
| Month 5 | The hero clip: one operator, one take, multi-angle action sequence | Marketing + fundraising asset |
| Month 9 | Field trial with an action-sports production partner | Real-world ruggedness |
| Month 12 | Productized T1; haptic framing feedback reliable outdoors | Sellable B2B product |
| Month 18 | T2 athlete/spotter mode; supported drone bundle | Market expansion |
| Month 24+ | Part 107.35 operations standardized; platform-partner talks | Scale path |

---

## 14. Why now

Skydio vacated the smart-follow consumer high ground in 2023; DJI's auto-follow demonstrably can't handle real action-sports motion; FPV proved the *appetite* for dynamic aerial action capture (Winter Olympics 2026) but exposed its cost/skill ceiling; the adjacent markets (camera drones ~20% CAGR, action cams, 360°) are large and growing; and proposed **Part 108 BVLOS** rules may soon expand what's legal. The demand and the enabling tech exist — the **intuitive, multi-drone, hands-free direction layer** is what's missing.

---

### Bottom line
Of all the H.E.R.M.E.S consumer-facing directions, this has the **strongest paradigm fit** — hands-busy/eyes-up is physically required, not optional — and a **clear, documented gap** between dumb auto-follow and expensive FPV. **Lead with B2B multi-drone capture (T1)** on open platforms, where the swarm engine is the unique value and DJI's closed ecosystem is irrelevant. The make-or-break issue is the **platform problem** — solve it by starting with the pro/FPV world, not by fighting DJI for consumers. Bring an action-sports media co-founder, fund it lean off one stunning clip, and treat consumer (T3) as a later prize, not the starting line.

---

### Sources
[Reanin — consumer camera drones market](https://www.reanin.com/reports/consumer-camera-drones-market) · [Fortune Business Insights — action cameras](https://www.fortunebusinessinsights.com/action-camera-market-111731) · [HDIN — Insta360 overtakes GoPro](https://www.hdinresearch.com/news/559) · [Mordor — 360° cameras](https://www.mordorintelligence.com/industry-reports/360-degree-camera-market) · [UAV Coach — Skydio consumer exit](https://uavcoach.com/skydio-consumer-exit/) · [UAV Coach — follow-me drones 2026](https://uavcoach.com/follow-me-drone/) · [DC Rainmaker — Mavic 4 Pro ActiveTrack Gauntlet test](https://www.dcrainmaker.com/2025/05/dji-mavic4-pro-activetrack-testing-review.html) · [DroneXL — Mavic 4 Pro too cautious](https://dronexl.co/2025/06/02/dji-mavic-4-pro-activetrack-flight/) · [DJI support — ActiveTrack limits](https://support.dji.com/help/content?customId=en-us03400006813) · [Digital Camera World — FPV / Winter Olympics 2026](https://www.digitalcameraworld.com/buying-guides/best-fpv-drone) · [FAA Part 107](https://www.faa.gov/newsroom/small-unmanned-aircraft-systems-uas-regulations-part-107) · [FAA Part 107 waivers](https://www.faa.gov/uas/commercial_operators/part_107_waivers) · [Knobbe Martens — proposed Part 108 BVLOS](https://www.knobbe.com/blog/proposed-faa-rule-to-allow-drone-operations-beyond-visual-line-of-sight/)
