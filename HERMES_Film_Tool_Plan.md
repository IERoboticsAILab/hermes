# H.E.R.M.E.S Film Tool — Detailed Plan
### Working name: **MAESTRO** — *embodied camera choreography for film & virtual production*

*Detailed expansion of Idea 6B from [HERMES_Startup_Ideas.md](./HERMES_Startup_Ideas.md). Prepared 2026-06-29. Market/competitor facts cited inline; treat exact figures as directional.*

---

## 1. Thesis (one line)

**Directors and DPs think with their hands; the tools make them think in keyframes and joint values.** MAESTRO lets a filmmaker *perform* a camera move — single shot or a coordinated multi-camera/drone swarm — with their hands and body, and have it drive a virtual camera (previs / virtual production), a physical motion-control rig, or a drone fleet, with haptic feedback for limits and collisions. **Author the move once, by gesture; execute it everywhere.**

---

## 2. The problem (researched, not assumed)

Three documented, specific pain points — this is where the money and frustration already are:

1. **Programming camera moves is a specialist bottleneck.** On MRMC rigs (the industry standard), "the camera operator programs the move by using a controller or manually entering values for each joint in the software called FLAIR," or builds it in Maya/Unreal and exports it ([MRMC Flair](https://www.mrmoco.com/motion-control/flair/)). "Arranging complex camera movements within traditional tools is time-consuming and can be a significant bottleneck in the production process" ([VFX Voice / previs](https://vfxvoice.com/taking-previs-tools-to-the-next-level/)). Translation: the creative intent ("swoop in like *this*") dies in a slow technical translation step done by a different person.

2. **Robotic cinematography is expensive and gated by expertise.** The MRMC **Milo** is the Academy-Award-winning industry standard; the **Bolt** is the high-speed workhorse; even the "affordable" Bolt Jr+ package starts ~**£85k**, and productions typically *rent* rigs with a "full team of expert operators and engineers" ([MRMC](https://www.mrmoco.com/motion-control/milo/), [Mocolab](https://www.mocolab.com/service/bolt-jr-motion-control-rental)). The intuitive-authoring layer is the missing democratizer.

3. **Multi-camera / swarm shots barely have an authoring tool at all.** Single virtual cameras are well served (below); **coordinated multi-unit camera choreography** — several drones or rigs moving as one designed ensemble — is hand-built, ad hoc, and rare. This is exactly the gap H.E.R.M.E.S's swarm-coordination engine was built for.

**The market is real and growing:** virtual production **~$2.84B (2025) → ~$12.25B (2033), ~20.4% CAGR** ([Grand View](https://www.grandviewresearch.com/industry-analysis/virtual-production-market)); previs has gone fully real-time on Unreal Engine ([VFX Voice](https://vfxvoice.com/how-previs-has-gone-real-time/)); LED-volume virtual production (StageCraft/*The Mandalorian* lineage) is now mainstream but "pricey" and operationally immature — "some shows super successful; others a bloodbath because people were unprepared" ([Hollywood Reporter](https://www.hollywoodreporter.com/business/digital/volume-house-of-the-dragon-stage-mandalorian-1235244158/)). Tools that compress prep time and de-risk the shoot have a clear buyer.

---

## 3. The product — three modes, one engine

MAESTRO is a gesture-authoring + execution layer. Same input (your hands + body, captured by the glove/IMU; haptic vest for feedback), three outputs:

| Mode | What the filmmaker does | What it drives | Primary buyer |
|---|---|---|---|
| **A. Previs / Virtual Camera** | Performs the shot by hand; sculpts and refines camera paths in real time inside Unreal/Maya | A virtual camera (and *multiple* virtual cameras at once) in the 3D scene | Previs houses, virtual-production stages, indie filmmakers |
| **B. Motion-Control Execution** | Same authored move, sent to a physical rig; live "conduct" or tweak on set | MRMC Bolt/Milo-class rigs (via Flair export / control API) | Rental houses, commercial/VFX shoots |
| **C. Swarm / Multi-Unit** | Choreographs several camera drones or rigs as a coordinated ensemble | Camera-drone fleet or multiple rigs, formation-aware | Aerial-cine units, large-format spectacle, sports/live |

**The strategic spine:** *author once, execute across all three.* A move designed by gesture in previs (Mode A) exports cleanly to the physical rig on shoot day (Mode B) — closing the previs→production gap that today requires re-translation. That continuity is the defensible product story, not gesture alone.

---

## 4. MVP — start narrow (Mode A, single + dual virtual camera)

Do **not** start by controlling £85k physical robots or FAA-regulated drones. Start in software, where iteration is free and risk is zero:

**MVP = an Unreal Engine / Maya plugin that turns H.E.R.M.E.S glove+IMU gestures into authored virtual-camera moves, with haptic feedback for path constraints.**

- Filmmaker performs a camera move with their hand → MAESTRO records it as a smooth, editable spline → playable, tweakable, exportable as standard camera animation (FBX/USD/Unreal sequence).
- Add the wedge competitors can't easily match: **drive two+ virtual cameras as a coordinated pair** from one performance (the swarm engine, applied to cameras).
- Haptic vest buzzes when the authored path would exceed a real rig's reach/speed envelope — so previs moves are *physically executable*, not fantasy. This single feature ties Mode A to Mode B and is genuinely novel.

**Why this MVP:** zero regulatory exposure, no hardware to ship beyond the wearable you already have, a demo any DP understands in 30 seconds, and it slots into the Unreal-based pipeline the whole industry already standardized on.

---

## 5. How it maps to H.E.R.M.E.S (you've built ~70% of this)

| MAESTRO need | Existing H.E.R.M.E.S component | Work required |
|---|---|---|
| Capture hand/body intent | Glove flex + IMU + classification pipeline | Repurpose from discrete gestures → **continuous trajectory capture** (the main new work) |
| Turn motion into a camera path | Gesture→intent layer | New: spline fitting, smoothing, easing, scale mapping (hand-space → world-space) |
| Coordinate multiple cameras | **Swarm auction + formation engine** | Reuse almost directly — cameras are "agents" |
| Feedback without a screen | Haptic vest (6 motors) | Remap to: rig limits, collision, end-of-travel, beat sync |
| Output to industry pipeline | — | New: Unreal/Maya plugin, USD/FBX export, MRMC Flair interop |

The hardest, most defensible parts — multi-agent coordination and a working low-latency wearable→intent loop — **already exist**. The new work is domain plumbing (DCC plugins, trajectory math, rig APIs), not core R&D.

---

## 6. Who buys it (customer, buyer, budget)

| Segment | Who signs | Why they buy | Budget reality |
|---|---|---|---|
| **Previs / virtual-art-dept studios** (The Third Floor, Halon, Day for Nite, NVIZ) | Previs supervisor / studio owner | Faster iteration, fewer specialist hours per shot | Per-seat software they expense to productions |
| **Virtual-production stages / LED volumes** | Stage technical director | De-risk shoot days, compress prep, sell a premium "embodied authoring" capability | Stage-level tooling budgets; high willingness to pay |
| **Motion-control rental houses** (MrMoco, Mocolab, Quinn) | Owner / lead operator | Lower the expertise barrier → more clients can use their rigs | Bundle into rental day-rate |
| **Indie / commercial / music-video** | DP / director | Robotic-grade moves without a robotics team | Affordable subscription tier |
| **Aerial-cinematography units** (Mode C, later) | Drone-cine lead | The only real multi-drone *shot-authoring* tool | Project licensing |

**Beachhead pick: previs/virtual-production studios.** They feel the authoring-bottleneck pain daily, already live in Unreal, buy software readily, and are the trendsetters the rest of the industry copies. Land them first.

---

## 7. Competitive landscape & honest differentiation

| Competitor | What they own | Where MAESTRO differs |
|---|---|---|
| **Virtual-camera systems** — Glassbox (DragonFly), NVIZ ARENA, Vcam | Intuitive *single* virtual camera via tracked tablet/phone — "one operator can lens up/down, pull focus, cue animation" ([NVIZ/postPerspective](https://postperspective.com/%E2%80%A8glassboxs-virtual-camera-toolset-for-unreal-unity-maya/)) | **This is the real incumbent.** They nail the single hand-held virtual camera. MAESTRO's edge must be: (1) **hands-free** gesture, not holding a device; (2) **multi-camera/swarm** choreography they don't do; (3) the **previs→physical-rig bridge** with haptic executability checking |
| **MRMC Flair** | The standard for *executing* moves on physical rigs | Flair is a programming environment for specialists; MAESTRO is an **intuitive authoring front-end** that exports *to* Flair. Position as complementary, then partner |
| **AI camera planning** (CinePreGen, research) | Automated/generative camera moves | MAESTRO keeps the **human in creative control** — performance, not generation. Complementary, not competing |
| **Drone-cine operators** | Aerial shots, mostly single-drone, manually flown | No one offers *multi-drone shot authoring*. Mode C is open field — but it's niche and later |

**The honest read:** Glassbox/NVIZ already solved the *single intuitive virtual camera* well — so "gesture-controlled virtual camera" alone is **not** enough of a wedge. MAESTRO must win on the **three things they don't do: hands-free embodiment, multi-unit coordination, and one authored move that runs on both virtual and physical cameras.** If you can't deliver at least two of those convincingly, this is a feature, not a company.

---

## 8. Business model & pricing

- **Mode A (software):** SaaS, per-seat. Indie tier (~$50–150/mo) for reach + brand; studio/pro tier (per-seat + support) for margin. The glove is a low-cost hardware add-on or BYO.
- **Mode B/C:** project/rig licensing + integration services; revenue-share or referral with rental houses.
- **Long game:** become the **standard authoring layer** that exports to every rig and engine — the "intuitive front end for camera motion," monetized on seats and execution. High margin, recurring, hardware-light. Mirrors the SDK logic of Idea 2, scoped to film.

---

## 9. Go-to-market — phased

1. **Phase 0 — Demo & validate (0–4 mo).** Build the Mode-A MVP. Get it in front of 3–5 previs supervisors / DPs. One killer demo video (gesture → swooping multi-camera move in Unreal) is your fundraising and marketing asset — entertainment sells on spectacle.
2. **Phase 1 — Design partners (4–12 mo).** 2–3 previs/VP studios as paid design partners. Nail the Unreal plugin, export pipeline, and the haptic-executability hook. Publish results; present at FMX / SIGGRAPH / Cine Gear.
3. **Phase 2 — Physical bridge (12–24 mo).** MRMC Flair interop (Mode B). Partner with a rental house. Now you own author→execute continuity.
4. **Phase 3 — Swarm (24 mo+).** Multi-drone Mode C with an aerial-cine partner who already holds FAA waivers. Highest spectacle, smallest market — do it for prestige and the crossover back to Ideas 1/6A.

---

## 10. Regulatory & IP

- **Modes A/B are essentially unregulated** — software and on-set robotics under existing studio safety practice. This is the cleanest regulatory profile of any H.E.R.M.E.S direction.
- **Mode C (drones)** invokes **FAA Part 107.35** (one pilot / multiple aircraft needs a waiver, ~90-day review) — but you **ride your aerial partner's existing waiver** rather than holding your own ([FAA waivers](https://www.faa.gov/uas/commercial_operators/part_107_waivers)). Indoor stage drone work is far simpler.
- **IP:** the defensible filings are around **continuous gesture→multi-camera-trajectory mapping** and **haptic physical-executability feedback during authoring**. File before the SIGGRAPH/FMX reveals.

---

## 11. Risks — honest

| Risk | Severity | Mitigation |
|---|---|---|
| Glassbox/NVIZ already own intuitive single-cam | High | Win on hands-free + multi-unit + physical bridge, or don't bother |
| Film is project-based, slow to adopt new tools | Med | Land previs studios (trendsetters); SaaS smooths revenue |
| Gesture precision vs. keyframe exactness | Med | Position as *authoring/blocking*, with frame-accurate refinement after — perform the intent, polish the curve |
| Niche market ceiling | Med | It's a wedge/brand play, not a unicorn; or fold into the broader swarm-control company |
| You're a roboticist, not a film-industry insider | High | A film-industry co-founder or advisor is close to mandatory here |

---

## 12. Roadmap & milestones

| When | Milestone | Proves |
|---|---|---|
| Month 2 | Continuous trajectory capture from glove/IMU working | Core tech repurposed |
| Month 4 | Unreal plugin: gesture → editable virtual-camera spline → export | MVP exists; demo-able |
| Month 6 | Dual-camera coordinated move from one performance | The differentiator works |
| Month 9 | Haptic executability check (path vs. rig envelope) | Previs↔physical bridge concept |
| Month 12 | 2 paid design-partner studios | Market pull, not just push |
| Month 18 | MRMC Flair export driving a real rig | Author→execute continuity |
| Month 24 | Multi-drone choreography with partner | Mode C / spectacle |

---

## 13. Funding

- **Phase 0–1 is cheap** (software + existing wearable) — angel / film-tech micro-VC / a creative-tech grant covers it. Entertainment-tech and media-innovation funds exist and love a viral demo.
- Avoid heavy hardware capital until Mode B/C, and even then offload onto rental-house and aerial partners.
- A strong demo reel is worth more than a deck here — budget for *one* genuinely beautiful proof shot.

---

## 14. Why now

Real-time previs on Unreal is now the industry default ([VFX Voice](https://vfxvoice.com/how-previs-has-gone-real-time/)); LED-volume virtual production is mainstream but operationally painful and prep-heavy ([Hollywood Reporter](https://www.hollywoodreporter.com/business/digital/volume-house-of-the-dragon-stage-mandalorian-1235244158/)); robotic camera moves are standard but still gated behind specialist programming ([MRMC Flair](https://www.mrmoco.com/motion-control/flair/)). The pipeline is real-time and engine-based — exactly the substrate a live, embodied, multi-camera authoring tool plugs into. The enabling shift already happened; the intuitive human front-end is the missing piece.

---

### Bottom line
A focused, low-regulation, demo-friendly way to commercialize the H.E.R.M.E.S engine in film. **The MVP is pure software (Unreal plugin, single→dual virtual camera) with near-zero risk.** It wins only if it beats the strong virtual-camera incumbents on the three things they don't do — **hands-free embodiment, multi-unit choreography, and the author-once-run-anywhere bridge to physical rigs.** Best run with a film-industry co-founder, funded lean off a stunning demo, and treated either as a standalone media-tech company or as the entertainment beachhead that proves the swarm engine before it points at bigger markets.

---

### Sources
[Grand View — virtual production market](https://www.grandviewresearch.com/industry-analysis/virtual-production-market) · [MRMC Flair](https://www.mrmoco.com/motion-control/flair/) · [MRMC Milo](https://www.mrmoco.com/motion-control/milo/) · [MRMC Bolt cost](https://www.mrmoco.com/what-is-the-cost-of-a-bolt-robot/) · [Mocolab Bolt Jr+ rental](https://www.mocolab.com/service/bolt-jr-motion-control-rental) · [VFX Voice — previs tools](https://vfxvoice.com/taking-previs-tools-to-the-next-level/) · [VFX Voice — real-time previs](https://vfxvoice.com/how-previs-has-gone-real-time/) · [postPerspective — Glassbox/NVIZ virtual camera](https://postperspective.com/%E2%80%A8glassboxs-virtual-camera-toolset-for-unreal-unity-maya/) · [Hollywood Reporter — VP growing pains](https://www.hollywoodreporter.com/business/digital/volume-house-of-the-dragon-stage-mandalorian-1235244158/) · [Autodesk — VP workflows](https://www.autodesk.com/design-make/articles/virtual-production) · [CinePreGen — AI camera previs](https://arxiv.org/html/2408.17424v1) · [FAA Part 107 waivers](https://www.faa.gov/uas/commercial_operators/part_107_waivers)
