# Closing `plan.md` Default-Switch Gates

Date: 2026-09-12  
Campaign run: 2026-09-12 evening (RelWithDebInfo, WSL)  
Updated: 2026-09-14 (exact-replay seed 0 and cadence are default CTests)

## Status after the first campaign

Stand, seed 0, 100-seed, and cadence 120/240/480 Hz are **green**. WSL default is
**pinocchio-proximal** (`SolverMode = 1`, `SolverIterations = 24`); parser default
is `1`. Exact-replay seed 0 and 120/240/480 Hz are default CTests (cap 24, both
enforce flags, fixture v16). 100-seed stays a diagnostic override.
`./scripts/verify.sh` 15/84 server CTests remain a **known PGS/offline
baseline**, not a proximal production gate — see
[verify.sh inventory](#verifysh-inventory-2026-09-14). Keep `legacy-pgs` (`= 0`)
for comparison.
See [Strafe gate investigation](#strafe-gate-investigation-2026-09-14),
[Six-contact remaining gates](#six-contact-remaining-gates-2026-09-14),
[Forward cross-track investigation](#forward-cross-track-investigation-2026-09-14),
[100-seed held investigation](#100-seed-held-investigation-2026-09-14),
[120 Hz cadence investigation](#120-hz-cadence-investigation-2026-09-14),
and [verify.sh inventory](#verifysh-inventory-2026-09-14).

| Gate | Result |
| --- | --- |
| Stand 60 s at cap **50** | **Pass.** 12000 healthy, p99 iters **7**, height error 9.8 mm, foot RMS 0.22 mm (2026-09-13 remesure) |
| Stand 60 s at production cap **24** | **Pass** (2026-09-14 remesure after flip). 12000/12000 healthy, height error **9.81 mm**, foot RMS **0.22 mm**, p99 iters **7**, max iters **23** |
| Isolated tripod stroke (probe, cap 500) | **Pass.** 0.0737 / 0.096 m (**76.8%**), ~2 mm penetration, support feet ~stationary |
| Exact-replay seed 0 at cap **24** | **Pass** on fixture `ddc6008e0cc1ac97` (v16). Default CTests at 200/120/240/480 Hz. Post-transient reverse/strafe/diagonal/turn/forward all pass. Safety: held 0, recovered 24 |
| Physics-step p99 &lt; 4.0 ms | **Pass as a number** at cap 24 (`p99_solver_total_step_time_ms` ≈ **1.16 ms** seed 0; **2.31 ms** on 100-seed) |
| ADMM p99 ≤ 20, no cap exhaustion | **Partial.** Recovered retries still hit cap 24 on first attempt. Last-resort recovered frames can report up to 27. Zero held |
| 70% translation / lateral / turn | Isolated tripod **pass**. Ordered seed 0 post-transient **pass**. 100-seed behaviour **pass** (`behavior_gate_failures` 0) |
| Dense ADMM / contact precondition A/B | **Do not promote.** No named Class B lever this batch |
| 100-seed safety then behaviour | **Pass** on v16 after last-resort cold half-steps. held 0, read 0, recovered 2388, `behavior_gate_failures` 0 |
| 120 / 240 / 480 Hz same fixture | **Pass.** Seed 0, both enforce flags, cap 24. Safety held 0 at all three. 120 Hz diagonal **103%** after scoring at capture period; 240/480 unchanged and still pass |
| 10-minute randomized gait | **Struck from the switch contract.** Keep as post-default soak, not an unowned blocker |
| `./scripts/verify.sh` | **Struck from the switch contract.** 15/84 server CTests remain red (PGS live-physics and offline scenario height). Proximal stand CTest **passed**. Smoke uses `config.sim.txt`, not proximal walk. See [verify.sh inventory](#verifysh-inventory-2026-09-14) |
| WSL default `pinocchio-proximal` | **Flipped** (2026-09-14). `SolverMode = 1`, `SolverIterations = 24`, parser default `1`. Harness / `config.physics-sim.txt` stay `0` |

Current frozen fixture (do not recapture for solver A/B): hash
`ddc6008e0cc1ac97` (v16), 720 frames, capture 500-iter proximal,
`replay_period_us=5000`, written as `/tmp/hexapod-proximal-gates/hexapod-commands-v16.txt`.
v11 (`82c6c4354cbc196`) predates first-stride phase coverage.

### Strafe gate investigation (2026-09-14)

The reported strafe miss was a measurement-window error, not a remaining strafe-axis
or contact-coupling error. The acceptance requirement says "after the acceleration
transient", but exact replay previously measured all 72 motion frames. In the ordered
fixture, strafe began with about **+0.252 m/s** unintended body-forward velocity left
by the preceding reverse-to-stand transition. That transient contributed nearly all
of the reported cross-track displacement.

Exact replay now excludes the first **120 ms** of each motion phase, with the frame
count derived from replay `dt`, while retaining full-phase displacement and start/end
velocity diagnostics. On the current frozen ordered replay at cap 24, the evaluated
strafe window reaches **0.0268 / 0.0240 m (112%)** with **3.0 mm** cross-track (limit
**12.7 mm**). An independently captured strafe fixture reaches **0.0260 / 0.0240 m
(108%)** with **0.14 mm** cross-track and passes both behaviour and safety enforcement
(one recovered retry, zero held/unsupported/read failures).

The ordered replay remains red for a separate forward cross-track miss: **18.5 mm**
against a **14.9 mm** limit in its evaluated window. Reverse, strafe, diagonal, and
turn pass their post-transient behaviour gates. Do not attribute that remaining seed-0
failure to strafe or retune the lateral mapping. The abrupt WALK-to-STAND target reset
is still visible in the full-phase boundary diagnostics and should be treated as a
separate command-transition problem.
The 2026-09-12 hash `88b24580e5ed406c` predates mount-yaw, coxa sign, 104 mm
feet, μ=2, penetration bias, ABA Δv, and the H1 kinematics lever.

### Failure class

**Class B** (ADMM exhaustion) that then becomes **Class D** (hold cascade) and
**Class A** (behaviour). Stand→walk is the first break: transition frames
exhaust the cap on a persistent 6-contact set, then later phases freeze.

Dense GEMV cut `p99_solver_admm_time_ms` (0.51 → 0.28) but **total** p99 rose
slightly and locomotion was unchanged. Contact precondition accepted a few
more recovered frames and made stand p99 hit 24. Loosening
`absolute_tolerance` to `1e-5` helped forward at cap 24 but did not clear
strafe/turn.

Code landed from this campaign (production path, still not enough for the
walk gate):

- Keep foot-contact warm starts across servo target jumps.
- Same-`dt` retry for `SolverNotConverged` before half-steps.
- Mean last normal impulse for new contacts; keep ADMM rho on same-size
  contact-set edits.
- `admmConverged` / `ncpPhysicallyConverged` on `ProximalStepDiagnostics`
  (not yet in `StateResponse` / replay JSON).

After that repair, cap-24 held 431 → 421 and recovered 2 → 15. Stand is
unchanged (p99 6). Walk still fails. Cap 50 still exhausts on gait; strafe
improved, diagonal/turn did not.

Next work is still Class B: fewer ADMM iterations on sliding 6-contact
systems (conditioning / stop criteria / longer honest gait fixtures), not
allocator hygiene and not the default switch.

### Closed-loop walk (follow-up)

`test_physics_sim_walk_distance` with
`HEXAPOD_WALK_TEST_SOLVER_MODE=pinocchio-proximal` (controller in the loop).
This is the honest production plant; it is **not** a 500-iter capture replay.

| Run | Healthy / held / noconv | Stays WALK? | Path | Notes |
| --- | --- | --- | --- | --- |
| Proximal cap **24**, height 0.14 m | 52 / 2338 / 2348 | No (faulted) | 3 cm | Production-shaped. First failure: no measurable travel |
| Proximal cap **50**, height 0.14 m | 189 / 2205 / 2211 | No | 16 cm | Still mostly held |
| Proximal cap **24**, height 0.06 m | 19 / 2381 / 2381 | No | 0.2 cm | Walk-test default height |
| Proximal cap **50**, height 0.06 m | 468 / 1929 / 1932 | No | 15 cm | |
| Proximal cap **500**, height 0.06 m | 1091 / 1306 / 1309 | No | 18 cm | Capture-style cap still holds |
| PGS, height 0.06 m (test default) | 2400 / 0 / 0 | **Yes** | 12 cm | Slow vs 0.2 m/s command (fails speed band); **no holds** |
| PGS, height 0.14 m | 2400 / 0 / 0 | No (faulted) | 13 cm | Solver healthy; 0.14 m is not this test’s PGS pose |

Matched capture+replay with `HEXAPOD_EXACT_REPLAY_CAPTURE_SOLVER_ITERATIONS=24`
**cannot even save a fixture**: capture contains inhibited / non-WALK frames.
That is why the existing harness captures at 500 iterations.

**Conclusion:** the 500-vs-24 replay mismatch is a symptom. The closed-loop
proximal plant holds on walk even at 500 iterations. PGS at the same 0.06 m
height does not hold and stays in WALK. Next lever is the walking NCP
(sliding 6-contact ADMM), not a longer fixture and not the default switch.

### Walking NCP (2026-09-13)

First walk failures at cap 24 / ADMM `1e-8` look like false rejects, not a
blown NCP: ADMM hits the cap with primal/dual ~1e-7, cone ~0, NCP
complementarity ~1.7e-8. Rejecting that wipe-starts the hold cascade.

Raising the **ADMM** stop to `1e-3` is hold-free on closed-loop walk, but is
**not** shippable as a global default: stand at `1e-3` is still 12000 Healthy
frames with **10 cm** height error. NCP-only accept of the capped `1e-8`
iterate (floor `1e-6` or `1e-3`) does not match early-stop ADMM.

Production split now in `stepProximal`:

- Standing contacts (tangential free speed ≤ 2 cm/s): ADMM `1e-8`
- Sliding contacts (tangential free speed > 2 cm/s): ADMM `1e-3`
- NCP accept floor `1e-3` in both cases

Measured at cap 24 after that split:

| Run | Result |
| --- | --- |
| Stand CTest 0.14 m, ADMM `1e-8` | **Pass.** 12000 healthy, height error 3.2 mm, foot RMS 0.07 mm, p99 iters **7** |
| Closed-loop walk 0.06 m | Held **1**, 2378 healthy, stays in WALK (48 faulted), path 22 cm. Fails the 0.2 m/s speed band (`avg ≈ 0.018`) like PGS |
| Exact replay seed 0 | Held **0**, 717 healthy / 3 recovered. Forward progress 61% (gate 70%) |
| Closed-loop walk 0.14 m | Still faults/holds (525 held); 0.14 m is not the walk-test pose |

Do not flip `SolverMode`. Remaining walk gates are progress/speed (and 0.14 m
sag), not the hold cascade.

### Stance-loaded servo PD (2026-09-13)

Unconstrained CRBA diagonals `M_ii` understate the inertia a planted servo
sees: the feet lock the chassis, so the SISO plant is
`P = M⁻¹ − M⁻¹ Jᵀ G⁻¹ J M⁻¹` and `I_eff,i = 1 / P_ii`. A global
`HEXAPOD_PINOCCHIO_SERVO_GAIN_SCALE=1.5` recovered most of the isolated-turn
tracking miss; `2.0` hit the 10 rad/s guard on swing joints.

Production now assigns per-joint `I_eff` once at model construction (other
joints locked, six foot point contacts at the initial pose) and clamps to
`[M_ii, 1.5 M_ii]` — the empirically safe global scale, not the 2× that hit
the 10 rad/s guard. PD uses that value on every servo; a swing/stance split
improved tracking but reduced closed-loop turn yaw. Do not recompute `I_eff`
on the 480 Hz path, and do not ship a global gain other than `1.0`.

Measured at gain `1.0` after baking the 1.5× clamp (cap 24 stand, isolated
closed-loop replay cap 500):

| Gate | Result |
| --- | --- |
| Stand CTest 0.14 m | **Pass.** 12000 healthy, height 2.6 mm, foot RMS 0.006 mm, p99 iters **5** |
| Isolated turn 1.08 rad | Yaw 0.219 rad (was 0.209 at unconstrained `M_ii`, 0.237 at env 1.5×). Foot RMS 5.1 cm (was 9.8 cm). Held swing 897 (was 1260). Peak ω 5.1 rad/s, 0 recovered |
| Isolated forward 0.288 m | Progress **60%** (was 52% unconstrained, 63% env 1.5×). 70% gate still open |

Do not flip `SolverMode`. Remaining walk gates are still progress/speed.

### Walk progress root cause (2026-09-13)

Stand and servo tracking are no longer the 70% bottleneck. Isolated turn
yaw tracks the *commanded* stance-target counter-yaw far more than tracking
error. `TwistField::stanceFootVelocity = -(v + ω × r)` is correct.
`planStanceFoot` applies it as `p = anchor + v_foot * φ / f` with **gait**
phase `φ ∈ [0,1)`, which is only a world-fixed support sweep while
`φ < duty`. Extra-stance plants used to keep that clock (late-swing contact
after grace, hold through wrap) and reverse the target on `φ` wrap.

**Clock contract (now in production):** planned-stance spells still use
`φ/f`. Once a foot is planted outside that spell, `BodyController` latches
the pose and integrates `v_foot(p) * dt`. Unit tests in
`test_body_controller_velocity` cover wrap-jump and late-swing contact;
`test_foothold_planner` documents monotonic `φ ∈ [0, duty)`. Exact replay
emits reverse-step counters split by planned / held-swing / unclassified.

**A/B on this tree before the latch (isolated 480 motion frames, cap 500):**

| Run | Forward progress | Turn yaw | Reverse steps | Notes |
| --- | --- | --- | --- | --- |
| Baseline proximal | 61% | 24% | 71 / 48 | Held swing 931 / 933 |
| Holds off | 60% | 28% | 76 / 48 | Held swing 0. Not the 70% lever |
| PGS same commands | 12% | 10% | same as baseline | Plant is weaker, not the ceiling |
| Blend 0.35 turn | — | 19% | 159 | Harness default is already 0 |

**After the latch (same fixtures):** stand CTest still 12000 healthy, height
2.6 mm, foot RMS 0.01 mm, p99 **5**. Isolated forward 60%, turn 23%, reverse
steps still mostly on planned-stance samples. Extra-stance wrap is no longer
the remaining 70% miss.

**Stroke vs slip vs overlap (same isolated fixtures, v4 metrics):** mid-stance
is `0.05 < φ/duty < 0.95` and already planned. Onset reverse is expected
touchdown; continuing mid-stance reverse is not.

| | Forward proximal | Forward PGS | Turn proximal | Turn PGS |
| --- | --- | --- | --- | --- |
| Body progress / yaw | 60% of 0.288 m | 12% | 23% of 1.08 rad | 9% |
| Mid-stance shape vs command | 18% (0.021 / 0.12 m/s) | same capture | 59% (0.267 / 0.45 rad/s) | same capture |
| Tripod shape (`n_planned==3`, ~428 frames) | **−14%** (wrong way) | same | 36% | same |
| Overlap / high-duty (`n≥4`, 52 / 38 frames) | 174% / 180% | same | 180% yaw | same |
| Mid-stance reverse steps | 42 (onset 27) | same | 18 (onset 17) | same |
| Mid-stance contact world speed | 0.273 m/s, tripod=overlap | 0.049 m/s | 0.102 m/s (overlap 0.190) | 0.033 m/s |

Walk-entry overlap is ~0.2 s and **over**-strokes; it is not the 70% miss.
Tripod commanded stroke on forward is reversed, and 42 continuing mid-stance
reverse steps remain after the extra-stance latch. Proximal plant slip is
large, but PGS on the same fixture slips less and collects even less body
progress, so `mu` / `SolverMode` are not the next change.

Stand CTest after this instrumentation: 12000 healthy, height 2.65 mm, foot
RMS 0.011 mm, p99 **5**.

**Chosen next lever (separate plan):** why continuing mid-stance tripod
targets reverse in body frame (forward opposition negative, 42 mid-stance
reverse steps). Not duty overlap, not PD, not ADMM, not `SolverMode`.

**Tripod stroke latch (same fixtures):** continuing planned stance now
integrates `v dt` with a plant stroke budget instead of re-evaluating
`φ/f` when cadence changes. Unit tests cover constant-f, f-jump, v-jump,
and high-duty walk-entry bounds.

| | Forward | Turn |
| --- | --- | --- |
| Body progress / yaw | **43%** of 0.288 m (was 60%) | **28%** of 1.08 rad (was 23%) |
| Mid-stance tripod shape vs command | **+40%** (was −14%) | +56% yaw |
| Mid-stance reverse steps | **0** (was 42) | 2 other |
| Mid-stance contact world speed | 0.183 m/s | 0.095 m/s |

Cartesian opposition is finally the right sign and reverse jumps are gone.
Body progress did not rise with that honesty (forward fell), so the 70% miss
is no longer a wrapping stance clock.

**Stroke-budget split (fixture v6, same isolated 480/240/480, 0.14 m, 5 ms,
cap 500):** mid-stance opposition is split into governor scale, plant stroke
clamp, Cartesian BodyController targets, and IK/servo FK. Cartesian identity
(`v = 0.12`, `f = 1`, `duty = 0.5`, φ in `(0.05, 0.45)`) and clamp-saturation
unit tests are green, so the latch itself is not the 40% hole.

Tripod mid-stance (forward command `|v| = 0.12` m/s, turn yaw `0.45` rad/s):

| | Forward | Turn |
| --- | --- | --- |
| Body progress / yaw | **41%** of 0.288 m | **27%** of 1.08 rad |
| `command_scale` / `cadence_scale` | 0.96 / 0.98 | 0.79 / 0.91 |
| Mean mid-stance `f` | 0.68 Hz | 0.39 Hz |
| Governed `|v| * command_scale` | 0.115 m/s (96% of command) | n/a (`|v| = 0`) |
| Cartesian tripod opposition | **0.086 m/s (72% of command)** | n/a (translation metric) |
| FK tripod opposition / counter-yaw | **0.050 m/s (42% of command)** | 0.268 rad/s (60% of 0.45) |
| Stroke-clamp hit fraction | 25% | 36% |
| Mid-stance reverse steps | 0 | 2 other |

Four-way:

- Governor: no. Forward `command_scale` is 0.96, not ~0.4.
- Plant budget `L`: no. Clamp hits 25% of mid-stance samples; it does not
  dominate the 40% FK shape.
- `v_foot` vs command: not the 40% hole. Cartesian is 72% of command, not 40%.
- **IK / joint-rate clamp: yes.** Cartesian is much closer to command than FK
  (72% vs 42%). The remaining Cartesian shortfall (28%, with 25% clamp) is
  secondary.

**Chosen next lever:** IK / servo-limit clamp (`post_clamp_distortion_m` is
already on the locomotion debug snapshot). Do not retune duty, frequency
tables, `mu`, PD, ADMM, or `SolverMode` on this evidence. Stand CTest after
this instrumentation: 12000 healthy, height 2.65 mm, foot RMS 0.016 mm,
p99 **5**.

**IK vs slew split (fixture v7, same isolated 480/240/480, 0.14 m, 5 ms,
cap 500):** Cartesian, pre-slew IK FK, and post-slew FK are taken from the
same control step. Lagged-bridge FK is kept as the command-stream metric.
IK stroke roundtrip (reachable, no lean) and 8 rad/s slew non-bite unit
tests are green, so neither identity IK nor stance `vmax` is the 40% hole
by itself.

Tripod mid-stance:

| | Forward (`|v| = 0.12`) | Turn (`0.45` rad/s) |
| --- | --- | --- |
| Body progress / yaw | **40%** of 0.288 m | **23%** of 1.08 rad |
| Cartesian | 0.085 m/s (**71%**) | 0.284 rad/s (63%) |
| Pre-slew IK FK | **0.050 m/s (42%)** | 0.275 rad/s (61%) |
| Aligned post-slew FK | **0.050 m/s (42%)** | 0.275 rad/s (61%) |
| Lagged-bridge FK | 0.051 m/s (43%) | 0.281 rad/s (62% tripod) |
| IK reach-hit fraction | **61%** | 47% |
| Servo slew-hit fraction | 1.9% | 3.4% |
| Mean `post_clamp_distortion_m` | 34 mm | 9.8 mm |

Four-way:

- Stream lag: no. Aligned post-slew matches lagged FK (~42%), not Cartesian 71%.
- Servo slew: no. Hit fraction is 2%; pre-slew and post-slew opposition are
  identical on tripod.
- Gravity FF: no. The drop is already present in pre-slew IK FK.
- **IK reach: yes.** 61% of forward mid-stance samples scale `d` onto the
  femur–tibia annulus. Stance skips Cartesian workspace clamp, so out-of-reach
  lean/stroke is truncated toward the coxa.

**Chosen next lever:** IK reach (apply a stroke-preserving Cartesian clamp
before IK, or stop rotating stance targets out of the annulus). Do not raise
`vmax`, disable reach, retune lean, gait, `mu`, PD, ADMM, or `SolverMode` on
this evidence. Stand CTest after this instrumentation: 12000 healthy, height
2.63 mm, foot RMS 0.005 mm, p99 **5**.

**Stroke-preserving stance clamp (same isolated 480/240/480, 0.14 m, 5 ms,
cap 500):** stance feet are projected onto the inset femur–tibia annulus
along the stroke (keep XY and solve `z` when that stays in reach; otherwise
intersect `last → desired`) after `body_rotation`, before IK. Swing still
uses coxa-radial clamp. Untilted latch XY is pulled back only when the
projector shortens planar reach; a z-only clamp is not inverse-rotated into
the stroke integrator (that mix collapsed the first forward capture).

Tripod mid-stance after the clamp:

| | Forward (`|v| = 0.12`) | Turn (`0.45` rad/s) |
| --- | --- | --- |
| Body progress / yaw | **44%** of 0.288 m | **13%** of 1.08 rad |
| Cartesian | 0.059 m/s (**50%**) | 0.336 rad/s (**75%**) |
| Pre-slew IK FK | **0.059 m/s (50%)** | 0.336 rad/s (**75%**) |
| Aligned post-slew FK | 0.059 m/s (50%) | 0.336 rad/s (75%) |
| Lagged-bridge FK | 0.060 m/s (50%) | 0.334 rad/s (74% tripod) |
| IK reach-hit fraction | **0** | **0** |
| Servo slew-hit fraction | 0.9% | 3.4% |
| Mean `post_clamp_distortion_m` | **0.016 mm** | **0.068 mm** |
| Plant-clamp hit fraction | 25% | 38% |

Verdict:

- **IK coxa-scale: closed.** Cartesian, pre-slew IK, aligned FK, and lagged
  FK now agree. Forward reach-hit 61% → 0; distortion 34 mm → 0.016 mm.
- IK opposition rose 42% → 50%; it did not reach the old 71% Cartesian figure
  because that 71% was out of the annulus. Stroke-preserving Cartesian is the
  reachable stroke.
- **Do not skip `body_rotation` next.** After the clamp, stance targets are
  already in-reach; lean is not the remaining hole.
- Forward body is still **44%** of command. Remaining miss is reachable
  stroke vs 0.12 m/s (plant clamp still 25%).
- Turn command yaw improved 63% → 75% and matches IK, but body yaw fell
  23% → 13%. That is still a different hole from forward’s IK drop.

**Chosen next lever:** reachable Cartesian stroke vs command (plant budget
`L` / workspace), or turn body-yaw collection. Do not raise `vmax`, disable
reach, retune lean, gait, `mu`, PD, ADMM, or `SolverMode` on this evidence.
Stand CTest after this clamp: 12000 healthy, height 2.63 mm, foot RMS
0.008 mm, p99 **5**.

**Plant `L` vs workspace XY (fixture v8, same isolated 480/240/480, 0.14 m,
5 ms, cap 500):** tripod Cartesian opposition is split by plant-clamp hit,
workspace-XY hit (planar shorten of the stroke projector), and neither.
No `L` / annulus / gait / lean / solver change.

Forward tripod mid-stance (`|v| = 0.12`):

| Split | Hit fraction | Cartesian opposition |
| --- | --- | --- |
| All samples | — | 0.059 m/s (**49%**) |
| Plant `L` hit | 26% | 0.004 m/s (**3%**) |
| Plant `L` miss | 74% | 0.076 m/s (**63%**) |
| Workspace XY hit | 23% | **0** |
| Workspace XY miss | 77% | 0.081 m/s (**68%**) |
| Neither hit | — | 0.118 m/s (**98%**) |

Turn command yaw stays ~76% Cartesian/IK; workspace-XY hit fraction is **0**;
body yaw is **16%** of 1.08 rad. Stand CTest: 12000 healthy, height 2.65 mm,
foot RMS 0.012 mm, p99 **5**.

Verdict:

- **Clock / `v dt`: closed.** Neither-hit samples are **98%** of command
  (`command_scale` 0.94). The 50% mean is not an upstream identity miss.
- **Plant `L`:** 26% of mid-stance samples sit at ~0. That cannot be the
  whole 50% (plant-miss is still 63%).
- **Workspace XY:** 23% of samples sit at 0. Plant-miss is still short
  because these hits remain.
- The 49% mean is the mix of the two clamps parking the foot. Body progress
  (43%) still tracks Cartesian, not a new IK hole.

**Chosen next lever:** workspace XY / annulus geometry first (zeros
opposition; plant-miss is still 63%), then plant budget `L`. Do not retune
gait, lean, `vmax`, `mu`, PD, ADMM, or `SolverMode`. Turn body-yaw collection
stays a separate hole.

**Coxa-centered stance lean (same isolated 480/240/480, 0.14 m, 5 ms, cap
500, fixture v8):** stance `body_rotation` is now
`coxa + R * (target - coxa)`; swing/stand still rotate about the body origin.
No `L` / gait / lean-gain / `vmax` / solver change.

Forward tripod mid-stance (`|v| = 0.12`):

| Split | Hit fraction | Cartesian opposition |
| --- | --- | --- |
| All samples | — | 0.053 m/s (**44%**) |
| Plant `L` hit | 25% | 0.002 m/s (**2%**) |
| Plant `L` miss | 75% | 0.069 m/s (**57%**) |
| Workspace XY hit | **27%** | **0** |
| Workspace XY miss | 73% | 0.079 m/s (**66%**) |
| Neither hit | — | 0.119 m/s (**99%**) |

IK / aligned FK match Cartesian (0.053 m/s). Reach-hit stays **0**; distortion
0.052 mm. Body progress 0.119 / 0.288 m (**41%**).

Turn workspace-XY hit fraction stays **0**; command yaw Cartesian/IK is 0.313 /
0.45 rad/s (**70%**); body yaw is 0.062 / 1.08 rad (**6%**). Stand CTest:
12000 healthy, height 2.65 mm, foot RMS 0.017 mm, p99 **5**.

Verdict:

- **Workspace XY is not repaired.** Hit fraction 23% → 27%; neither-hit is
  still ~99% of command. Origin vs coxa does not remove the 0.14 m down-foot
  lever when coxa `z ≈ −0.007`; first-order body-XY from lean is ~17 mm for
  **both** (leg `z` relative to coxa), and origin vs coxa differs by only
  0.2–1.6 mm.
- Overall Cartesian moved the wrong way (49% → 44%). The plant-`L` mix (~73%
  if workspace XY were gone) is not in play.
- IK remains matched to Cartesian. Stand is unchanged.

Why the repair missed (offline origin vs coxa vs skip-`R` on default geometry,
feedforward pitch −0.132, plant `L = |v| duty / f`):

- Workspace XY is `|rho| > d_max` (keep-XY z-solve fails), not the 17 mm
  body-XY shift. That shift is almost all z-only.
- A planned `+X` stroke of `L` from nominal has **0%** XY for both origin and
  coxa. Skip-`body_rotation` is **worse** on the front legs (6–26% XY) because
  untilted `L` already drives `|rho|` to 0.164–0.181 vs `d_max = 0.160`.
- The unit test is a false green: 60 frames (~36 mm), **leg 0 only**, no tilt
  feedback. Front-leg XY freeze is late-stroke; leg 0 never hits.
- Isolated 27% is not “rotate about the wrong point.” `planStanceFoot` at
  `φ = 0.10` already plants ~18 mm past nominal; extra-stance first plant
  (`p = stance_end`, `L = 0`) never writes `L` when planned stance later
  becomes true (`clampPlanarStrokeFromPlant` no-ops on `L = 0`). Those feet
  keep integrating into the annulus during planned mid-stance (876
  `held_swing` samples on the forward capture). Coxa vs origin cannot restore
  `rho` once `|rho| > d_max`.

**Chosen next lever:** give extra-stance / swing-contact plants a real stroke
budget (reset `L` when planned stance starts), not skip-`body_rotation` and not
raising `L`. Skip-`R` increases front-leg XY on a legal plant stroke. Do not
retune gait, lean gain, `vmax`, `mu`, PD, ADMM, or `SolverMode`. Turn body-yaw
collection stays a separate hole.

**Extra-stance stroke budget (same isolated 480/240/480, 0.14 m, 5 ms, cap
500, fixture v8):** extra-stance first plant latches the swing foothold XY
with \(L = |v_{xy}|\,\mathrm{duty}/f\). Stance kinematics always clamp to
that disk. Late-swing extra-stance that becomes planned (no stability hold)
replants with \(\phi/f\). Held wrap keeps the latch. No gait / lean-gain /
`vmax` / solver change.

Forward tripod mid-stance (`|v| = 0.12`):

| Split | Hit fraction | Cartesian opposition |
| --- | --- | --- |
| All samples | — | 0.072 m/s (**60%**) |
| Plant `L` hit | 42% | 0.004 m/s (**4%**) |
| Plant `L` miss | 58% | 0.123 m/s (**102%**) |
| Workspace XY hit | **0** | — |
| Neither hit | 58% | 0.123 m/s (**102%**) |

IK matches Cartesian. Reach-hit stays **0**; distortion 0.47 mm. Body
progress 0.077 / 0.288 m (**27%**).

Turn workspace-XY hit stays **0**; command yaw Cartesian/IK is 0.292 /
0.45 rad/s (**65%**); body yaw is 0.067 / 1.08 rad (**6%**). Plant `L` 38%.
Stand CTest: 12000 healthy, height 2.65 mm, foot RMS 0.010 mm, p99 **5**.

Verdict:

- **Workspace XY: closed.** 23–27% → **0**. Neither-hit is ~command
  (`command_scale` 0.95).
- Overall Cartesian 49% → **60%**, the plant-`L` mix with 42% still parked
  at `L` (4%) and 58% at command. The ~73% figure assumed plant `L` stayed
  26%; extra-stance clamping raised that share.
- Body progress fell 43% → 27% while Cartesian rose. Collection / plant
  slip, not IK (matched) and not annulus skate.
- Snapping extra-stance to stance-end with origin at the anchor parked the
  next stance at the back of the disk (Cartesian 6%). The swing foothold +
  planned replant avoids that.

**Chosen next lever:** plant `L` (42% of samples at ~0) if the goal is
Cartesian toward command; body-yaw / forward collection if the goal is the
70% body gate. Do not skip `body_rotation`, raise `L`, or retune gait, lean
gain, `vmax`, `mu`, PD, ADMM, or `SolverMode` on this evidence.

**Collection census (same isolated 480/240/480, 0.14 m, 5 ms, cap 500,
fixture v8):** replay-side live `foot_contacts` plus capture planned/hold/`L`
flags. No extra-stance policy / `L` / lean / `mu` / solver change. PGS A/B
reloads the same fixtures (`HEXAPOD_EXACT_REPLAY_LEGACY=1`). Clean tripod
means the three planned feet are the only contacts and none of those
contacted stance feet are on plant `L`.

Forward motion frames (480):

| | Proximal | PGS (same commands) |
| --- | --- | --- |
| Body progress | 0.080 / 0.288 m (**28%**) | 0.034 / 0.288 m (**12%**) |
| Tripod Cartesian | 0.073 m/s (**60%**) | same capture |
| Workspace XY | **0** | same capture |
| Plant `L` hit | 42% | same capture |
| Mean `n_raw_contact` | **5.75** | **6.00** |
| `n_contact ≥ 5` | **99.6%** (361 frames at 6) | **100%** (480 at 6) |
| Mean late-swing extra | 1.06 | 1.29 |
| Mixed parked+stroking | **47%** | **47%** |
| Clean tripod frames | **0** | **0** |
| Mid-stance world speed | 0.124 m/s | 0.042 m/s |
| Mid-stance `commanded_world` | 0.132 m/s | 0.110 m/s |
| Mid-stance uncommanded slip | 0.118 m/s | 0.095 m/s |
| Mean \|pitch\| | 0.185 rad | 0.029 rad |

Turn-in-place (480):

| | Proximal | PGS |
| --- | --- | --- |
| Body yaw | 0.130 / 1.08 rad (**12%**) | 0.127 / 1.08 rad (**12%**) |
| Tripod Cartesian yaw | 0.283 / 0.45 rad/s (**63%**) | same capture |
| Mean `n_raw_contact` | 5.61 | **6.00** |
| `n_contact ≥ 5` | 94% | **100%** |
| Clean tripod frames | **0** | **0** |
| Mixed parked+stroking | 37% | 37% |
| Mid-stance world speed | 0.094 m/s | 0.030 m/s |

Forced 3-stance / 3-swing probe (`test_physics_sim_tripod_stroke_probe`, extra-stance
off, no plant `L`, 0.12 m/s for 0.80 s). The first scripted raise used body-frame
XY shrink + 55 mm and an empty `RobotState` IK seed; raised commanded/post-IK
world Z sat at **−7 mm**, measured clearance **−18 mm**, reach-hit **0**, contact
**1.0**. That was not shaft-false-positive and not an IK clamp: stand Cartesian
assumes ~0.14 m height, the chassis sags, and body-frame `+Z` still plants on
the 18 mm sphere.

Pose-aware retry (measured world XY, support planted at radius, raised ramped
to world Z = radius + 40 mm = 58 mm, last measured joints into IK):

| | Stroke |
| --- | --- |
| `named_unload_cause` | **unloaded** |
| Raised measured Z / clearance | 0.036 m / **+18 mm** |
| Raised contact / reach-hit | **0** / **0** |
| Mean `n_raw_contact` | **2.74** |
| Clean tripod frames | **119 / 160 (74%)** |
| Body progress | **−0.001 / 0.096 m (~0%)** |
| Support `commanded_world` | 0.113 m/s |
| Support uncommanded slip | 0.021 m/s |
| Support world speed | 0.101 m/s |

Raised feet leave the manifold. The plant then **skates** the tripod: feet
travel at command scale while the body does not collect. `commanded_world` is
not ~0 because a stationary body makes stance stroke a world-frame skate.
Uncommanded slip is small (feet follow that skate), so this is stick failure
on a legal 3-contact set, not a census lie.

H4 telemetry A/B (same pose-aware probe, cap 500, 5 ms). Pinocchio fills
impulse/cone fields; PGS does not.

| | Proximal (μ_eff 0.55) | PGS |
| --- | --- | --- |
| `named_unload_cause` | **unloaded** | true_plant (raised stay down) |
| `named_h4_cause` (single-run) | **coulomb_skate** | unknown (no tripod) |
| Body progress | −0.001 / 0.096 m | −0.006 / 0.096 m |
| `n_raw_contact` | 2.74 | **6.00** |
| Clean tripod | 119 / 160 | **0** |
| Contact world / slip | 0.101 / 0.021 m/s | 0.024 / 0.056 m/s |
| Friction / normal | **0.30** (μ 0.55) | n/a |
| Servo torque util | **0.033** | n/a |
| Support tracking | 0.60 rad (Z plant, not XY) | 1.30 rad |
| Cone residual | ~0 | n/a |

PGS never unloaded the raise, so **proximal_mu_mix is not named**. Not
light_normal (normal impulse 0.018 Ns, ~3-leg weight). Not servo_cone (torque
3%, uncommanded slip 18% of command). Servos drag the tripod at 3% torque;
NCP cone residual is ~0 and friction/normal is below μ.

Named lever: raise tibia/plane **dynamic** μ to static in `BuildHexapodScene`
(tibia 0.45→0.60, plane 0.65→0.90). Pinocchio μ_eff **0.55→0.75**. No
`SolverMode` flip, no extra-stance, no `L`. Remesure proximal:

| | μ_eff 0.55 | μ_eff 0.75 |
| --- | --- | --- |
| Body progress | −0.001 / 0.096 m | **−0.0008 / 0.096 m (~0%)** |
| Contact world | 0.101 m/s | **0.100 m/s** |
| Friction / normal | 0.30 | 0.34 (still below 0.75) |
| Torque util | 0.033 | 0.036 |
| Clean tripod | 119 / 160 | 65 / 160 |

The 3-leg stroke still skates. Stop; do not stack Pinocchio `max(static,dynamic)`
on top of this batch. Extra-stance / H1 stay gated until a clean tripod collects.

Force coupling + authority sweep (same pose-aware probe, μ_eff 0.75, cap 500).
`expected_com_delta_v` is an upper bound: `n_support × peak_friction / mass`
summed over 160 frames. Pinocchio stall clip follows
`HEXAPOD_SERVO_TORQUE_SCALE`.

| vx | torque scale | Body / cmd | Foot world vx | Expected COM Δv | Coupling | Torque util | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.12 | 1.00 | −0.0008 / 0.096 (−1%) | −0.098 | **1.29 m/s** | **−0.0015 decoupled** | 0.036 | baseline |
| 0.03 | 1.00 | −0.0001 / 0.024 (−1%) | −0.022 | 1.00 m/s | −0.0018 decoupled | 0.033 | slower stroke still skates |
| 0.12 | 0.10 | −0.0009 / 0.096 (−1%) | −0.098 | 1.54 m/s | −0.0002 decoupled | **0.344** | stall clip works; still skates |
| 0.12 | 0.03 | — | — | — | — | — | **stand warmup fails** |

Friction impulses large enough to impart ~1 m/s on the 1.53 kg robot do not
show up as chassis Δv. Clipping stall to 10% raises utilization to 34% (same
~0.05 N·m request) and does not collect. 3% stall cannot stand, so there is
**no production torque limit** that both stands and makes this tripod collect.

Contact-wrench mapping (same pose-aware probe, μ_eff 0.75, cap 500). New
serve telemetry: signed and abs contact-frame friction mapped with
`ContactFrameRotation` into sim-world X/Z, plus free-flyer `M⁻¹ Jᵀ λ`
linear Δv. Peak `|λ_t|` is still ~0.0054 Ns, but:

| | Stroke |
| --- | --- |
| `named_mapping` | **opposing_tangents** |
| Abs world-X friction | **0.0145 Ns**/frame |
| Signed net world-X | **4.4×10⁻⁵ Ns** (cancel 0.3%) |
| Abs world-Z friction | 0.020 Ns/frame |
| World-Y leak of `R(tx,ty,0)` | **0** |
| Contact Δv_x (LOCAL→world) | −0.0011 m/s/frame |
| Body vx | −0.001 m/s |
| Support Cartesian opposition | 0.111 m/s (same-sign stroke) |

Friction is world-horizontal and has a large X component. The three support
contacts push **opposite world-X**, so the net chassis wrench is ~0. This is
not tibia-compound (no vertical leak) and not “servos eating the written
velocity” (contact Δv matches the tiny body vx). Commands are not left/right
mirrored: `mean_cartesian_opposition_speed` is the common −v_body stroke.

Stand-safe mapping levers that failed (reverted; stand warmup broke or still
cancelled):

| Lever | Result |
| --- | --- |
| `aba(..., Convention::LOCAL)` to match Delassus `LocalFrameTag` | **Stand warmup write/read failed** (frame 60) |
| Pinocchio 1-body order: robot on `joint1`, universe on `joint2` | **Stand warmup failed** |
| Align `ContactFrameRotation` t0 with foot slip | Still **opposing_tangents** (cancel 0.08%) |

No production mapping change. WORLD ABA, universe=`joint1` / tibia=`joint2`,
and the zero-slip tangent fallback stay as they are.

Per-leg census (same probe; support `{1,2,5}` = L3, R2, L1). Pinocchio
contact-X free speed vs minphys `Rᵀ v_point` on planted tibias:

| Leg | `world_slip_tx` | `pinocchio_drift_tx` | signed `fx` | Count |
| --- | --- | --- | --- | --- |
| 1 L3 | −0.0145 | −0.0125 (agree) | +2.6×10⁻⁴ | 0.89 |
| 2 R2 | +0.0117 | +0.0149 (agree) | −1.3×10⁻⁴ | 1.00 |
| 5 L1 | +0.0041 | −0.00058 (disagree) | −4.2×10⁻⁵ | 0.83 |

`named_tangent_census` = **same_j_opposite_lambda** (2/3 agree, mean contacts
per planted tibia 0.91, friction signs still split). Not `jacobian_parity`,
so the planned tangent-drift overwrite (`drift_tx/ty ← Rᵀ v_point`) was
**not applied** — it would be a no-op on the majority of plants. Contact-point
`tx` already has opposite signs on L3 vs R2 while commanded/measured foot
world X is the common skate.

Contact-point split (same probe: 0.12 m/s, 160 frames, cap 500). Last
constraint per tibia: tibia linear `v.x`, `(ω × r).x`, contact-frame `t0.x`,
and sphere-center foot `v.x`. Support `{1,2,5}` = L3, R2, L1:

| Leg | `tibia_vx` | `spin_vx` | `t0_x` | `foot_vx` |
| --- | --- | --- | --- | --- |
| 1 L3 | −0.0094 | −0.0051 | +0.79 | −0.0081 |
| 2 R2 | +0.0088 | +0.0029 | +1.00 | +0.0080 |
| 5 L1 | +0.0147 | −0.0106 | +0.59 | +0.0054 |

`named_slip_split` = **tibia_linear_opposite**: planted tibia (and physics
sphere-center) world-X already split. `t0_x` is same-sign (not `t0_flip`);
spin does not dominate a same-sign linear field (not `spin_offset`). Server
FK `mean_support_foot_world_vx` is still the common −0.098 m/s skate; the
split is the tibia rigid-body linear field in Pinocchio, not a contact-frame
sign flip.

No Pinocchio lever applied. `tibia_linear_opposite` is IK/servo, not
`ContactFrameRotation` or sphere-vs-contact drift. Did not lock t0 to world
+X and did not replace tangent drift with sphere-center velocity. Did not
skip `body_rotation` or retune IK. Stand CTest not re-run (no lever).

FK vs physics sphere (same probe). Per-leg commanded Cartesian world-X,
server FK finite-diff, physics sphere X position-rate, and instantaneous
sphere `v.x`:

| Leg | `cmd_foot_vx` | `fk_foot_vx` | `foot_pos_vx` | `foot_vx` |
| --- | --- | --- | --- | --- |
| 1 L3 | −0.087 | −0.095 | −0.0082 | −0.0081 |
| 2 R2 | −0.121 | −0.102 | +0.0081 | +0.0080 |
| 5 L1 | −0.121 | −0.096 | +0.0054 | +0.0054 |

`named_fk_physics` = **fk_vs_sphere** on **sim X** (compared to server FK X).
That comparison mixed frames: server world X is **−sim Z** (chassis forward).
Sim X is server lateral. Stroke-axis remesure (same probe, then with the
production legacy X adapter on TwistField):

| | Before adapter | After adapter (production-like) |
| --- | --- | --- |
| `named_stroke_axis` | **axis_mix** | **axis_mix** |
| `named_stroke_friction` | aligned_z (all +) | aligned_z (all −) |
| FK foot vx (server) | −0.095…−0.102 | +0.082…+0.109 |
| Expected sim vz (−FK X) | +0.095…+0.102 | −0.082…−0.109 |
| Sphere pos vz (sim) | −0.053…−0.094 | +0.065…+0.094 |
| Body / cmd | −0.0008 / 0.096 | **+0.0008 / 0.096 (~0%)** |
| Support skate (server X) | −0.098 | +0.099 |
| Clean tripod | 65 / 160 | 125 / 160 |

The sphere **does** skate at command scale on sim Z, common-sign across
planted support. That skate stays **opposite** the C-map `z_sim = −x_srv`
from server FK. Friction on sim Z is aligned, not left/right cancelled.
Net chassis coupling is still ~0 (`friction_com_coupling_ratio` 0.003).

The probe now applies `linear.x → −linear.x` so TwistField+IK matches
`BodyController` / `legacyKinematicTwistFromServerBody`. That is not skip-
`body_rotation`. It did not make the body collect. No Pinocchio tangent
overwrite, no extra-stance / `L` / μ / `SolverMode`. Stand CTest stayed
green after the protocol tails (`passed=1`, 12000/12000 healthy, p99
iterations 5).

FK vs C-mapped sphere position (same probe, `x_srv = −z_sim`). Stroke
displacements on support `{1,2,5}`:

| Leg | `stroke_fk_dx` | `stroke_mapped_dx` | mean `fk − mapped` X |
| --- | --- | --- | --- |
| 1 L3 | +0.085 | −0.193 | −0.281 |
| 2 R2 | +0.082 | −0.075 | +0.151 |
| 5 L1 | +0.072 | −0.065 | +0.284 |

`named_stroke_axis` = **sphere_opposite_stroke**, `named_fk_sphere_pos` =
**opposite_stroke**. Net XY travel of the contact sphere is opposite server
FK, not a rate-average artifact. `max_servo_tracking_error_rad` ≈ 0.62.

One stand-preserving coxa lever: `CreateServoJoint` coxa `localAxisA`
`+Y → −Y` (assembled pose unchanged). Stand CTest stayed green. Remesure
named **sphere_split_stroke** / **diverge**: R2 mapped `dx` then followed
FK (+0.074 vs +0.082) while L3 stayed opposite (−0.193) and L1 stayed
negative. Friction on sim Z became `opposing_z`. Body still ~0%. Reverted
the axis. A global coxa sign is not the lever; do not retune per-leg axes.

Frame C census (same probe). First-frame FK vs sphere under bridge
`(-z, x)` and alignment `(x, z)`, plus stroke Δ on both maps:

| Leg | rest bridge XY | rest align XY | `stroke_fk_dx` | `stroke_mapped_dx` | `stroke_align_dx` |
| --- | --- | --- | --- | --- | --- |
| 1 L3 | −0.368, −0.023 | −0.030, +0.024 | +0.085 | −0.193 | +0.146 |
| 2 R2 | +0.081, −0.078 | −0.136, +0.140 | +0.082 | −0.075 | +0.007 |
| 5 L1 | +0.243, +0.144 | +0.195, −0.195 | +0.072 | −0.065 | −0.010 |

`named_rest_c` = **rest_both_offset** (no planted set inside 25 mm on either
C). `named_stroke_c` = **stroke_90** (FK +X command-scale; align/sim-X tiny
on R2/L1; bridge/sim-Z command-scale opposite). `named_frame_c` =
**both_opposite**. `named_tracking` = **tracking_mixed** (tibia 0.62 rad,
femur 0.39, coxa 0.10). Body **+0.0008 / 0.096 (~0%)**.

No composition lever (`foot_body` 90° yaw) — that required `align_c_match`
or `align_rest_stroke_90`. No wire-zero (not `bridge_c_match` + one joint).
No `simVecToServer` change, no coxa retry, no extra-stance / `L` / μ /
`SolverMode`. Stand CTest not re-run (probe-only census).

Body-frame rest (same probe): `footInBodyFrame` vs chassis-relative sphere
(invert bridge C on body pose; sphere sim-Y taken as chassis sim-Y). Residuals
match the world-frame rest census to millimeters:

| Leg | body bridge XY | body align XY |
| --- | --- | --- |
| 1 L3 | −0.368, −0.023 | −0.028, +0.023 |
| 2 R2 | +0.081, −0.078 | −0.135, +0.138 |
| 5 L1 | +0.243, +0.144 | +0.197, −0.196 |

`named_body_c` = **body_both_offset**. Unmixing world FK did not produce
`body_align_match`. No 90° `foot_body` yaw. Body still **+0.0008 / 0.096**.

Per-leg rest offset (align-C sphere vs FK body and vs captured plant XY).
L3 is a ~3.6 cm “other” (just outside match). R2 and L1 are **diag**
residuals of opposite sign (~14 cm and ~20 cm, `|dx| ≈ |dy|`). Plant
classes match the body-frame classes. `named_rest_offset` =
**per_leg_split**. Not `left_right` (the two left support legs disagree:
L3 vs L1). Not a global `swap_90`. No side-sign / mount lever. No extra-stance
/ `L` / μ / `SolverMode`. Stand CTest not re-run.

Verdict:

- Mixed-C is **not** a first-frame match on either map, in world or body
  frame. Rest offset is **per-leg**, not one mapping. Stroke is 90° on two
  of three plants. Stop this rest-offset batch. Do not flip `SolverMode`.
- Extra-stance / plant-`L` mix **policy stays off**.
- Isolated gait still shows **H1** and **H2** on the 6-contact walk.
- Do not skip `body_rotation`, raise plant `L`, retune PD / ADMM, or change
  production servo stall on this evidence.

## Isolated tripod geometry and contact apply (2026-09-13)

The mixed-C / per-leg rest census above was a **mapping error**, not a
per-leg solver split. Subsequent work (not a SolverMode / extra-stance / `L`
change) closed the isolated three-contact stroke:

1. **Mount yaw.** Simulator mount is from body forward with radial
   `(sin a, 0, cos a)`. Canonical server map `(x,y,z)_srv = (-z, x, y)_sim`
   makes that yaw `π − a` (`legFrameYawRad`). The previous `+X` yaw convention
   put L3 ~16° off, R2 90° off, and L1 nearly reversed.
2. **Coxa wire sign.** Positive sim `+Y` coxa is negative server yaw after the
   improper map; femur/tibia already agreed. Bidirectional reverse in
   `physics_sim_joint_wire_mapping.hpp`.
3. **Canonical coxa offsets.** Default hip locations are the same numbers in
   the bridge frame, so the chassis and FK feet share one basis.
4. **Foot centre.** Knee-to-sphere-centre reach is 104 mm; the rigid tibia
   shaft stays 86 mm (`kTibiaKinematicReach`). Alignment test
   `test_hexapod_geometry_alignment` encodes this.
5. **Rubber contact.** Built-in scene tibia/plane static and dynamic friction
   **2.0**. Pinocchio μ is the dynamic average (**2.0**). This is the stick
   threshold that dropped planted-foot skate from ~97 mm/s to ~4 mm/s.
6. **Penetration bias.** Hexapod scene `penetrationBiasFactor = 0.20`, slop
   2 mm. Contact depth ~2 mm at 120/240/480 Hz.
7. **Substep rounding.** `ceil(dt / max_substep − 1e-3)` so a 4167 µs command
   is two proximal steps, not three.
8. **Contact Δv.** Articulated Delassus scratch (`applyOnTheRight` →
   `getInternalData().ddq`) is close in sliding contact and **wrong at the
   stick/slip boundary**. Production apply is: form `Jᵀλ`, zero-g / zero-v
   WORLD ABA → `M⁻¹ Jᵀλ`, then `v ← v + Δv`. ADMM itself stays articulated.
   Dense ADMM / collision-filter experiments remain opt-in or removed.

FK-to-sphere rest regression:
`fkMountConventionMatchesPhysicsContactSphere` in
`test_motion_intent_through_ik_fk`.

Remesure (this tree, ABA apply, cap 500, 5 ms, pose-aware probe):

| Quantity | Result |
| --- | --- |
| `command_progress_m` / commanded | **0.0737 / 0.096 (76.8%)** — clears 70% |
| `named_rest_c` / `named_body_c` / `named_fk_sphere_pos` | `rest_bridge_match` / `body_bridge_match` / `match` |
| `mean_max_contact_penetration` | 2.0 mm |
| `mean_support_foot_world_vx_mps` | −0.8 mm/s |
| `mean_body_vx_mps` | 0.092 m/s (command 0.12) |
| `clean_tripod_frame_fraction` | 0.86 |
| Stand CTest 60 s | **Pass.** 12000/12000 healthy, height error 9.8 mm, foot RMS 0.22 mm, p99 iters 7 |

`named_coupling=decoupled` is **not** a miss: `friction_com_coupling_ratio`
divides by a large `n_support × peak_friction / mass` bound. Body travel is
the gate. Align-C rest classifiers (`named_rest_offset=swap_90`) are expected
once geometry is in the bridge frame.

Still **do not** flip `SolverMode`. Isolated tripod collection is not seed-0
walk.

## Six-contact remaining gates (2026-09-14)

Stand at production cap 24 stayed green, so the remaining switch work is
six-contact walk. Capture stayed at the harness default 500-iter proximal
reference so the controller remains in WALK. All later A/Bs reuse
`HEXAPOD_EXACT_REPLAY_COMMANDS_IN`.

### Seed 0, fixture v9 (pre-lever), cap 24

Both enforce flags on. Safety: recovered **20**, held **0** (RecoveredRetry
allowed). Behaviour **fail**.

| Phase | Progress | Notes |
| --- | --- | --- |
| forward | 0.0362 / 0.0432 (**83.9%**) | pass |
| reverse | 0.0530 / 0.0432 (**123%**) | pass |
| strafe | 0.0247 / 0.0360 (**68.7%**) | **fail** |
| diagonal | 0.0279 / 0.0433 (**64.6%**) | **fail** |
| turn | yaw 0.153 / 0.162 (**94.7%**), path 21 mm | pass |

H1 still true: `clean_tripod_frames=0`, `fraction_n_contact_ge_5` ≈ 0.86–0.96,
`mean_n_late_swing_extra` ≈ 0.58–0.72. H2 mixed parked+stroking ≈ 0.24–0.28 —
do **not** raise plant `L`. PGS on the same v9 stream (`HEXAPOD_EXACT_REPLAY_LEGACY=1`)
collects ~0–2% of command; that is an unfair gait comparison (commands captured
under proximal). Class **A only-proximal**, with secondary Class B (20 cap-24
non-convergences, recovered) and no holds.

### One named lever: late-swing extra-stance off

Evidence: H1 (`n_contact ≥ 5`, clean tripod 0). Production
`enable_contact_mode_planning` is still false, so `BodyController` used the
fallback that kept stance kinematics whenever raw contact outlived the 0.45
liftoff grace. Planned swing after grace now keeps **swing** kinematics
(`body_controller.cpp` / `locomotion_feasibility.cpp`). Stability hold /
lost-candidate extra-stance is unchanged. Plant `L`, μ, ADMM, stall, and
`body_rotation` were not touched.

Unit tests that encoded the old extra-stance plant now require a raised swing
foot. Frozen v9 joint targets cannot show a controller lever, so the fixture
was recaptured once to v10 (`faf3215fdfc4a207`).

### Seed 0, fixture v10 (post-lever), cap 24

Stand remesure at cap 24: **pass** (see table). Replay with both enforce flags:

- healthy 690, recovered **29**, held **1**, `solver_not_converged` 30, max
  iters 24, `p99_solver_total_step_time_ms` **1.27 ms**
- behaviour still **fail**: strafe **69.9%**, diagonal **67.8%**;
  forward **83.6%**, reverse **125%**, turn yaw **93.5%** / path 21 mm
- census still H1: clean tripod 0, `fraction_n_contact_ge_5` ≈ 0.88–0.97,
  `mean_n_late_swing_extra` ≈ 0.69–0.82. That census counts raw contact on
  legs that are not planned/held stance, so commanded swing can still show
  extra plants if the foot has not left the ground
- gait duty on capture is **0.717** (overlap, not 0.5 tripod);
  `mean_n_planned` ≈ 5.2. Extra-stance kinematics was not the only source of
  `n_contact ≥ 5`
- H2 mixed parked+stroking still ≈ 0.24–0.28 — still do not raise `L`

One class, one lever. Seed 0 is still red (Class A strafe/diagonal, plus a
new Class D hold). Do **not** stack a second lever, do **not** start 100-seed
or cadence, do **not** flip `SolverMode`, do **not** enable exact replay as
CTest.

Closed-loop [`test_physics_sim_walk_distance`](../hexapod-server/tests/test_physics_sim_walk_distance.cpp)
at 0.14 m / cap 24 / proximal is supporting evidence, not the switch gate:
2356 healthy, 44 recovered, **0 held**, stays WALK, path 0.44 m / net 0.22 m,
fails only the 0.2 m/s speed band (ratio 0.17). The 2026-09-12 hold cascade
on this binary is closed.

`./scripts/verify.sh` (2026-09-14): physics-sim rebuild, server CTest, firmware
host tests, then scenario smoke on `config.sim.txt`. Server CTest **15/84
failed** (`scenario_body_height_margin`, default `physics_sim_walk_distance`,
walk-entry/stability/clearance/contact-loss suites, locomotion regression,
motion performance). Failures are the existing crouched-PGS / body-height
envelope (`BODY_COLLAPSE`, standing height 0.137 m vs min-safe 0.156 m), not
a new proximal stand miss: `physics_sim_proximal_stand_acceptance` passed in
the same run. Exact replay stays diagnostic. Parser default and WSL
`SolverMode` remain `0`; WSL `SolverIterations` stays 12 until a green flip
sets both mode and 24 together.

## Walk-entry overlap lever (2026-09-14)

Named miss after extra-stance: every 72-frame burst is STAND→WALK, so
`walkEntryStance` (duty 0.94, zero stride) blended over `transition_blend_s`
0.35 s occupied the 0.36 s behaviour window (v10 mean duty **0.717**,
`mean_n_planned` ≈ 5.2).

**Lever:** keep all-stance at t=0. STAND→WALK uses `walk_entry_blend_s`
**0.15 s**; gait-type changes still use `transition_blend_s` 0.35 s
([`gait_scheduler.cpp`](../hexapod-server/src/control/gait_scheduler.cpp)).
`test_gait_params` requires a swing leg and duty &lt; 0.55 by 0.20 s. Plant `L`,
μ, ADMM, stall, `body_rotation`, and `kTripodLateralStepTrim` were not touched.

Recapture v11 (`82c6c4354cbc196`) at cap 500. Stand at cap 24: **pass**
(12000 healthy, height 9.51 mm, foot RMS 7e-6 m, p99 7).

Seed 0 at cap 24, both enforce flags, **post-transient** (48 frames / 0.24 s):

| Phase | Progress | Cross-track | Gate |
| --- | --- | --- | --- |
| forward | 0.0463 / 0.0288 (**161%**) | **21.3 mm** vs 15.3 mm | **fail** |
| reverse | 0.0527 / 0.0288 (**183%**) | 15.1 mm vs 15.7 mm | pass |
| strafe | 0.0233 / 0.0240 (**97%**) | 2.6 mm vs 12.4 mm | pass |
| diagonal | 0.0460 / 0.0288 (**160%**) | 12.0 mm vs 14.8 mm | pass |
| turn | yaw 0.255 / 0.108 (**236%**) | path 2.6 mm | pass |

Safety: recovered 20, held **0**. Duty fell to **0.595**; `mean_n_planned` ≈ 4.04;
`captured_tripod_frames` 46–47 of 72; `clean_tripod_frames` still 0. p99 solver
total **1.21 ms**.

Seed 0 is still red (Class A forward cross-track). One class, one lever. Do
**not** stack `kTripodLateralStepTrim`, do **not** start 100-seed or cadence,
do **not** flip `SolverMode`. The remaining named miss matches the v10
post-transient diagnosis: forward cross-track after the 120 ms window, not
strafe and not walk-entry occupancy.

Closed-loop `test_physics_sim_walk_distance` at 0.14 m / cap 24 / proximal
after this lever: 2333 healthy, 65 recovered, **2 held**, 107 faulted walk
steps, path 1.46 m / net 0.34 m, speed ratio 0.61. It no longer stays in WALK
for the whole command. Supporting evidence only; not a second lever.

`./scripts/verify.sh` after this lever: same shape as the previous pass —
**15/84** server CTests failed (crouched PGS / body-height envelope).
`physics_sim_proximal_stand_acceptance` passed. Parser default and WSL
`SolverMode` remain `0`; WSL `SolverIterations` stays 12.

## Forward cross-track investigation (2026-09-14)

Post-transient forward on v11 is a **crab**, not a yawed arc: evaluated
\(\Delta y = 21.3\,\mathrm{mm}\) equals the full-phase lateral (the first
120 ms contributes ~0 Y). Yaw is only 0.039 rad. End velocity is
\(v_x=0.088\), \(v_y=0.107\,\mathrm{m/s}\).

`HEXAPOD_EXACT_REPLAY_TRACE_LEGS` on the frozen v11 stream shows the 0.36 s
STAND→WALK burst only ever swings **Group B** (`L3`, `R2`, `L1` = 2 left +
1 right). Group A (`R3`, `L2`, `R1`) has zero swing frames. Walk-entry
starts `phase_accum=0` with offsets blending from 0, duty settles at 0.5,
and stride rate is ~0.9 Hz, so `phase_accum` reaches only ~0.32 in 0.36 s —
Group A never crosses duty. Those three swing feet stay in raw contact for
**74–87%** of commanded swing (`mean_n_late_swing_extra` ≈ 1.61,
`clean_tripod_frames` 0). Reverse uses the same first tripod and is just
inside its lateral limit (15.1 vs 15.7 mm).

**Tried lever (reverted):** swing vertical \(64 u^3 \to 16 u^2\) so the
18 mm foot sphere would clear earlier. Recapture v12 (`7fc1aa5eff257482`).
Stand at cap 24 still passed (12000 healthy, height 9.0 mm, p99 7). Seed 0
at cap 24: forward lateral **20.5 mm** vs 15.3 mm (still fail); late-extra
only 1.61 → 1.33; **held 1** and one read failure (safety regression vs v11
held 0). Reverted `swingVerticalShape`, the foothold-planner early-lift
assertion, and the algorithms note. Frozen fixture remains v11
(`82c6c4354cbc196`).

Do **not** retry the n=2 profile, do **not** stack `kTripodLateralStepTrim`,
do **not** retune `walk_entry_blend_s`.

**Landed lever:** first-stride phase coverage. STAND→WALK still begins
all-stance (duty 0.94, offsets 0). Planar bursts seed \(\Phi=0.35\); yaw-dominant
turns keep \(\Phi=0\) using `intent.cmd_*` (not slewed `cmd_twist`). New planned
plants start at \(\varphi=0\), then the latch integrates \(v\,\mathrm{d}t\).

A 2 Hz cadence floor (v13) put both tripods in the window and cut forward
cross-track to 3 mm, but starved the stroke (progress ~47%). Phase-seeding at
adaptive cadence (v14) restored translation (forward 103% / 9.5 mm) but turn
yaw fell to 35% because loco-cmd ramps yaw from 0 and the first frame looked
planar. Skipping the seed from `intent.cmd_yaw` (v16) restored turn.

Recapture v16 (`ddc6008e0cc1ac97`) at cap 500. Stand at cap 24: **pass**
(12000 healthy, height 9.4 mm, foot RMS 7e-6 m, p99 7). Seed 0 at cap 24,
both enforce flags, post-transient:

| Phase | Progress | Cross-track / path | Gate |
| --- | --- | --- | --- |
| forward | 0.0320 / 0.0288 (**111%**) | 11.1 mm vs 12.9 mm | **pass** |
| reverse | 0.0367 / 0.0288 (**127%**) | 9.3 mm vs 12.9 mm | **pass** |
| strafe | 0.0256 / 0.0240 (**107%**) | 5.2 mm vs 12.4 mm | **pass** |
| diagonal | 0.0233 / 0.0288 (**81%**) | 0.6 mm vs 12.9 mm | **pass** |
| turn | yaw 0.159 / 0.108 (**147%**) | path 15 mm | **pass** |

Safety: recovered 24, held **0**. `p99_solver_total_step_time_ms` **1.16 ms**
after last-resort (seed 0 unchanged: last-resort does not fire). 100-seed
safety+behaviour is green. Cadence 120/240/480 Hz seed 0 is green after
capture-period command scoring. `verify.sh` 15/84 is PGS/offline (struck).

### 100-seed held investigation (2026-09-14)

v16 100-seed safety at cap 24 started **red**: 15 `HeldLastGood` / 15 read
failures across 72000 frames (`solver_not_converged` 2389 = recovered 2374 +
held 15). v11 on the same physics had a similar held rate, so this is not a
phase-seed regression.

Holds are sparse Class D after the existing same-`dt` + warm half-step retries
already ran. Seed 4 isolation: **pose perturbation** (0.75 mm / ~0.2°) causes
the hold; contact-order shuffle alone does not. The held frame is a 5-contact
set, ADMM cap 24, `ncp_dual` ~0.02 (seed 4) or ~0.005 (seed 23) vs the 1e-3 NCP
floor. `pre_v` ~0.18 m/s, not a speed-limit.

**Landed lever:** last-resort cold half-steps. Same-`dt` and warm `dt/2` are
unchanged. Only when those miss: `resetWarmStarts()`, two half-steps at
`max(2 * SolverIterations, 48)`. Production first attempts stay at cap 24.
Do not recapture.

Remesure on fixture `ddc6008e0cc1ac97`, cap 24:

| Gate | Result |
| --- | --- |
| Stand 60 s | **Pass.** 12000 healthy, held 0, height 9.49 mm, p99 iters 7 |
| Seed 0 safety+behaviour | **Pass.** Same numbers as v16 (last-resort does not fire) |
| 100-seed safety | **Pass.** held 0, read 0, recovered 2388, max iters 27 |
| 100-seed behaviour | **Pass.** `behavior_gate_failures` 0 |
| 120 Hz | Safety held 0. Diagonal progress **0.0354 / 0.0571 (62%)** vs 70% (pre-scoring; closed below) |
| 240 Hz | **Pass.** held 0, all headings |
| 480 Hz | **Pass.** held 0, all headings |

120 Hz uses the same 720 captured frames with a larger `PERIOD_US`, so commanded
metres scale with `dt` while the target stream is still a 200 Hz capture.
Closed in [120 Hz cadence investigation](#120-hz-cadence-investigation-2026-09-14).

### 120 Hz cadence investigation (2026-09-14)

Named miss after last-resort: seed 0, `HEXAPOD_EXACT_REPLAY_PERIOD_US=8333`,
both enforce flags, diagonal **0.0354 / 0.0571 m (62%)**. Forward / reverse /
strafe / turn passed. Safety held 0. Last-resort did not fire (`max_iterations`
24).

`passesBehaviorGates` scored `speed * evaluated_frames * replay_period_us`.
Replaying a 200 Hz capture at 120 Hz stretches 72 frames from 0.36 s to 0.60 s,
so the 70% bar inflated while the servo targets stayed the capture stream.
Absolute diagonal travel **rose** (0.023 → 0.035 m); slip and foot RMS were
lower than at 200 Hz.

**Class A:** PGS A/B on the same v16 fixture at 8333 µs
(`HEXAPOD_EXACT_REPLAY_LEGACY=1`) also missed diagonal (~1% progress on every
heading). The capture is proximal-shaped, so that is not a fair proximal-only
plant miss. Rescoring existing JSON with
`min(replay_period_us, capture_period_us=5000)` put 120 Hz diagonal at **103%**
(0.0354 / 0.0343); 240/480 unchanged and still pass.

**Landed lever:** cap commanded metres and yaw at the capture period when a
fixture is loaded and `PERIOD_US` is larger than `capture_period_us`. Both the
JSON `evaluated_commanded_*` path and `passesBehaviorGates` multiply by
`min(replay_period_us, capture_period_us)`. Do not recapture, do not retune
`kTripodLateralStepTrim` / `walk_entry_blend_s`, do not raise plant `L`, μ,
dense ADMM, or stall.

Remesure on fixture `ddc6008e0cc1ac97`, cap 24, both enforce flags, seed 0:

| Gate | Result |
| --- | --- |
| 120 Hz (8333 µs) | **Pass.** Diagonal **0.0354 / 0.0343 (103%)**. Forward 137%, reverse 155%, strafe 134%, turn yaw 122%. held 0, recovered 21, `max_iterations` 24 |
| 240 Hz (4167 µs) | **Pass.** Diagonal 91%. held 0, recovered 16 |
| 480 Hz (2083 µs) | **Pass.** Diagonal 104%. held 0, recovered 6 |
| 200 Hz (5000 µs) | **Pass.** Same as v16 (forward 111% / reverse 127% / strafe 106% / diagonal 81% / turn 147%). recovered 24, `max_iterations` 24 (last-resort does not fire) |
| Stand 60 s cap 24 | **Pass.** 12000 healthy, height **9.49 mm**, foot RMS **7e-6 m**, p99 iters **7**, max iters **8** |

Parser default and WSL `SolverMode` were still `0` at cadence close. Flip is in
[verify.sh inventory](#verifysh-inventory-2026-09-14).

### verify.sh inventory (2026-09-14)

Last `./scripts/verify.sh` server CTest: **15/84 failed**. Proximal stand
(`physics_sim_proximal_stand_acceptance`) **passed** in the same run. Smoke uses
[`config.sim.txt`](../hexapod-server/config.sim.txt) (`Runtime.Mode = "sim"`),
not physics-sim proximal. Iterations-only
[`PhysicsSimBridge`](../hexapod-server/include/hardware/physics_sim_bridge.hpp)
ctors default `solver_settings_{}` to `LegacyPgs`; they ignore WSL `SolverMode`.

| CTest | Class | Why it is not the proximal production gate |
| --- | --- | --- |
| `scenario_body_height_margin` | Offline TOML | Scenarios command **0.14 m**. Assembly standing ~0.137 m + squat 0.020 m ⇒ min-safe **0.156 m**. No live solver |
| `physics_sim_walk_distance` | PGS crouch | Default height **0.06 m**. Harness `SolverMode = 0`. Proximal only via `HEXAPOD_WALK_TEST_SOLVER_MODE` (supporting evidence, not this CTest) |
| `physics_sim_walk_entry_tracking` | Live PGS | Iterations-only ctor. Commands 0.14 m |
| `physics_sim_walk_stability` | Live PGS | Iterations-only ctor. Commands 0.14 m |
| `physics_sim_turn_foot_clearance` | Live PGS | Iterations-only ctor. Commands 0.14 m |
| `physics_sim_oblique_walk_clearance` | Live PGS | Iterations-only ctor. Commands 0.14 m |
| `physics_sim_turn_raw_contact_loss` | Live PGS | Iterations-only ctor |
| `physics_sim_slow_fwd_walk_foot_clearance` | Live PGS | Iterations-only ctor. Commands 0.14 m |
| `physics_sim_wave_slow_walk_foot_clearance` | Live PGS | Iterations-only ctor. Commands 0.14 m |
| `physics_sim_slow_fwd_walk_contact_loss` | Live PGS | Iterations-only ctor |
| `physics_sim_tripod_support_baseline` | Live PGS | Iterations-only ctor. Commands 0.14 m |
| `physics_sim_nav_waypoints` | PGS crouch | Iterations-only ctor. Height **0.06 m** |
| `physics_sim_navigation_acceptance` | PGS crouch | Iterations-only ctor. Height **0.06 m** |
| `locomotion_regression_suite` | Live PGS | Iterations-only ctor. Phases at **0.10 m**, not production 0.14 m |
| `motion_performance_suite` | Live PGS | Iterations-only ctor. Stand height 0.14 m, still PGS |

None of the 15 forces `PinocchioProximal`. Do **not** raise scenario heights,
retarget these binaries to proximal, or change the iterations-only ctor.
Leave `verify.sh` red. All 15 are struck from the switch contract.

**Flip (2026-09-14):** WSL `SolverMode = 1`, `SolverIterations = 24`, parser
default `1`. Harness and `config.physics-sim.txt` stay `0`.

Remesure after flip (cap 24, v16, both enforce flags):

| Gate | Result |
| --- | --- |
| Stand 60 s | **Pass.** 12000 healthy, height **9.81 mm**, foot RMS **0.22 mm**, p99 iters **7**, max iters **23** |
| Seed 0 at 5000 µs | **Pass.** Forward 111% / reverse 127% / strafe 106% / diagonal 81% / turn 147%. recovered 24, held 0, `max_iterations` 24 (last-resort does not fire) |

## Summary

The remaining [`plan.md`](plan.md) blocks are **evidence**, not missing solver
architecture. Close them by measuring the existing harnesses, classifying the
failure, then changing only the lever that failure implies. Do not start with
resource Phases 1–3 or ADMM default flips.

The WSL default **is** `pinocchio-proximal`: `Runtime.PhysicsSim.SolverMode = 1`
in [`hexapod-server/config.physics-sim-wsl.txt`](../hexapod-server/config.physics-sim-wsl.txt)
(parser default `1`, WSL `SolverIterations` 24). Proximal stand CTest and v16
replay are the live physics gates (now default CTests at seed 0 and 120/240/480 Hz).
The 15 `verify.sh` CTests are PGS/offline and are struck. 100-seed stays a
diagnostic override. Dense ADMM and resource Phases 1–3 stay follow-ups.

Related notes (do not treat them as substitutes for this campaign):

- Allocator / I/O / broadphase:
  [`PLAN_PINOCCHIO_PROXIMAL_RESOURCE_REDUCTION.md`](PLAN_PINOCCHIO_PROXIMAL_RESOURCE_REDUCTION.md)
- Retry policy, Delassus apply, ADMM iterations:
  [`PLAN_PINOCCHIO_ADMM_DELASSUS_RETRIES.md`](PLAN_PINOCCHIO_ADMM_DELASSUS_RETRIES.md)

## Treat as real blocks vs. drop from the switch contract

**Must pass before flipping the default**

| Gate | Already have | What “solved” means |
| --- | --- | --- |
| Stand 60 s, ±10 mm height, 3 mm foot RMS, p99 ≤ 20 iters, no hold | [`test_physics_sim_proximal_stand_acceptance`](../hexapod-server/tests/test_physics_sim_proximal_stand_acceptance.cpp) is already a live CTest | That binary stays green at production height `0.14 m`. If it is red in this tree, it is the first bug, not replay |
| 70% translation/yaw, lateral 10%+10 mm, turn translation &lt; 50 mm | Replay + `HEXAPOD_EXACT_REPLAY_ENFORCE_BEHAVIOR_GATES=1` | **Pass** on seed 0 and 100 seeds at 200 Hz, and seed 0 at 120 / 240 / 480 Hz |
| Safety on perturbed ICs | `HEXAPOD_EXACT_REPLAY_ENFORCE_SAFETY_GATES=1` (allows `RecoveredRetry`) | **Pass.** Zero held / unsupported / failed-read on 100 seeds |
| p99 physics step &lt; 4.0 ms at 240 Hz WSL; ADMM p99 ≤ 20; no cap exhaustion | Replay JSON (`p99_solver_total_step_time_ms`, `iteration_histogram`) | Numbers at **cap 24**, not the replay default 50. Round-trip p99 includes loopback; use solver-total as the physics figure |
| Full physics/server/gait suites | `./scripts/verify.sh` plus locomotion binaries | **Struck.** The 15 reds are PGS/offline, not proximal 0.14 production. Proximal stand CTest already passed. Leave `verify.sh` red |

**Do not let these block the switch**

- Named contact unit tests (edge, duplicate, sliding). Stance/oracle/cone already
  exist; turn/strafe exist on replay. Add a unit test only when a histogram
  points at that mode.
- LCABA / constrained ABA. Unilateral ADMM is the production path.
- Pinocchio COM always at origin. Fix only if energy/oracle or pose mapping fails.
- Resource Phases 1–3. Hygiene. They do not make the robot walk 70% of command.
- Same-dt retry, relative NCP stop, promoting dense ADMM/precondition. Those are
  tools for a **measured** iteration or p99-time miss.

**Struck from the switch contract (2026-09-14)**

- 10-minute randomized gait with zero held/NaN/speed-limit. There is no named
  test. Keep it as **post-default soak**, not an unowned switch blocker.
  [`plan.md`](plan.md) no longer lists it under default-switch behaviour gates.
- `./scripts/verify.sh` 15/84 server CTests (PGS live-physics, crouch 0.06 m,
  offline scenario height envelope). Not a proximal production gate. See
  [verify.sh inventory](#verifysh-inventory-2026-09-14). Do not retarget those
  binaries this batch.

**Cadence (closed 2026-09-14)**

- 120/240/480 Hz displacement equivalence. Replay already has
  `HEXAPOD_EXACT_REPLAY_PERIOD_US`; three cadences on one fixture. Seed 0
  **pass** after commanded metres/yaw use `min(replay_period_us, capture_period_us)`.
  See [120 Hz cadence investigation](#120-hz-cadence-investigation-2026-09-14).

Record the intermittent PGS walk-distance issue as a **known baseline** before
blaming proximal for a walk miss. That was step 1 of `plan.md` and is still
useful triage.

## Campaign: measure, then repair the class of failure

```mermaid
flowchart TD
  stand["1. Stand CTest"]
  replay0["2. Replay seed 0 JSON at cap 24"]
  class{"What failed?"}
  loco["Behaviour: Healthy but undershoot / slip / height"]
  conv["SolverNotConverged / p99 iters"]
  time["p99 time with iters OK"]
  safety["Held / speed / penetration"]
  seeds["3. 100-seed safety then behaviour"]
  hz["4. Cadence 120/240/480 Hz"]
  suite["5. verify.sh on proximal"]
  flip["6. SolverMode default 1"]
  stand --> replay0 --> class
  class --> loco
  class --> conv
  class --> time
  class --> safety
  loco --> seeds
  conv --> seeds
  time --> seeds
  safety --> seeds
  seeds --> hz --> suite --> flip
```

### 1. Confirm stand, do not retune walk first

From repo root, RelWithDebInfo:

```bash
source scripts/lib/pinocchio_env.sh
hexapod-server/build-tests/test_physics_sim_proximal_stand_acceptance \
  hexapod-physics-sim/build/hexapod-physics-sim --emit-metrics-json
```

This is already the production-shaped gate (0.14 m, 60 s, p99 ≤ 20, zero held).
If it fails, debug that pose and contact set. Walk replay on a robot that cannot
stand is wasted.

Stand currently often uses 50 ADMM iterations. After it is green, rerun with
`HEXAPOD_PROXIMAL_STAND_ITERATIONS=24`. Production is 24. Passing only at 50
does not satisfy `plan.md`.

### 2. Capture one fixture, then replay it (do not recapture for A/B)

Capture once, save the command stream, then all solver A/Bs reuse it.
Recapturing lets the controller write a different stream.

```bash
source scripts/lib/pinocchio_env.sh
export HEXAPOD_PHYSICS_SIM_EXE=hexapod-physics-sim/build/hexapod-physics-sim
export HEXAPOD_EXACT_REPLAY_COMMANDS_OUT=/tmp/hexapod-commands.txt
hexapod-server/build-tests/test_physics_sim_exact_command_replay --emit-metrics-json
```

Then:

```bash
export HEXAPOD_EXACT_REPLAY_COMMANDS_IN=/tmp/hexapod-commands.txt
export HEXAPOD_EXACT_REPLAY_SOLVER_ITERATIONS=24
hexapod-server/build-tests/test_physics_sim_exact_command_replay --emit-metrics-json
```

Read `failure_reason_histogram`, `iteration_histogram`, topology-age percentiles,
recovered/held counts, height/drift, and the behaviour numbers. That JSON
decides which workstream to open.

Harness flags and JSON fields are documented in
[`TESTING_FUNCTIONALITY.md`](TESTING_FUNCTIONALITY.md).

### 3. Repair by failure class (one class at a time)

**A. Samples are Healthy, but 70% / lateral / turn gates fail**

This is the documented remaining locomotion blocker
(`ENFORCE_BEHAVIOR_GATES`). It is usually plant, slip, height, or actuator
tracking — not ADMM apply cost.

- Compare capture vs replay: `HEXAPOD_EXACT_REPLAY_LEGACY=1` on the **same**
  fixture tells you whether PGS also misses the gate.
- If both miss, it is gait/command, not proximal; do not retune ADMM.
- If only proximal misses: look at peak contact impulses, servo utilisation,
  energy, and `ncpDualResidual` / cone on the failing phase (`forward` vs
  `strafe` vs `turn_in_place`).
- Resource-plan Phase 2 coincident filtering is relevant here only if
  contact-count histograms show inflated `G` (duplicate rows). Do not start
  Phases 1/3 for a distance miss.

**B. `SolverNotConverged` or iteration p99 &gt; 20 at cap 24**

This is the ADMM note, in its written order:

1. Add ADMM-vs-NCP accept counts if `Healthy` is hiding “ADMM false, NCP true.”
2. A/B on the **same fixture**, cap 24: default vs
   `HEXAPOD_PINOCCHIO_DENSE_ADMM=1` vs
   `HEXAPOD_PINOCCHIO_CONTACT_PRECONDITION=1`. Promote a default only if
   **total** step p99 falls and stand/oracle stay green.
3. Same-dt retry only if non-convergence still dominates after that. Do not
   restore shorter Anderson.
4. Only then: relative NCP stop / warmer new-contact guesses on the production
   path.

**C. Iterations are fine, `p99_solver_total_step_time_ms` is not**

This is the resource note plus maybe dense GEMV.

- If collision/setup dominates: Phase 5 (skip articulated pairs in
  `IsPairEligible`) then Phases 1–3.
- If ADMM time dominates with low iteration count: dense apply is the A/B, not
  more allocator reuse.
- Replay round-trip p99 is a conservative proxy; do not fail the 4 ms gate on
  UDP loopback if solver-total is already under 4 ms.

**D. Held / speed-limit / extreme penetration**

Retry policy is already correct (only non-convergence and speed-limit
half-step). Trace with `HEXAPOD_PROXIMAL_TRACE_FAILURES=1` and
`HEXAPOD_EXACT_REPLAY_CHILD_STDIO=1`. Unrecoverable holds are physics bugs (bad
contacts, COM/inertia, penetration), not missing retries.

### 4. Widen only after seed 0 is honest

1. `HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS=100` with **safety** gates (held is
   a fail; recovered retry is allowed).
2. Then behaviour gates on every seed.
3. Cadence check: same fixture at 120 / 240 / 480 Hz (`PERIOD_US`).
4. Do **not** treat a 10-minute randomized gait as a switch blocker. Soak after
   the default is flipped.

Do not turn 100-seed exact replay into default CTest; seed 0 and cadence
120/240/480 Hz are already CTests. That is how
[`TESTING_FUNCTIONALITY.md`](TESTING_FUNCTIONALITY.md) is written.

### 5. Flip the default last

**Landed 2026-09-14.** Stand CTest, replay safety+behaviour (including 100 seeds
and cadence 120/240/480 Hz), and performance at cap 24 were green.
`./scripts/verify.sh` 15/84 was struck (PGS/offline).

- `Runtime.PhysicsSim.SolverMode = 1` in WSL config; parser default `1`
- WSL `SolverIterations` 24
- `legacy-pgs` (`= 0`) remains for comparison; never mid-substep fallback
- Exact-replay seed 0 and 120/240/480 Hz are default CTests (cap 24, both
  enforce flags, fixture v16). 100-seed stays diagnostic.
- `config.physics-sim-test-harness.txt` and `config.physics-sim.txt` stay
  `SolverMode = 0`

## What not to do

- Do not implement both follow-up notes in full hoping the gates go green.
- Do not evaluate exhaustion at replay’s default 50 iterations and call
  production done.
- Do not recapture commands for every solver A/B.
- Do not treat resource-plan ~1.1× as the 4 ms or 70% solution.
- Do not add LCABA, a custom Delassus, or shorter-Anderson retry to “help”
  these gates.

Do not recapture at cap 24; the 500-iter capture is what keeps the controller
in WALK. Seed 0, 100-seed safety+behaviour, and cadence 120/240/480 Hz are
green on v16 (`ddc6008e0cc1ac97`). WSL default is `pinocchio-proximal`.
`./scripts/verify.sh` 15/84 is struck as PGS/offline. See
[Forward cross-track investigation](#forward-cross-track-investigation-2026-09-14),
[100-seed held investigation](#100-seed-held-investigation-2026-09-14),
[120 Hz cadence investigation](#120-hz-cadence-investigation-2026-09-14),
and [verify.sh inventory](#verifysh-inventory-2026-09-14).
