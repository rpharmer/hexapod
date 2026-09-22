# Closing `plan.md` Default-Switch Gates

Date: 2026-09-12  
Campaign run: 2026-09-12 evening (RelWithDebInfo, WSL)  
Updated: 2026-09-15 (pitch sign, recovery resync, armature, STAND height, turn `cmd_yaw`; last-resort 2× NCP; reverse 5-contact NCP and tilt path-before-rate-trip are Class A residuals)

## Latest walking follow-up (2026-09-22)

The rebuilt default passes reverse ×5, isolated turn ×5 and complete sequential
walk-distance ×5 with zero held samples, plus canonical aggressive governor
(two strides). The changes preserve retry damping while reducing proportional
drive, and enable bounded turn-position feedback only for physics bridges with
reliable absolute position. Mode 1 remains default, including its existing CCP
recovery; implicit actuation and gravity feedforward remain off. See
[walking campaign §3.24](SEQUENTIAL_WALK_DISTANCE_LEFTOVERS.md) for the evidence,
rejected alternatives and current wider-verification status: server **97/98**
(only tilt path-before-fault remains), full physics **68/75** (seven legacy
World/scene failures also reproduce with the changes disabled), firmware **3/3**,
simple-sim smoke pass. Full root verification is therefore still red. The historical
red status below is retained as the first campaign record, not the latest
sequential result. No new 100-seed or soak qualification is claimed.

## Status after the first campaign

Stand, seed 0, 100-seed, and cadence 120/240/480 Hz are **green**. WSL default is
**pinocchio-proximal** (`SolverMode = 1`, `SolverIterations = 24`); parser default
is `1`. Exact-replay seed 0 and 120/240/480 Hz are default CTests (cap 24, both
enforce flags, fixture v16). 100-seed stays a diagnostic override.
`./scripts/verify.sh` live-physics CTests now default to pinocchio-proximal at
production **0.14 m / cap 24**. Offline height-margin is green. **2/15** of the
former red set remain Class A closed-loop misses (WAVE and slow-fwd height
closed; walk-distance and regression tilt still red). Chassis-only speed guard
was tried and reverted. SpeedLimit retry at 0.5 PD gain was insufficient by
itself; applying the existing actuator target-rate envelope to STAND recovery
closed the WALK-to-STAND SpeedLimit hold class. Recovery-only target resync
after a held rollback then closed the post-hold SpeedLimit cascade;
canonical `aggressive_governor` now passes. Isolated forward / slow-forward /
straight / turn-in-place pass. Isolated reverse still flakes on a 5-contact
NCP complementarity hold (not a shared-process harness artefact). Last-resort
dt/4 cold steps recovered some of those snapshots under 2×, then dumped
leftover-ω tibia SpeedLimit on forward and broke isolated straight stay-WALK;
reverted to two cold `dt/2` 48-iter steps. Cold 96 at the same `dt/2` still
leaves dual over 2e-3 on half of the held snapshots (not iteration-limited).
Last-resort graze filtering of 50/100 µm extras is **not** a clean 3-loaded-foot
fix: 50 µm often drops nothing or the dual-worst row; 100 µm can drop joint 13.
Isolated reverse stay-WALK is a **Class A residual** (hard 5-contact NCP).
`tilt_safety_trip` is a second **Class A residual**: rate-rule TIP_OVER after
~15 mm of pre-fault travel vs the 0.10 m path gate (do not loosen 0.10 or 0.45).
Sequential walk-distance remains red. See
[hold-class census](#hold-class-census-no-lever-2026-09-14),
[SpeedLimit retry gain](#speedlimit-retry-gain-hold-class-2026-09-14),
[recover BUS_TIMEOUT census](#recover-bus_timeout-census-no-lever-2026-09-15),
[recovery target resync](#recovery-target-resync-hold-cascade-2026-09-15),
[stay-WALK hold census](#stay-walk-one-sample-hold-census-2026-09-15),
[stay-WALK initiating NCP hold](#stay-walk-initiating-ncp-hold-2026-09-15),
[servo armature and stand height](#servo-armature-and-stand-height-2026-09-15),
[turn-in-place cmd_yaw](#turn-in-place-cmd_yaw-construction-2026-09-15),
[isolated reverse NCP complementarity](#isolated-reverse-ncp-complementarity-2026-09-15),
[last-resort 4-contact omit](#last-resort-4-contact-omit-2026-09-15),
[last-resort quarter-steps](#last-resort-quarter-steps-2026-09-15),
[last-resort 96-iter](#last-resort-96-iter-2026-09-15),
[last-resort graze filter](#last-resort-graze-filter-2026-09-15),
[tilt path-before-rate-trip](#tilt-path-before-rate-trip-2026-09-15),
[hold batch remesure](#chassis-only-speed-guard-hold-batch-2026-09-14), and
[verify.sh inventory](#verifysh-inventory-2026-09-14). Keep `legacy-pgs` (`= 0`)
for comparison. The iterations-only `PhysicsSimBridge` ctor is still PGS.
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
| `./scripts/verify.sh` | **Retargeted 2026-09-14.** Live-physics CTests use pinocchio-proximal at 0.14 m / cap 24. Offline `scenario_body_height_margin` **pass**. Isolated reverse still flakes on 5-contact NCP complementarity; sequential walk-distance remains red. Regression tilt path-before-TIP_OVER remains red. Canonical `aggressive_governor` **pass**. WAVE height, slow-fwd height, tripod support, stand, and v16 multi-rate screens pass. Chassis-only speed guard remains reverted. Smoke still uses `config.sim.txt`. See [verify.sh inventory](#verifysh-inventory-2026-09-14) |
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

**As written (PGS / crouch / offline envelope), before retarget:** 14/15 failed
(`physics_sim_nav_waypoints` already passed at 0.06 m PGS). Offline
`scenario_body_height_margin` failed because TOMLs command **0.14 m** while the
test floor was standing ~0.137 + squat 0.020 ⇒ **0.156 m**. Live PGS at 0.14 m
tripped `BODY_COLLAPSE` (fault 8) with body height ~0.032 m. Walk-distance at
0.06 m PGS failed slow-forward direction. Iterations-only
[`PhysicsSimBridge`](../hexapod-server/include/hardware/physics_sim_bridge.hpp)
ctors still default to `LegacyPgs`; they ignore WSL `SolverMode`. Smoke uses
[`config.sim.txt`](../hexapod-server/config.sim.txt).

**Retarget (same day):** live failing binaries take
`productionProximalSolverSettings()` (pinocchio-proximal, cap **24**). Default
heights **0.14 m** (walk-distance / nav / regression 0.10). Offline floor is
production **0.14 m**; scenario TOMLs unchanged. Int ctor and harness
`SolverMode = 0` unchanged. `HEXAPOD_WALK_TEST_SOLVER_MODE=legacy-pgs` remains
an A/B (no `max(50, …)` bump).

**After retarget (cap 24, 0.14 m):** 10/15 **pass**, then height-hold remesure
moved WAVE to pass and reduced-support servo compensation closed tripod support
(**12/15**). Contact-consistent inertia then closed slow-fwd height
(**13/15**). 2 remain **Class A** (Healthy undershoot /
tracking). Do not retune gait, plant `L`, μ, or ADMM on this evidence. Do not
hide the remainder with CTest labels. See
[closed-loop height sag](#closed-loop-height-sag-slowwave-2026-09-14) and
[slow-fwd plant vs command](#slow-fwd-walk-height-plant-vs-command-2026-09-14).

| CTest | After retarget | Notes |
| --- | --- | --- |
| `scenario_body_height_margin` | **Pass** | Command floor 0.14 m; TOMLs stay 0.14 |
| `physics_sim_walk_entry_tracking` | **Pass** | |
| `physics_sim_walk_stability` | **Pass** | |
| `physics_sim_turn_foot_clearance` | **Pass** | |
| `physics_sim_oblique_walk_clearance` | **Pass** | |
| `physics_sim_turn_raw_contact_loss` | **Pass** | |
| `physics_sim_slow_fwd_walk_contact_loss` | **Pass** | |
| `physics_sim_nav_waypoints` | **Pass** | Now 0.14 m proximal (was 0.06 m PGS pass) |
| `physics_sim_navigation_acceptance` | **Pass** | |
| `motion_performance_suite` | **Pass** | smoke |
| `physics_sim_walk_distance` | **Fail** Class A | Last-resort 2× NCP accept kept. Isolated cases usually pass; sequential CTest still stay-WALK on reverse 5-contact NCP (`ncp_dual` ~0.006). See [stay-WALK initiating NCP hold](#stay-walk-initiating-ncp-hold-2026-09-15) |
| `physics_sim_slow_fwd_walk_foot_clearance` | **Pass** | Contact-consistent CRBA on load-bearing feet (2.625 cap, 50 ms blend). After hold-batch revert remesure min body *z* **0.141 m** (undershoot **−1.4 mm** vs 10 mm). 10 mm gate unchanged. See [contact-consistent inertia](#contact-consistent-inertia-slow-fwd-height-2026-09-14) |
| `physics_sim_wave_slow_walk_foot_clearance` | **Pass** | After `kBodyHeightHoldMaxEffectiveMarginM` 12→40 mm: undershoot 6.0–7.4 mm; after hold-batch revert remesure min **0.141 m**. Chassis-only guard flaked this gate (TIP_OVER) and was reverted. 10 mm gate unchanged |
| `physics_sim_tripod_support_baseline` | **Pass** | Static reduced-support gain compensation: **26–39 mm** tracking across contact-order seeds 0–4 vs 45 mm; true three-foot contact in the metrics window. Six-foot stand and moving-gait gains are unchanged. See [tripod support tracking](#tripod-support-commanded-foot-tracking-2026-09-14) |
| `locomotion_regression_suite` | **Fail** Class A | Canonical `aggressive_governor` **pass**. Stress `tilt_safety_trip` remains out: honest rate-rule TIP_OVER after ~15 mm pre-fault travel vs 0.10 m path. See [tilt path-before-rate-trip](#tilt-path-before-rate-trip-2026-09-15) |

`./scripts/verify.sh` was **not** re-run: the 15-set is not green (walk-distance
and regression tilt remain red). Remaining work is closed-loop collection at
0.14 m plus the separately classified tilt/NCP failures, not a solver-mode flip.
Do not stack a second SpeedLimit gain scale.

**Flip (2026-09-14):** WSL `SolverMode = 1`, `SolverIterations = 24`, parser
default `1`. Harness and `config.physics-sim.txt` stay `0`.

Remesure after flip (cap 24, v16, both enforce flags):

| Gate | Result |
| --- | --- |
| Stand 60 s | **Pass.** 12000 healthy, height **8.15 mm** (contact-CRBA remesure; was 9.81 mm at flip), foot RMS **7e-6 m**, p99 iters **7**, max iters **9** |
| Seed 0 at 5000 µs | **Pass.** v16 unchanged. Replay sets `HEXAPOD_PINOCCHIO_DISABLE_CONTACT_INERTIA` so the frozen plant stays spawn six-foot I. 120/240/480 Hz also **pass**. |

### Closed-loop height sag (slow/wave, 2026-09-14)

Named gates: `physics_sim_slow_fwd_walk_foot_clearance` and
`physics_sim_wave_slow_walk_foot_clearance` (command **0.14 m**, proximal cap
**24**, 10 mm undershoot vs min body *z* over every walk frame). Walk-distance
net is supporting only.

Census (no lever yet): stand-end *z* ~0.149–0.150 m (inside stand’s 10 mm).
First **120 ms** of walk is the high point (~0.149 m), not the sag. Min is after
120 ms. Median stays near command (slow-fwd 0.142 m, WAVE 0.138 m). Governor
stays at ~0.14 m (slow-fwd min 0.1395 m; WAVE min 0.136 m). PGS A/B on
walk-distance at 0.14 m collapsed (`BODY_COLLAPSE`, min height 0.031 m, error
0.109 m) — not the same 20–35 mm proximal sag.

Four-way: **not window** (excluding 120 ms would not change the min). **Not
governor** (command stays at 0.14 m). **Hold cap:** at the named min, sag was
20 mm / 35 mm so proportional hold requested more than
`kBodyHeightHoldMaxEffectiveMarginM` (12 mm).

One lever: raised that constant to **40 mm**. Did not touch integral gain, gait,
plant `L`, μ, or ADMM. Remesure: WAVE **pass** (undershoot 6.0–7.4 mm). Slow-fwd
still ~19 mm; after the raise, hold request at the dip is below 40 mm, so the
leftover is **plant/IK**. The follow-up census below splits that leftover.
10 mm still means min pose over the full walk window, not a post-transient score.

### Slow-fwd walk height (plant vs command, 2026-09-14)

Named gate: [`test_physics_sim_slow_fwd_walk_foot_clearance.cpp`](../hexapod-server/tests/test_physics_sim_slow_fwd_walk_foot_clearance.cpp)
(command **0.14 m**, proximal cap **24**, 10 mm undershoot vs min body *z* over
every walk frame). WAVE already **pass**. Walk-distance `forward_walk` net
(0.045 vs 0.05 m) is supporting only. Do not extend
`kStaticReducedSupportGainScale` (1.85× static, 250 ms dwell) into moving gait.

Census (proximal, `HEXAPOD_WALK_TEST_SOLVER_MODE` unset). Gates unchanged. CTest
remesure: min body *z* **0.118 m** (undershoot **22 mm**). Median **0.145 m**.
First 120 ms **0.149 m**; after **0.118 m**. Governor **0.137–0.140 m**. Stand-end
**0.149 m**. Commanded stance-foot body *z* median **−0.135 m**, min **−0.169 m**
(hold requesting extra lift at the dip, not a squat). Planned min **−0.166 m**.
Measured stance-foot world *z* **0.015 m**. Max stance commanded tracking **161 mm**.
Latch: `max_unchanged_target_s` **0**, `max_latch_candidate_s` **0**,
`reduced_support_latch_could_arm` **false**. Repeat runs in the same tree were
0.111–0.119 m (19–29 mm undershoot); same class.

Four-way:

- **Not latch leak.** Walk joint targets change every step; the static 1.85×
  predicate never dwells.
- **Not Cartesian command sag.** Governor stays at ~0.14 m. Median commanded
  stance-foot body *z* stays near **−0.14 m** (nominal for that height). The
  more-negative min is height hold, not workspace/IK sag. Body does not track a
  lowered command.
- **Plant during gait.** Commanded stance *z* stays near 0.14 m, stance feet stay
  on/above the plane, body dips cyclically after 120 ms.
- **Not contact sink.** Stance feet world *z* ~15 mm above the plane.

One lever: **stop.** Do not extend 1.85× into WALK, and do not raise plant `L`,
μ, ADMM, stall, the 40 mm hold cap, or the 10 mm gate. 10 mm still means min
pose over every walk frame. Closed later by
[contact-consistent inertia](#contact-consistent-inertia-slow-fwd-height-2026-09-14).

### Contact-consistent inertia (slow-fwd height, 2026-09-14)

Named gate: [`test_physics_sim_slow_fwd_walk_foot_clearance.cpp`](../hexapod-server/tests/test_physics_sim_slow_fwd_walk_foot_clearance.cpp)
(command **0.14 m**, proximal cap **24**, **10 mm**). The 1.75× moving-contact
gain was an existence proof; it is removed. PD stays `τ = Kp e − Kd q̇`
with the torque-speed envelope. Commanded rates now travel on `StepCommand`
(`joint_target_velocities[18]`; legacy 81-byte packets still reconstruct Δq/Δt).

Live plant: load-bearing tibia mask (geometric contact and previous-substep
normal impulse ≥ 2% of `m g Δt`), then
`AssignStanceLoadedServoInertias` on that mask. Six load-bearing feet keep spawn
nominal (1.5× cap). Reduced support recomputes at cap **2.625** and blends over
**50 ms**, at most once per physics step, only at production command period
4.5–5.5 ms. Static 1.85× latch does not stack on reduced-contact CRBA.
Exact-replay sets `HEXAPOD_PINOCCHIO_DISABLE_CONTACT_INERTIA=1` so fixture v16
(`ddc6008e0cc1ac97`) stays on the capture plant. Do not recapture.

Remesure (`HEXAPOD_WALK_TEST_SOLVER_MODE` unset, cap 24, 0.14 m):

| Gate | Result |
| --- | --- |
| Slow-fwd height | **Pass.** Min body *z* **0.141 m** (undershoot **−0.9 mm**) |
| WAVE height | **Pass.** Min body *z* **0.143 m** |
| Proximal stand 60 s | **Pass.** 12000 healthy, height **8.15 mm**, p99 **7**, max **9** |
| Tripod support | **Pass.** Tracking **39.7 mm** vs 45 mm; 720/720 three-foot |
| Exact-replay seed 0 | **Pass** at 5000 / 8333 / 4167 / 2083 µs |

10 mm still means min pose over every walk frame. `./scripts/verify.sh` was not
re-run. Remaining Class A: walk-distance hold/BUS_TIMEOUT and
regression aggressive-recover / tilt path-before-TIP_OVER. Chassis-only speed
guard was tried and reverted. See
[walk-distance and regression census](#walk-distance-and-regression-census-2026-09-14)
and [hold batch remesure](#chassis-only-speed-guard-hold-batch-2026-09-14).

### Walk-distance and regression census (2026-09-14)

Named gates: [`test_physics_sim_walk_distance.cpp`](../hexapod-server/tests/test_physics_sim_walk_distance.cpp)
and canonical [`test_locomotion_regression_suite.cpp`](../hexapod-server/tests/test_locomotion_regression_suite.cpp)
(proximal cap **24**, 0.14 m, `HEXAPOD_WALK_TEST_SOLVER_MODE` unset). Contact-consistent
CRBA is on for these live loops.

**Walk-distance `forward_walk` (0.20 m/s, 2400 steps).** The old net-progress miss
is closed. Path **0.975 m**, net **0.265 m** (gate 0.05), heading cosine **0.96**,
speed ratio **0.69** (band 0.20–0.90). It then leaves WALK: 2014 walk / 386
non-walk, `final_fault=BUS_TIMEOUT`. Solver: 1984 healthy, 129 recovered, **287
held**, 93 not-converged, max iters **48**, rollbacks **715**, stall utilization
**1.0**. `PhysicsSimBridge` rejects `HeldLastGood` as a failed read (`!bus_ok`),
and safety maps that to BUS_TIMEOUT. This is Class D hold → Class A mode drop,
not circling and not a 500 ms UDP hitch. Slow closed-loop walk still works:
regression `steady_forward_walk` at **0.08 m/s** passes (path 0.62 m, net 0.15 m,
fault NONE).

**Regression canonical.** `steady_forward_walk`, `turn_in_place` (yaw **0.41 rad**
vs 0.25), `gait_transition_stability`, `command_timeout_fallback`, and
`low_support_walk` **pass**. Two remain:

| Case | Historical note | Now |
| --- | --- | --- |
| `turn_in_place` | yaw miss | **Pass** |
| `aggressive_governor` | stride_count / stepping | Walk 400–959 completes (stride 2, path 1.02 m, governor scales). WALK→STAND recover at ~0.43 m/s: three STAND samples then `bus_ok=false`, pose frozen, BUS_TIMEOUT at step 963. Same HeldLastGood mapping as walk-distance, at the mode change |
| `tilt_safety_trip` | motion before TIP_OVER | Still that class. Tightened `max_tilt_rad=0.25` and `rapid_body_rate_radps=0.45`. TIP_OVER at step 492 (`bus_ok` still true) with roll **0.14** (below 0.25), so the **rate** rule trips. Path **0.073 m** vs 0.10. 92 walk frames (~0.46 s) at heading π/2, 0.45 m/s command |

Do not loosen 0.05 m net, 0.10 m tilt path, or the rapid-rate envelope on this
evidence. Do not extend 1.85× into WALK. The speed-dependent hold that becomes
BUS_TIMEOUT (walk-distance 0.20 m/s and aggressive recover) was the next named
class; chassis-only speed guard was measured and **reverted** (see
[hold batch](#chassis-only-speed-guard-hold-batch-2026-09-14)). Tilt
path-before-rate-trip stays a second, separate class.

### Chassis-only speed guard (hold batch, 2026-09-14)

Named gate: `physics_sim_walk_distance` `forward_walk` must stay in WALK.
Supporting: canonical `aggressive_governor` must not BUS_TIMEOUT on WALK→STAND
recover. Tilt path-before-TIP_OVER stayed out of batch. Do not publish
`HeldLastGood` as healthy, lengthen the 500 ms poll, raise ADMM / μ / `L` /
stall, drop the CRBA cap, extend 1.85× into WALK, recapture v16, or loosen
0.05 m net / 10 mm height.

**Census** (`HEXAPOD_WALK_TEST_SOLVER_MODE` unset). Histogram of
`solver_failure_reason` on recovered/held frames: **1420/1421 held = SpeedLimit**.
Recovered mixed (NCP 65, SpeedLimit 29). SpeedLimit dominated held+recovered, so
the lever proceeded. Peak pre-integration WORLD_ALIGNED ω **20.3 rad/s** vs
`ProximalSolverSettings::maxAngularSpeed = 10` on every body frame. Linear peak
1.03 < 2.0. Distal WORLD_ALIGNED ω can exceed 10 inside the MG996R 7.48 rad/s
joint envelope. 0.08 m/s `steady_forward_walk` stays Healthy.

**Lever (applied, then reverted).** Pre-integration
`maxLinearSpeed` / `maxAngularSpeed` on the free-flyer only; other links reject
non-finite ω/v. Chassis caps stayed 2 m/s and 10 rad/s. No `vNew` clamp. Retries
unchanged.

**Remesure with the lever on.** Held went to **0**; SpeedLimit recovered went to
**0**; peak_pre_w still 23–33 (links allowed). That did **not** stay in WALK:

| Screen | With chassis-only guard |
| --- | --- |
| `forward_walk` | 0 held, recovered all NCP. One lucky net **0.529 m**. Typical: TIP_OVER after ~63 walk steps (`fault=3`, 2337 non-walk), net **0.027 m** vs 0.05, `peak_pre_w` **25** |
| Canonical regression | `steady_forward_walk` **pass**. `turn_in_place` and `gait_transition_stability` **TIP_OVER** (were pass). `aggressive_governor` recover reached STAND with `fault=NONE` (BUS_TIMEOUT mapping closed) but `stride_count=1` vs 2. Timeout / low-support **pass**. Tilt path still **0.073 m** vs 0.10 |
| WAVE | CTest once **pass**, then emit-metrics **TIP_OVER** at step 5250 (closed-gate flake) |
| Slow-fwd / tripod / stand | **Pass** (min *z* 0.141 m; tracking 39.2 mm vs 45; stand 9.14 mm, 12000 healthy) |

Integrating the previously rejected distal WORLD_ALIGNED ω couples into body
rate (yaw peak **1.4 rad/s**, regression body rate **3.9–5.2 rad/s**) and trips
the existing rapid-rate TIP_OVER rule (0.45 rad/s). That is not a SpeedLimit
hold. Do not loosen the rate envelope here. Do not raise ADMM.

**Revert.** All-body 10 rad/s guard restored. After revert: WAVE **pass** (min
*z* **0.141 m**); slow-fwd **pass** (min *z* **0.141 m**). Walk-distance on that
binary: 1 held (NCP), recovered SpeedLimit 53 + NCP 117, stayed WALK
(`mode=WALK`, `fault=NONE`), failed the speed band. Exact-replay was not
re-run: walk-distance is not green; fixture v16 is unchanged. `./scripts/verify.sh`
was not re-run.

Remaining named class is still SpeedLimit **HeldLastGood** → `!bus_ok` →
BUS_TIMEOUT at 0.20 m/s / aggressive recover. Skipping the link ω guard trades
that for TIP_OVER and a WAVE flake, so it is not the lever. Next class is not
ADMM and not tilt-path. Post-revert five-run taxonomy:
[hold-class census](#hold-class-census-no-lever-2026-09-14).

### Hold-class census (no lever, 2026-09-14)

No plant lever. All-body 2 m/s / 10 rad/s reject/retry/hold unchanged. Census
only: which body frame trips SpeedLimit, and how `forward_walk` fails after the
chassis-only revert.

Instrumentation (kept): pre-integration records chassis WORLD_ALIGNED ω, max
link ω, and the max-ω body that exceeded the cap (`chassis` / `coxa` / `femur`
/ `tibia`), plus load-bearing vs swing on that leg. Walk-distance prints
reason histograms, frame/support counts, max consecutive `HeldLastGood`, and
first non-WALK / first fault. Env `HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT` logs
per-trip lines. Do not publish held as `bus_ok`, lengthen 500 ms, raise ADMM,
or skip the link check.

**Five `forward_walk` runs** (0.20 m/s, cap 24, 0.14 m, solver mode unset).
Chassis peak ω **2.3–4.6 rad/s** (always &lt; 10). CTest fail line is the first
gate that trips (speed is checked before stay-WALK).

| Run | CTest | Final | Held / streak | SpeedLimit winner | Support | chassis_w / max_link_w |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | stay-WALK | FAULT **BUS_TIMEOUT** @ 963 | 359 SpeedLimit / **335** (≥100) | tibia 340, femur 27 | swing 363, stance 4 | 2.33 / 15.6 |
| 2 | speed-band | **WALK** NONE | 0 / 0 | femur 15, none 23 (recovered) | mixed | 2.34 / 19.0 |
| 3 | stay-WALK | FAULT **BUS_TIMEOUT** @ 258 | 2142 SpeedLimit / **2142** | femur 2147 | **stance 2132**, swing 15 | 4.58 / 24.3 |
| 4 | speed-band | FAULT **TIP_OVER** @ 385 | 0 / 0 | femur 1 | stance 1 | 2.70 / 15.7 |
| 5 | speed-band | first BUS_TIMEOUT @ 508, final **TIP_OVER** | 3 NCP / **3** (&lt;100) | femur 14, none 28 | mixed | 2.61 / 14.0 |

No majority class. Live misses on the named CTest:

1. **HeldLastGood → BUS_TIMEOUT** (2/5), streak ≥ 100 at 5 ms. Winner is a
   **link**, never chassis. Not uniquely swing tibia: run 1 is swing tibia, run 3
   is **stance femur**. Chassis-only is still the wrong lever (already remesured).
2. **Stay-WALK speed-band crawl** (1/5): 0 held, NCP recovered majority, `fault=NONE`.
3. **TIP_OVER** (2/5) even with the all-body 10 rad/s guard. Speed-band is the
   CTest message because it is checked first.

**`aggressive_governor`:** still **BUS_TIMEOUT** at recover (WALK 400–959, three
STAND samples 960–965, FAULT @ 966). `stride_count=2`. Same hold→`!bus_ok`
mapping as walk-distance class 1. Tilt path-before-TIP_OVER stays out.

Stop-rule table: do **not** reclassify as crawl-only, chassis-guard, or “hold
is gone.” Next lever (later batch) must keep rejecting or slowing the vNew that
trips **link** WORLD_ALIGNED ω; it must not skip the link check. Stance-femur
holds mean this is not only a swing-foot composition effect. Do not raise ADMM.
Do not loosen rapid-rate / tilt path / 0.05 m net / 10 mm.
Follow-up lever and remesure:
[SpeedLimit retry gain](#speedlimit-retry-gain-hold-class-2026-09-14).

### SpeedLimit retry gain (hold class, 2026-09-14)

Named class: SpeedLimit **HeldLastGood** → `!bus_ok` → BUS_TIMEOUT. Supporting:
canonical `aggressive_governor` recover must not BUS_TIMEOUT. Tilt path and
TIP_OVER-without-hold stayed out. Do not skip the all-body 10 rad/s link check
(chassis-only already failed). Do not publish held as `bus_ok`, lengthen 500 ms,
raise ADMM / μ / `L` / stall, drop CRBA 2.625, extend 1.85× into WALK, recapture
v16, or loosen 0.05 m net / 10 mm / tilt path / rapid-rate.

**Lever (kept).** [`pinocchio_hexapod.cpp`](../hexapod-physics-sim/src/demo/pinocchio_hexapod.cpp)
`advanceOnce` multiplies the existing PD request
(`servoGainScale * loadGainScale * …`) by an optional per-call scale (default
**1.0**). Stall envelope unchanged. When `firstAttempt.failureReason == SpeedLimit`
only, the retry chain (same-`dt`, warm `dt/2`, cold 48) runs at
`kSpeedLimitRetryGainScale = 0.5`. NCP retries stay 1.0. Caps stay 2 m/s and
10 rad/s on **every** body. `dt/2` cannot help SpeedLimit: the guard is velocity,
not step size. One constant; do **not** stack 0.25.

**Remesure** (`HEXAPOD_WALK_TEST_SOLVER_MODE` unset).

**`aggressive_governor`:** still **BUS_TIMEOUT**. WALK 400–959, STAND 960–962,
FAULT @ **963** (`bus_ok=false`). `stride_count=2`, path 0.465 m, governor still
attenuates. Same recover mapping as the census. WAVE, slow-fwd, stand 60 s,
tripod, and exact-replay v16 were **not** re-run: aggressive is not green.

**Five `forward_walk` runs** (0.20 m/s, cap 24, 0.14 m). Chassis peak ω
**2.7–3.7 rad/s** (always &lt; 10). Link peak ω still **13.9–19.2**. Recovered
SpeedLimit is common (9–59/run); long hold streaks are gone.

| Run | CTest | Final | Held / streak | SpeedLimit recovered | Winner | chassis_w / max_link_w |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | net 0.045 vs 0.05 | FAULT **TIP_OVER** @ 690 | 0 / 0 | 9 | femur 4 | 3.50 / 13.9 |
| 2 | speed-band 0.137 | **WALK** NONE | 0 / 0 | 59 | femur 21 | 3.10 / 18.0 |
| 3 | stay-WALK | first **BUS_TIMEOUT** @ 1686, final WALK NONE | 2 SpeedLimit / **2** | 30 | femur 9, tibia 2 | 2.68 / 16.7 |
| 4 | net 0.047 vs 0.05 | FAULT **TIP_OVER** @ 418 | 0 / 0 | 12 | femur 3, tibia 1 | 3.75 / 19.2 |
| 5 | forward_walk **pass** (net 0.400, ratio 0.385) | **WALK** NONE | 0 / 0 | 41 | femur 9 | 2.71 / 17.9 |

Named gate **BUS_TIMEOUT 0/5: miss** (1/5, run 3). Stay-WALK speed-band and
TIP_OVER-with-0-holds remain out of class; they still fail the CTest. Run 5's
full binary then failed `slow_forward_walk` on a 2398-step SpeedLimit hold
cascade — 0.5 does not always recover; that case was not this batch's named
five-run.

Stop: do **not** stack 0.25. Do not skip link ω. Do not raise
`maxAngularSpeed`. `./scripts/verify.sh` was not re-run (walk-distance CTest is
not fully green). Fixture v16 is unchanged. Recover BUS_TIMEOUT is named in
[recover census](#recover-bus_timeout-census-no-lever-2026-09-15). Tilt path
stays out.

### Recover BUS_TIMEOUT census (no lever, 2026-09-15)

No plant lever. Named screen: one canonical `aggressive_governor` recover.
Walk 2-sample hold, TIP_OVER / speed-band, and tilt path stayed out. Do not
stack 0.25, skip link ω, publish held as `bus_ok`, lengthen 500 ms, or recapture
v16.

Instrumentation (kept): `[proximal-speed-limit]` / `[proximal-first-failure]`
print `pd_gain`, `dt`, and peak PD `|error|`. Regression
`CapturingPhysicsSimBridge` snapshots solver telemetry on every read (including
failed) and latches the first failed read. `HEXAPOD_LOCOMOTION_CHILD_STDIO=1`
un-quiets the sim child. Recover line + first-held fields on metrics JSON.

**`aggressive_governor`** (`HEXAPOD_WALK_TEST_SOLVER_MODE` unset). Still
**BUS_TIMEOUT**. WALK 400–959, STAND 960–1010, FAULT @ **1011**. First failed
read:

| Field | Value |
| --- | --- |
| `solver_status` | **HeldLastGood** |
| `failure_reason` | **SpeedLimit** |
| `speed_limit_frame` / support (held packet) | none / unknown |
| chassis ω / max link ω (held packet) | 2.74 / 7.79 |
| `retry_count` / `held_state_count` | 85 / 1 |
| `stride_count` / path | 2 / 0.99 m |

Not a poll timeout and not an NCP-only first fail. 68 NCP first-failures exist
in the trace stream (reason 7, ω recorded 0); the sample that maps to
`!bus_ok` is SpeedLimit **HeldLastGood**.

**Traces** (`HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT=1`, 1095 trips, 288 first
attempts). Chassis ω **2.0–2.7 &lt; 10**. Every first-attempt trip is a **femur**
WORLD_ALIGNED ω **10.1–18.4** (median **14.4**). Support is mostly **swing**
(1088/1095). Recover lock is `leg_5_femur_body` swing.

| Attempt | `pd_gain` | `max_link_w` | peak PD `\|error\|` |
| --- | --- | --- | --- |
| First | 1.0 | **14.43** | 1.12 rad at recover lock (walk onset 0.43–0.75) |
| Same-`dt` / half-step retry | 0.5 | **13.92–14.42** | same 1.12 (targets unchanged) |

**0/269** SpeedLimit retry chains ended with `max_link_w` ≤ 10. `dt/2` does not
help. 0.5 gain knocks ~0.5 rad/s off 14, not 4. The STAND recover command is a
**large PD error** (~1.12 rad), not leftover walk ω on the chassis.

Stop: do **not** stack 0.25 (already shown not to clear 14→10). Do not skip the
link check. Do not raise `maxAngularSpeed`. Next lever must slow that **swing
femur** vNew on the recover STAND target step so first-attempt or 0.5-retry ω
falls under 10, without publishing held as `bus_ok`. WAVE / v16 / five
`forward_walk` were not re-run. `./scripts/verify.sh` was not re-run.

### WALK-to-STAND actuator slew continuity (2026-09-15)

Root cause confirmed at the command boundary. `RobotRuntime::controlStep()`
applied the MG996R target-rate envelope only when the current request was
`WALK`. A normal recover request, or a fusion-forced recover request, changed
the mode to `STAND` before the clamp decision. The displaced swing femur could
therefore receive the static stand target in one control sample. This matches
the recover census: a 1.12--1.31 rad PD error and a distal femur frame above the
10 rad/s all-body guard, while chassis angular speed remained well below the
guard.

**Lever (kept).** Servo target slew limiting now applies to both actively
positioned modes, `WALK` and `STAND`. `SAFE_IDLE` and `FAULT` remain outside the
policy. This does not change gait construction, target geometry, the MG996R
torque-speed curve, solver settings, or any speed/safety gate. The existing
per-joint configured positive/negative no-load rate remains the only limit.
A unit assertion fixes the mode policy alongside the existing small-step and
large-jump slew checks.

**Remesure.** The pre-change canonical `aggressive_governor` reproduced
`HeldLastGood / SpeedLimit / BUS_TIMEOUT`; the trace winner was again a swing
femur. Four post-change canonical runs produced **zero SpeedLimit-held
samples**. Three passed the whole case with final `STAND / NONE`. One separate
run held at recover for `solver_not_converged`; that is the existing NCP class,
not a residual command-step SpeedLimit, and no ADMM lever was applied.

The required compatibility screen stayed green: WAVE slow-walk height,
slow-forward height, static tripod tracking, proximal 60 s stand, and frozen
fixture v16 at its native rate plus 120/240/480 Hz. The fixture was not
recaptured. `./scripts/verify.sh` remains deferred until walk-distance is fully
green.

One supporting `physics_sim_walk_distance` remesure also had zero held samples,
stayed in WALK, and reported no fault. It remained red on the separate
collection class: average horizontal speed **0.0218 m/s** for a **0.20 m/s**
command (ratio **0.109**, lower gate **0.20**). Recovered retries were 81 NCP
and 18 SpeedLimit. Do not attribute this no-hold crawl to the solved recovery
target step.

### Forward pitch sign and no-hold crawl (2026-09-15)

The no-hold walk-distance crawl was a body-pose sign error, not low friction or
an under-commanded gait. The walk census showed a governed command around
0.16--0.19 m/s while loaded stance targets moved only a few centimetres per
second. Nearly every loaded stance sample was under the absolute 0.20 rad
emergency-tilt hold. Signed attitude then identified the feedback loop: a
positive forward command requested **-0.22 rad** pitch and the chassis moved in
the same negative direction (mean about **-0.30 rad**, minimum **-0.62 rad**).
The pose request reinforced the natural forward pitching moment, crossed the
hold threshold, and parked each stance latch at its stroke/workspace boundary.

**Lever (kept).** `kLeanPitchPerVx` retains its 0.22 magnitude but changes sign.
Forward motion now requests counter-pitch. No gait, contact, actuator, solver,
or safety limit changed. A controller regression assertion fixes the new sign.
Zero-lean and reduced-magnitude experiments restored some translation but were
less stable; a feedback target-lead clamp and a swing-femur gain experiment did
not reduce the remaining held class and were reverted.

With corrected sign, representative pre-fault runs command loaded stance sweep
around **0.08--0.15 m/s**, mean signed pitch is roughly **-0.02 to -0.07 rad**,
and the chassis collects substantial forward motion instead of the old 0.109
speed-ratio crawl. One full remesure travelled **0.51 m** in the commanded
direction before an independent swing-femur SpeedLimit hold at step 1565; its
body-forward mean was **0.169 m/s**. The full walk-distance CTest therefore
remains red on `HeldLastGood -> BUS_TIMEOUT`, not on the no-hold lower speed
band. Do not hide that residual by publishing a held sample or loosening either
speed band.

Compatibility screens after the sign change: controller pose/stability unit,
motion-through-IK/FK unit, slow-forward height, WAVE height, static tripod
support, proximal 60 s stand, and frozen fixture v16 at native/120/240/480 Hz
all **pass**. The fixture was not recaptured. `./scripts/verify.sh` remains
deferred until walk-distance is fully green.

### Recovery target resync (hold cascade, 2026-09-15)

Named leftover after the pitch-sign fix: a single SpeedLimit `HeldLastGood`
was enough to start an unbounded hold cascade. After a held rollback the
simulator stored the current joint angles as its internal command state, then
ignored that state on the next step and reapplied the distant server target in
full. The same SpeedLimit therefore repeated forever and mapped to
`BUS_TIMEOUT`.

**Lever (kept).** Recovery-only target resynchronisation in
[`pinocchio_hexapod.cpp`](../hexapod-physics-sim/src/demo/pinocchio_hexapod.cpp).
Healthy steps still use the server target directly. After a hold / unsupported
island / read-state failure, the internal command is latched to the measured
joints. On later steps it slews toward the live server stream at
**0.5 × no-load servo speed**, then returns to the unfiltered path once every
joint is inside that per-step delta. The first held sample remains invalid;
`HeldLastGood` is not published as `bus_ok`. Torque, friction, solver caps,
and the 10 rad/s link guard are unchanged.

**Remesure** (`HEXAPOD_WALK_TEST_SOLVER_MODE` unset, production proximal).

**`forward_walk`:** cascade closed. `solver_held=1` (`speed_limit`),
`max_held_streak=1`, recovered 350 (260 NCP / 90 SpeedLimit). Path 1.28 m,
net 0.60 m, average speed ratio **0.53** (gates 0.20–0.90). Mean signed pitch
**-0.019 rad**. Final mode WALK / fault NONE after automatic bus recovery.
CTest still **FAIL** stay-WALK: first non-walk at step **1492**
(`first_fault=BUS_TIMEOUT`), 108 non-walk samples. Do not hide that by
publishing the held sample or loosening stay-WALK. The residual is a separate
one-sample solver-recovery class.

**`aggressive_governor`:** **pass.** 1280 samples, stride 2, `fault=NONE`,
zero read fails / held states. WALK 400–959 then STAND 960–1279.

Compatibility screens after resync: WAVE height, slow-forward height, static
tripod support, proximal 60 s stand, and frozen fixture v16 at
native/120/240/480 Hz all **pass**. Pose/stability unit still **pass**. The
fixture was not recaptured. `./scripts/verify.sh` remains deferred until
walk-distance is fully green.

### Stay-WALK one-sample hold census (2026-09-15)

Named leftover after recovery resync: `forward_walk` still fails stay-WALK on
the first `HeldLastGood -> BUS_TIMEOUT`, even when the speed band would pass.
Instrumentation (kept): walk-distance latches first-failed solver telemetry
after stand warmup and prints status/reason/frame/support/ω plus the
RecoveredRetry SpeedLimit streak immediately before the first hold. Traces
remain `HEXAPOD_WALK_TEST_CHILD_STDIO`, `HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT`,
and `HEXAPOD_PROXIMAL_TRACE_FAILURES`.

**Five `forward_walk` census runs** (production proximal, traces on):

| Run | Held | First failed reason | First frame/support | SL streak before hold | Stay-WALK |
| --- | --- | --- | --- | --- | --- |
| 1 | 2 (NCP+SL) | SpeedLimit | femur / stance | 2 (`none`) | fail @ 1073 |
| 2 | 630 | NCP | none | 0 | fail @ 907, then 629 SL cascade |
| 3 | 1 | NCP | none | 0 | fail @ 1607 |
| 4 | 0 | none | — | — | **pass** (slow-fwd also pass; reverse later cascaded) |
| 5 | 2 (NCP+SL) | NCP | none | 0 | fail @ 645 |

Traces (trust these, not the held packet ω): initiating SpeedLimit trips are
**femur** WORLD_ALIGNED ω **10–15** with peak PD `|error|` **0.36–0.62 rad**,
mixed stance/swing. Histogram `none` is a frame-name classification gap; the
winner string is `leg_N_femur_body`. Post-hold SpeedLimit cascades are a
different sample: PD error **~0.006 rad** with leftover femur ω **~10.7**, and
retry gain **0.5 increases** ω (damping term shrinks). Recovery slew is already
active there; the target is not the problem.

**Lever (reverted).** Arming post-hold recovery slew plus first-attempt 0.5
gain after `RecoveredRetry` SpeedLimit until a Healthy step. Five remesures
produced 704–1536 held SpeedLimit streaks. Reducing gain while PD error is
already tiny strips damping and recreates the leftover-ω cascade. Do not
restore. Do not stack 0.25, skip link ω, publish held as `bus_ok`, or loosen
stay-WALK.

Stay-WALK remains red on the **first** held sample. That sample is often NCP,
not SpeedLimit. A continuity lever aimed at SpeedLimit near-misses does not
close it. Next class must name the first `HeldLastGood` reason from this
latch, not the mixed recovered histogram.

### Stay-WALK initiating NCP hold (2026-09-15)

Named leftover after the one-sample SpeedLimit census: `forward_walk`
stay-WALK on the first `HeldLastGood`, usually `SolverNotConverged`. Census
instrumentation (kept): walk-distance prints first-failed iterations /
`ncp_dual` / `ncp_comp` / contacts / worst contact / rho, plus RecoveredRetry
NCP streak before the first hold. Exhausted retries print `[proximal-held]`
(first-attempt vs last-retry/cold) when `HEXAPOD_PROXIMAL_TRACE_FAILURES=1`.

**Census split (before last-resort 2×):** when the first hold is NCP, cold 48
drives `ncp_dual` from ~0.057 to ~0.0016 — just over the 1e-3 floor, 4–5
contacts. **Almost-feasible**, not leftover-ω (held packet `pre_w=0`; NCP
returns before the speed guard). Hundreds of first-attempt NCP samples still
RecoveredRetry.

**Lever (kept).** Last-resort cold half-steps only: if the first attempt was
NCP, accept at **2×** `ncpAbsoluteTolerance` (2e-3) on the existing 48-iter
cold path. First-attempt and standing ADMM (`1e-8`) stay at production
tolerances. SpeedLimit first-fails keep the 1e-3 floor. Do not raise cap 24,
μ, `L`, or dense ADMM.

**Remesure (2× in plant):**

| Gate | Result |
| --- | --- |
| Isolated `forward_walk` ×5 | **4/5 pass**, held 0. One NCP hold: cold `ncp_dual=1.8e-4` / `ncp_comp=0.00387` on 5 contacts — **stuck complementarity**, not almost-feasible dual. Do not raise last-resort above 2× |
| Isolated slow / reverse / straight / turn | **pass**, held 0. Turn net **0.181 m**, `raw_wz=0.45` |
| Sequential five-case CTest | **fail** on `reverse_walk` stay-WALK. Cold 48 `ncp_dual=0.00569` / `ncp_comp=0.00167`, 5 contacts, worst 7. Dual is far above 2e-3 (stuck set). Later leftover-ω tibia SpeedLimit cascade is out of class |
| `aggressive_governor` | **pass.** 1280 samples, stride 2, `fault=NONE`, held 0 |
| WAVE height | **pass.** min body *z* **0.139 m**, undershoot **0.83 mm** vs 10 mm |
| Slow-fwd height | **pass.** min body *z* **0.145 m** |
| Tripod support | **pass.** tracking **14 mm** vs 45 mm; 720/720 three-contact |
| 60 s stand | **pass.** 12000 healthy, height error **6.42 mm**, p99 iters **7**, held 0 |
| v16 200/120/240/480 Hz | **pass.** hash `ddc6008e0cc1ac97`, held 0, `behavior_gate_failures` 0. Fixture not recaptured |

Do not publish `HeldLastGood` as `bus_ok`. Sequential walk-distance stays red
on reverse 5-contact NCP (dual ~0.006 or complementarity ~0.0039). Isolated
forward leftover is the same complementarity class. `./scripts/verify.sh` was
not re-run.

### Servo armature and stand height (2026-09-15)

SpeedLimit femur trips on healthy walk were an inertia mismatch: the 25 rad/s
PD law is tuned with reflected inertia, but Pinocchio's plant had no joint
armature, so ABA saw ~0.00007 kg·m² coordinates. Adding free-joint CRBA
inertia as armature on the production contact-inertia path (frozen replay
still disables it) dropped peak link speed from ~15–20 rad/s to ~6.6–7.5.

Full-scale armature closed SpeedLimit holds but shifted STAND equilibrium
~12.6 mm above the 0.14 m command. Scale 0.01 kept stand but restored the
hold cascade. Quarter-scale (`kServoArmatureInertiaScale = 0.25`) plus
STAND-only downward height feedback (capped at 20 mm; walking stays sag-only)
is the kept pair: 60 s stand **6.42 mm** error, 12000 healthy; WAVE / slow-fwd
height / tripod / v16 multi-rate / `aggressive_governor` pass. Isolated
walk-distance translation cases then pass with **zero held samples**.

Do not raise plant `L`, μ, or dense ADMM. Do not recapture v16.

### Turn-in-place `cmd_yaw` construction (2026-09-15)

Named leftover after armature: yaw was correct, but walk-distance
`turn_in_place` translated **0.556 m** against the **0.21 m** gate.

Census: the test filled only `twist.twist_vel_radps.z = 0.45` and left
`cmd_yaw_radps = 0`. `planarMotionCommand` then copies `twist.z`, and
`rawLocomotionTwistFromIntent` adds it again, so the body twist is **0.90
rad/s** while gait is planned at **0.45**. Walk-entry seeding also uses
`intent.cmd_yaw`, so the turn looked planar and took `kFirstStridePhaseSeed`
instead of yaw-dominant Φ=0 (the v16 seed path). Regression
`turn_in_place` already used `ScenarioMotionIntent` yaw_rate 0.45.

**Lever (kept).** Construct walk-distance turn on `cmd_yaw_radps = 0.45` and
leave `twist.z = 0`, same as scenario/replay. Scoring uses
`planarMotionCommand`. A `locomotion_command` unit test locks single-count
yaw. No gait, solver, friction, or gate change.

Isolated remesure (`HEXAPOD_WALK_TEST_CASE`):

| Case | Result |
| --- | --- |
| `forward_walk` | **pass**, held 0, ratio 0.48 |
| `slow_forward_walk` | **pass**, held 0, ratio 0.90 |
| `reverse_walk` | **pass**, held 0, ratio 0.53 |
| `straight_walk` | **pass**, held 0, lateral 0.089 m |
| `turn_in_place` | **pass** 4/4, `raw_wz=0.45`, net **0.176–0.187 m**, yaw Δ ~2.14 rad, held 0 |

The sequential five-case CTest can still drop a later case on one recovered
NCP `HeldLastGood` in a shared sim process. Census of that class is in
[sequential NCP hold census](#sequential-ncp-hold-census-2026-09-15).
`./scripts/verify.sh` remains deferred until walk-distance is green.

### Sequential NCP hold census (2026-09-15)

Named leftover after `cmd_yaw` turn construction: the sequential five-case
CTest still maps one `HeldLastGood` to stay-WALK. Hypothesis was shared-process
warm-start contamination, because isolated cases had passed and regression
already respawns per case.

**Sequential** (`HEXAPOD_WALK_TEST_CASE` unset, 5 runs): **0/5 pass**.

| Run | First fail | Held / reason | Latch |
| --- | --- | --- | --- |
| 1 | `turn_in_place` net | reverse held 0 | turn after reverse+straight in the same plant |
| 2 | `reverse_walk` stay-WALK | 2 NCP | cold 48, 5 contacts, `ncp_dual=0` / `ncp_comp=0.0039` |
| 3 | `reverse_walk` stay-WALK | 1 NCP | cold 48, 5 contacts, `ncp_dual=0.010` |
| 4 | `reverse_walk` stay-WALK | 1 NCP | cold 48, 5 contacts, `ncp_dual=0.006` |
| 5 | `turn_in_place` net | reverse held 0 | same as run 1 |

**Isolated:** reverse 2/3 pass held 0, **1/3 NCP hold** at step 377
(`ncp_dual=0.010`, 5 contacts). Straight 3/3 pass. Turn 3/3 pass, net
0.180–0.186 m.

**Traces** (isolated reverse, `HEXAPOD_PROXIMAL_TRACE_FAILURES`): 2/5 held.
`[proximal-held]` fails on **complementarity**, not dual. Example: first-attempt
`ncp_dual=3e-4` (under 1e-3) / `ncp_comp=0.00237`; cold 48 `ncp_dual=0` /
`ncp_comp=0.00269`. Worst contact hops (13↔4). Peak joint error ~0.63 rad.
`retry_max_link_w=0` (NCP returns before the speed guard). Last-resort 2×
absolute (2e-3) does not accept 0.0027 complementarity. Stuck 5-contact set.

**Split: isolated stuck NCP**, not sequential-only. Isolated reverse already
holds. Per-case sim respawn would hide sequential turn-drift after a long
shared plant, but it would not close isolated reverse. Do not respawn as a
way to skip that hold.

**Lever (not applied).** Stop. Do not raise cap 24, μ, `L`, dense ADMM, or the
last-resort NCP floor. Next named class is isolated reverse 5-contact
complementarity (cold `ncp_comp` ~0.0025–0.0027), not harness isolation.
`./scripts/verify.sh` was not re-run.

### Isolated reverse NCP complementarity (2026-09-15)

Named leftover after sequential census: isolated `reverse_walk` stay-WALK on
one `HeldLastGood`. `[proximal-held-contacts]` census (`id:leg:joint:comp:dual:cone:pen`):

- duplicates **0**, unique robot joints **5**, legs `{0,1,2,3,5}` joints
  `{4,7,10,13,19}` (five unique tibias; missing leg 4). Not same-tibia
  multi-point and not a non-foot extra.
- Complementarity offender is **always contact 13** (leg 3, joint 13,
  penetration ~0.05–0.1 mm). `worstContactId` is dual/cone on a different
  contact (`4`/`7`/`10`/`19`). Split 2 evidenced: retries were erasing the
  dual/cone row, not the complementarity row.
- Cold 48 already `resetWarmStarts()`, so last-resort-only erase of contact 13
  is a no-op. After cold 48, either contact-13 `ncp_comp` stays ~0.0026–0.004
  or complementarity recovers and another contact's dual exceeds 2e-3.

**Lever (reverted).** Skip-insert / `clearFailedSolverState` used
`worstComplementarityContactId` instead of dual/cone `worstContactId` on NCP
reject. Isolated reverse remesure **4/8 pass** (holds still cold-48 dual on
contact 7 after complementarity on 13 recovers, or both still over 2e-3). The
wrong-victim split is real; erasing the complementarity row does not make
last-resort accept the 5-contact set. Reverted. Do not raise last-resort NCP
above 2×, cap 24, μ, or `L`. Census print kept.

`./scripts/verify.sh` was not re-run.

### Last-resort 4-contact omit (2026-09-15)

Named leftover after complementarity-victim erase: last-resort still **assembles
all five unique tibias**. Warm-start skip ≠ omitting the row.

**Census** (non-integrating `[proximal-held-omit]` after 5-contact cold 48
rejects, 2× NCP floor): three NCP holds. Omit complementarity-worst always
accepted (4 unique joints, `ncp_dual` ~3e-4–1.1e-3, `ncp_comp` ~1e-5). Omit
dual/cone-worst also accepted. Split 1.

**Lever (reverted).** Last-resort cold 48 omitted complementarity-worst when
unique joints ≥ 5. Isolated reverse remesure **5/8 pass**. Fails: one 4-contact
NCP (`ncp_dual=0.00271`, below the ≥5 gate) and two **initiating SpeedLimit**
leftover-ω cascades (tibia/femur swing, `max_link_w` 10–12, streaks 782–1423).
Forward 5/5, slow/straight/turn pass. Omitting the row recovers some 5-contact
NCP then dumps unconstrained tibia speed into the SpeedLimit class. Reverted.
Omit-id plumbing and reject probes kept. Do not raise 2×, drop a second
contact, cap 24, μ, or `L`.

`./scripts/verify.sh` was not re-run.

### Last-resort quarter-steps (2026-09-15)

Named leftover after 4-contact omit: last-resort still two cold `dt/2` 48-iter
steps at 2× NCP, all contacts. A 5-contact (or 4-contact almost-feasible) set
can sit over 2e-3 at `dt/2`. Question: does one cold 48 at **`dt/4`** land
under 2× on the same snapshot without dropping a contact?

**Census** (non-integrating `[proximal-held-quarter]` after last-resort reject,
same 2× floor, omit-id 0): five isolated `reverse_walk` NCP holds. **9/10**
probes accepted (dual/comp under 2× plus relative). **1/10** rejected on
complementarity `0.00211` (5 unique joints). Split 1 on the snapshot.

**Lever (reverted).** Replaced the two last-resort `dt/2` NCP steps with four
integrating `dt/4` cold 48 steps (SpeedLimit first-fails kept the `dt/2` pair).
Isolated remesure:

| Gate | Result |
| --- | --- |
| Isolated `reverse_walk` ×8 | **7/8 pass.** Remaining hold: 5-contact NCP `ncp_dual=0.00869` / `ncp_comp=5e-5`, streak 1 |
| Isolated `forward_walk` ×5 | **3/5 pass.** One leftover-ω tibia SpeedLimit cascade (held 334, swing tibia 333, `peak_pre_w=10.1`, `max_link_w=10.7`) after 88 NCP RecoveredRetry; one 5-contact NCP `ncp_dual=0.00371` / `ncp_comp=0.00171` |
| Isolated slow-fwd | **pass** |
| Isolated straight | **fail** stay-WALK: 2 NCP holds, `ncp_dual=0.00386` / `ncp_comp=0.00188`, 5 contacts |
| Isolated turn | **pass**, net **0.181 m** |

Forward leftover-ω and the straight stay-WALK miss are the revert criteria.
Production last-resort is again two cold `dt/2` steps at 2×. `[proximal-held-quarter]`
probe kept. Sequential CTest, plant screens, and `./scripts/verify.sh` were not
run on the rejected lever. Do not restore omit, do not raise 2×, cap 24, μ, or
`L`. Do not publish `HeldLastGood` as `bus_ok`.

Named class remains isolated reverse 5-contact NCP (dual ~0.006–0.009 or
complementarity ~0.0026–0.0039). Next lever is not a smaller last-resort `dt`.

`./scripts/verify.sh` was not re-run.

### Last-resort 96-iter (2026-09-15)

Named leftover after dt/4 revert: last-resort still two cold `dt/2` **48-iter**
steps at 2× NCP, all contacts, and already hits the 48-iter cap. Question: is
the leftover residual **iteration-limited**, or a hard 5-contact set at
production last-resort `dt/2`?

**Census** (non-integrating `[proximal-held-iters]` after last-resort 48 reject,
same 2× floor, `dt/2`, omit-id 0): four isolated reverse NCP holds, eight
probes. Complementarity always recovered (`ncp_comp` ~1e-5–1e-4). Dual vs 2e-3:

| Probe | `ncp_dual` | vs 2e-3 | iterations |
| --- | --- | --- | --- |
| 1 | 0.00232 | over | 76 |
| 2 | 0.00393 | over | 60 |
| 3 | 0.00130 | under | 62 |
| 4 | 0.00158 | under | 60 |
| 5 | 0.00060 | under | 73 |
| 6 | 0.00675 | over | 81 |
| 7 | 0.00216 | over | 82 |
| 8 | 0.00025 | under | 54 |

**4/8 still over 2e-3 on dual** (up to 0.00675). `accept=1` on all eight is ADMM
`converged ||` physical, not dual-and-comp under 2×. Split 2.

**Lever (not applied).** Stop. Do not raise last-resort to 96 or 192. Do not
restore omit or `dt/4`. Do not raise 2×, first-attempt cap 24, μ, `L`, or dense
ADMM. `[proximal-held-iters]` probe kept. Sequential CTest, plant screens, and
`./scripts/verify.sh` were not run.

Named class remains isolated reverse 5-contact NCP at `dt/2` (hard set, not
iteration-limited). Next lever is not more last-resort ADMM iterations.

`./scripts/verify.sh` was not re-run.

### Last-resort graze filter (2026-09-15)

Named leftover after 96-iter stop: five unique tibias at last-resort `dt/2`,
usually including 15–80 µm extras plus complementarity offender contact 13 at
~0.1 mm. Question: if last-resort does not assemble those grazes, does a
3-loaded-foot set land under 2× **keeping joint 13**, with dropped rows at ~0
dual/comp?

**Census** (non-integrating `[proximal-held-graze]` after last-resort 48 reject,
same 2×, `dt/2`, omit-id 0): five isolated reverse NCP holds. Dropped column is
`id:leg:pen:dual:comp` from the failed 5-contact retry.

| Hold | 50 µm | 100 µm |
| --- | --- | --- |
| 11a | unique 3, under 2×, kept 13; dropped 10/19 dual=0 | same (same extras) |
| 11b | dropped nothing, still 5, dual 0.014 | unique 4, **dropped 13** (77 µm) |
| 14 | unique 4; dropped dual-worst 7 (dual 0.001) | unique 2, **dropped 13**, dual 0.004 still over |
| 15 | dropped nothing, still 5 | unique 4, dual 0.00324 still over |
| 17 | unique 4; dropped dual-worst 10 (dual 0.002) | unique 3 under 2× kept 13, but dropped complementarity offender (leg 4) and dual-worst |

Split 2. 50 µm often drops nothing or an **impulse-carrying** row. 100 µm can
filter joint 13. The one 3-foot under-2× probe that kept 13 still dropped a
loaded dual/comp row. Not unused grazes.

**Lever (not applied).** Stop. Do not last-resort-filter grazes. Do not restore
omit-13, `dt/4`, or 96. Do not raise 2×, cap 24, μ, `L`, or retune gait / 1.85× /
stall. `minPenetration` plumbing and `[proximal-held-graze]` probes kept
(production last-resort still unfiltered). Isolated reverse stay-WALK is a
**Class A residual**: hard 5-contact NCP at production `dt/2`. Sequential CTest
and `./scripts/verify.sh` were not run.

`./scripts/verify.sh` was not re-run.

### Tilt path-before-rate-trip (2026-09-15)

Named leftover after reverse Class A: [`tilt_safety_trip`](../hexapod-server/tests/test_locomotion_regression_suite.cpp)
must accumulate `path_length_m > 0.1` before `TIP_OVER`. Envelope stays
`max_tilt_rad=0.25`, `rapid_body_rate_radps=0.45`, `rapid_body_rate_max_contacts=4`.
Command: TRIPOD `vx=0.45` at heading `π/2`, body height 0.12 m.

**Census** (isolated `--case tilt_safety_trip` ×5, production proximal, no plant
change):

| Run | total path | pre-fault path | net | walk samples | mean meas vx | first_fault_step | trip gyro rate | trip supp (raw/est) | roll |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0.074 | 0.017 | 0.013 | 90 | 0.038 | 490 | 1.28 | 3/4 | 0.11 |
| 2 | 0.052 | 0.015 | 0.009 | 89 | 0.035 | 489 | 1.26 | 4/4 | 0.10 |
| 3 | 0.074 | 0.016 | 0.008 | 91 | 0.035 | 491 | 1.20 | 2/4 | 0.11 |
| 4 | 0.044 | 0.014 | 0.007 | 85 | 0.034 | 485 | 1.14 | 4/4 | 0.10 |
| 5 | 0.080 | 0.015 | 0.007 | 90 | 0.035 | 490 | 1.22 | 4/4 | 0.11 |

All five: `TIP_OVER`, `bus_ok=true`, solver first-failed **none** (not a hold).
Roll ~0.10–0.11 **below** 0.25, so the **rate** rule trips. `max(raw,est)` drops
to 4 on that frame (`>4` would mask the rule). For 8–10 walk frames before the
trip, gyro rate is already **>0.45** but 5–6 contacts keep the rule off. Mean
measured planar speed ~0.035 m/s vs commanded 0.45 (governor still ~0.44).
Pre-fault travel ~15 mm; 0.45 s × 0.45 m/s commanded ≈ 0.20 m. Stride count 0.

Split 2 + 3. Rate trip is honest. Translation is not on track to 0.10 m
(would need ~2.4 s more at 0.035 m/s). Do not loosen 0.10 m path or 0.45 rad/s.
Closing it would be gait / collection / μ / `L` / 1.85×-in-WALK, all forbidden.

**Lever (not applied).** Document as the second **Class A residual** alongside
isolated reverse 5-contact NCP. Canonical `aggressive_governor` remains pass.
`./scripts/verify.sh` was not re-run.

### Tripod support commanded foot tracking (2026-09-14)

Named gate: `physics_sim_tripod_support_baseline` (open-loop `PhysicsSimBridge`
hold, no runtime/gait/height hold). Tracking is body-frame FK(command joints)
vs FK(measured joints) on support legs `{1,2,5}`. Gate **45 mm**.

Census (proximal cap 24, 0.14 m): six-leg stand **0.149 m**, tracking **16 mm**.
Tripod height 0.115 m (not collapse). Tracking **94 mm** (L1 50 / L2 94 / L5
54 mm). First 120 ms **91 mm**, after **94 mm**. Pre-IK vs post-IK support
residual **0**. Support joint *commands* vs six-leg stand **0**. Joint error vs
command **0.52 rad**; drift from first measured **0.12 rad** (inside 0.14 rad).
PGS at 0.14 m collapses (stand ~0.033 m), so the 94 mm miss is proximal loaded
tracking, not a solver-mode exclusive.

Four-way: **not window**, **not command/IK**, **not one-leg-only** (all three
support feet exceed 45 mm). **Servo/plant:** the same support joint command that
tracks to 16 mm on six legs yields ~94 mm on three. Stop. Do not raise stall,
μ, ADMM, or loosen 45 mm. 45 mm still means max over the full metrics hold.

Follow-up census resolved the servo/plant split. Peak servo torque utilisation
was only **15.5%** despite R2 settling with about **0.50 rad** tibia error, so the
MG996R torque-speed envelope was not saturated. The six-foot reflected-inertia
calibration was under-stiff once support was reduced. A global gain was rejected:
1.75× broke stand and the 120/240 Hz frozen replays. The production correction is
therefore limited to a sustained static three-leg command: it latches a
six-foot command reference, requires exactly three leg command groups to differ,
requires fewer than six contacting tibias and unchanged targets for **250 ms**, then
ramps the effective gain to **1.85×** over **100 ms**. Moving commands and ordinary
six-foot stand remain at 1.0×; stall torque, μ, ADMM and the 45 mm gate are unchanged.

This also exposed that the old weak hold was not a genuine tripod: its metrics
window contained five contacts for 509/720 samples and four for 174. With the
correction, the raised feet unload and seed 0 has three contacts for 720/720
samples. Contact-order seeds 0–4 all pass, with worst support tracking **39.4 mm**,
joint drift below **0.034 rad**, and body height about **0.142–0.143 m**. The
60-second proximal stand and frozen 200/120/240/480 Hz replays all remain green.

## Summary

The remaining [`plan.md`](plan.md) blocks are **evidence**, not missing solver
architecture. Close them by measuring the existing harnesses, classifying the
failure, then changing only the lever that failure implies. Do not start with
resource Phases 1–3 or ADMM default flips.

The WSL default **is** `pinocchio-proximal`: `Runtime.PhysicsSim.SolverMode = 1`
in [`hexapod-server/config.physics-sim-wsl.txt`](../hexapod-server/config.physics-sim-wsl.txt)
(parser default `1`, WSL `SolverIterations` 24). Proximal stand CTest and v16
replay are the live physics gates (now default CTests at seed 0 and 120/240/480 Hz).
verify.sh live-physics CTests are proximal 0.14 / cap 24; **2 remain Class A**.
100-seed stays a diagnostic override. Dense ADMM and resource Phases 1–3 stay
follow-ups.

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
| Full physics/server/gait suites | `./scripts/verify.sh` plus locomotion binaries | Live-physics CTests are proximal 0.14 / cap 24. **2 Class A** closed-loop misses remain (walk-distance; regression aggressive recover + tilt path-before-TIP_OVER). Recover census: HeldLastGood SpeedLimit; 0.5 retry leaves swing-femur ω ~14. Do not stack 0.25, skip link ω, retune gait, extend 1.85× into WALK, or loosen 10 mm |

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
- Making `./scripts/verify.sh` green by excluding the five remaining Class A
  closed-loop CTests. See [verify.sh inventory](#verifysh-inventory-2026-09-14).

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

Retry policy is already correct (NCP same-dt + half-step at full gain;
SpeedLimit retries at 0.5 PD gain, still under the all-body 10 rad/s cap).
Trace with `HEXAPOD_PROXIMAL_TRACE_FAILURES=1` and
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
verify.sh live-physics CTests were retargeted to proximal 0.14 / cap 24;
Class A closed-loop misses remain (see [verify.sh inventory](#verifysh-inventory-2026-09-14)).

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
verify.sh live-physics CTests are proximal 0.14 / cap 24; two Class A
closed-loop misses remain. See
[Forward cross-track investigation](#forward-cross-track-investigation-2026-09-14),
[100-seed held investigation](#100-seed-held-investigation-2026-09-14),
[120 Hz cadence investigation](#120-hz-cadence-investigation-2026-09-14),
and [verify.sh inventory](#verifysh-inventory-2026-09-14).

### Scoped last-resort spectral experiment (2026-09-15)

A scoped change that set the ADMM spectral power initialisation to `0.4` only
for the two cold last-resort half-steps was tested against the production
initialisation (`0.2`). It improved an isolated reverse sample set, but regressed
the shared sequential campaign: forward accumulated HeldLastGood speed/NCP
cascades and turn-in-place lost its stay-WALK or drift gate. The change was
removed and the production spectral setting restored.

A restored-default baseline then passed forward, slow-forward, reverse, and
straight in one sequential run; turn passed in a focused ten-run sample but
remains intermittent in the shared campaign. A focused forward trace reproduced
the same underlying class: a five-contact cold NCP hold (`ncp_dual` about
`0.0067`, `ncp_comp` about `0.0033` in one run) with all pre-integration
angular speeds below the 10 rad/s guard. No solver-mode, friction, iteration,
safety-envelope, or gait-limit change is justified by this experiment.

### Deterministic regression clock and canonical recensus (2026-09-16)

The locomotion regression runner refreshed command timestamps from wall time
while scoring a fixed 5 ms cadence. Contact-solver runtime therefore changed
gait phase and apparent travel. Refreshed phases now use a synthetic clock that
advances exactly one configured sample period per control step. The deliberately
stale command-timeout scenario continues to use wall time.

This closes two false residuals without changing production physics or gates:

- compliant canonical turn yaw increased from about 0.2 rad to 2.62 rad; rigid
  reached 2.39 rad;
- `tilt_safety_trip` now travels 0.141 m before the intended TIP_OVER, clearing
  the unchanged 0.10 m path gate.

The timeout scenario was also restored from a historical 0.20 m pose to the
production 0.14 m height. At 0.20 m, a 2.36 m/s link-frame speed violation
pre-empted the zero-motion freshness test. At 0.14 m, rigid and compliant modes
both reach COMMAND_TIMEOUT with no held samples.

For `low_support_walk`, the previous 90.7 mm maximum was sample 4 of STAND
settling, not WALK. The all-phase metric remains reported, and a new walk-only
tracking metric drives this walk gate. WALK maxima are 56.4 mm compliant and
45.8 mm rigid against the unchanged 80 mm limit.

All seven canonical cases now pass with the compliant experiment enabled.
Rigid production remains slightly variable on turn translation: one run reached
207 mm against the unchanged 200 mm bound, while three immediate repeats were
191--199 mm. The deterministic long-walk stress case remains red in both modes
because a linear/angular SpeedLimit hold becomes BUS_TIMEOUT; it is separate
from rigid NCP convergence.
