# Coordinated swing-rate governor

Date: 2026-09-16
Status: mapping and opt-in prototype implemented; acceptance screen remains red.
Integrated-state guard kept. Reverse SpeedLimit census: no production lever.
Sequential turn command census: no production lever. Commanded-tibia SpeedLimit
hunt: no production lever (captured schema-2 is v2 contact-amplified).

## Objective and boundaries

Test whether correlated swing-joint requests can avoid excessive composed link
angular velocity without damaging locomotion. Contact can still amplify velocity;
this governor is not a replacement for pre-integration safety validation.

Rigid Pinocchio remains the WSL default (mode 1, cap 24). Mode 2 remains opt-in.
Preserve the dirty worktree, frozen v16 and contact fixtures, all existing gates,
the 2 m/s and 10 rad/s physical guards, actuator limits, PD gains, contact model,
static latch, and fault handling. No runtime pruning or silent solver fallback.

## 1. Prove the rotational mapping

- Derive coxa, femur, and tibia angular velocities from existing server geometry,
  measured configuration, calibration signs, and chassis body-frame angular rate.
- Compare every leg/wire against Pinocchio link velocities for 1,000 randomized
  configurations and velocities, including opposing pitch rates and chassis motion.
- Explicitly audit the improper sim/server basis and axial-vector convention;
  position-FK axes must not be assumed to be bridge angular-velocity axes.
- Require double-precision agreement within 1e-9 absolute/relative tolerance.
- Add one complete kinematic capture when necessary: q, v, chassis angular vector,
  outgoing rates, predicted link rates, and contact velocity increment. Existing
  SpeedLimit v2 lacks enough vector state for a complete reconstruction. Do not
  overwrite existing snapshots or hunt many more failures before implementation.

## 2. Implement a direction-preserving governor

After the existing independent joint slew clamp, let r be the outgoing three-joint
rate vector. For each planned WALK swing leg, choose the largest s in [0,1] such
that all three link rates satisfy:

```text
norm(omega_body + s * A_link(q) * r) <= command_angular_budget
next_target = previous_target + s * requested_increment
```

Use exact quadratic feasible-interval intersection with deterministic bounded
cost. A common scale preserves pitch cancellation and joint-rate ratios. Do not
snap targets to measured positions or independently clip another joint.

Start with a 10 rad/s command budget. Keep physical guards unchanged. Require
valid measured joints and body angular rate; leave stance, STAND, and recovery
unchanged. Infeasible or unavailable input retains the baseline path and reports
the condition, rather than fabricating braking. Activation is explicit opt-in.

## 3. Diagnostics and focused tests

Record requested/governed rates, scale, limiting link, predicted rates before and
after, tracking error, pre-ABA/free/post-contact velocity, contact delta, and
activation/unavailable/infeasible counts. Prefer test-local instrumentation.

Test cancellation, reinforcing rates, chassis rotation, calibration, timestep
changes, invalid input, and exact inactive-path equivalence. No new solver mode
or production wire-protocol change is needed for this experiment.

## 4. Early screens

Compare baseline and experiment using the same build/settings:

| Screen | Gate |
| --- | --- |
| canonical aggressive_governor, 3 runs | Existing 2 strides; zero held |
| isolated straight, 3 runs | Existing direction/distance gates |
| isolated turn-in-place, 5 runs | Existing yaw/translation gates |
| rigid sequential walk-distance, 5 runs | 5 complete passes; zero held/SpeedLimit |

Straight and turn precede longer campaigns: prior following-error variants
improved translation but damaged turning. NCP holds remain a separate class;
replacing SpeedLimit failures with NCP holds is not a successful resolution.

## 5. Bounded decision

Only if the nominal governor reduces unsafe requests but contact still trips the
guard, compare command budgets 9 and 8 rad/s. These restrict commands, never raise
the physical guard. Reject any setting that damages aggressive, straight, or turn.

Reject this approach if it rarely activates on failing steps, PD backlog still
dominates free velocity, contact supplies the unsafe increment despite shaping,
or established gates regress. Do not progressively slow commands or reopen
pruning/iteration/cap increases. Record a plant-coupling follow-up instead.

## 6. Established-green regression campaign

After early screens pass: WAVE and slow-forward height, static tripod tracking,
stand 60 s, v16 at 200/120/240/480 Hz, and controller/mapping/physics tests. Then
100 seeds and a 10-minute soak. Full verify only after rigid sequential is green.

## Deliverables and experiment record

Rotational helper and Pinocchio oracle; opt-in governor and unit tests; narrow
diagnostics; machine-readable comparison report. For each experiment record date,
HEAD/dirty tree, fixture checksum, solver/settings/budget, active bounds, residuals,
peak velocities/contact delta/energy, gate results, and keep/reject decision.

The batch ends with a validated governor or a documented rejection isolating the
remaining plant coupling. No gate is loosened and mode 2 is not promoted.

## Progress — 2026-09-16

### Implemented and verified

- Plan saved before implementation. Existing dirty campaign preserved.
- Bridge-convention rotational helper: 1,000 randomized poses, 18,000 per-link
  comparisons, worst Pinocchio angular error 1.58882e-14 rad/s. A second pass
  writes projected velocities through Pinocchio and checks all 18 link bounds.
- Deterministic common-scale projection and simulator-only runtime opt-in:
  `HEXAPOD_SWING_LINK_RATE_EXPERIMENT=1`; command budget defaults to 10. Stance,
  STAND, hardware, and the simple simulator remain baseline. Bridge convention and
  simulated measurement provenance are both checked.
  A default-false bridge capability is forwarded by physics test decorators;
  a dynamic type check would silently disable the experiment in those wrappers.
- Unit tests cover cancellation/reinforcement, maximality, boundary/base rates,
  randomized projection, calibration/mirroring, cadence, previous-target anchoring,
  inactive paths, missing measurements, hardware provenance, and failed bus.
- A/B runner: `tools/run_swing_link_rate_screen.py`. Explicit rigid mode/cap,
  binary/fixture checksums, settings, projection counts, JSON metrics and logs.
- Fixed test-harness output contamination: enabled simulator child diagnostics
  now go to stderr; the runner captures stdout/stderr separately and rejects
  metric parse failures rather than silently treating incomplete reports as valid.
  The runner also requires positive governor execution evidence for opt-in runs
  and cleans up its own test/simulator process group on timeout. Final capability
  smoke: baseline and actual opt-in aggressive both pass, two strides/no holds;
  opt-in records 356 limited and 1,273 unchanged evaluations, not a silent no-op.
  Checksums/results: `contact-snapshots/coordinated-swing-rate-capability-smoke-v1.json`.
- Seven Pinocchio model/audit/census/sweep/oracle CTests and the governor plus
  motion/IK/FK unit tests pass. No production solver/settings/gates were changed.

### Screens and decision

Clean-stream machine-readable results are frozen in
[`contact-snapshots/coordinated-swing-rate-screen-v1.json`](contact-snapshots/coordinated-swing-rate-screen-v1.json).

| Screen | Baseline | Governor, 10 rad/s |
| --- | --- | --- |
| aggressive, initial screen | 3/3 pass | 2/3; one NCP BUS_TIMEOUT, 2 strides in all |
| aggressive, clean-stream repeat | not repeated | 3/3, 2 strides, zero holds |
| isolated straight, initial screen | 2/3; NCP holds | 2/3; NCP hold |
| isolated turn, initial screen | not repeated | 5/5 |
| sequential, initial screen | 0/5 | 2/5; other runs turn drift, no holds |
| sequential, clean-stream repeat | 1/5 | 1/5 |

The final governor sequential failures are reverse NCP (2 held), forward NCP
(2), straight NCP (5), and straight SpeedLimit (381). Baseline failures are
reverse NCP (1), turn drift (zero held), reverse SpeedLimit (244), and straight
NCP (1). Small live samples do not establish an improvement or a regression rate.
They establish that this prototype does not close the existing gates.

**Decision: not production-ready.** Retain only the explicit opt-in prototype for
reproduction. No default activation, 8/9 rad/s selection, green-test campaign,
100-seed/soak, fixture recapture, or full verify is justified yet.

### Newly isolated limitation: requested cancellation is not actual cancellation

The initiating governor-run-5 trip is L3 tibia, not the earlier rigid-v2 femur
trip. The last projection was active on that leg, with femur/tibia requested
servo rates -7.48/+7.48 and common scale 0.85035. Predicted tibia angular rate was
6.976 rad/s; the actual guard saw 10.0445 rad/s.

The physical wire pitch rates still reinforce:

| | Femur | Tibia | Sum |
| --- | --- | --- | --- |
| incoming | 3.62076 | 3.25503 | 6.87579 |
| free motion | 3.59717 | 3.32376 | 6.92093 |
| after contact | 3.45922 | 3.46046 | 6.91968 |

Tracking errors are still 0.278/0.213 rad. The requested reversal/cancellation
has not propagated through the actuator plant. Contact pitch increments almost
cancel in this frame; do not reuse the earlier contact-amplified-femur diagnosis
as the explanation for this tibia trip. Full base angular vectors are absent,
so a complete link/contact decomposition is not claimed.

Next lever, if reopened: a tracking-aware rotational reference governor that
predicts measured actuator response/braking before assuming cancellation. First
freeze full q/v, base angular vectors and requested rates for this initiating
state, then test a predictor against Pinocchio/servo response. A lower request
budget alone is not a demonstrated solution. Keep NCP convergence and turn drift
separately classified, and retain aggressive-governor in every remesure.

## New-clue investigation — 2026-09-16

Scope: diagnosis and test-local observation only. No production gain, torque,
contact equation, cap, target path, validity handling or default changed.
HEAD remains `055940da3a5bb7bc3983e5b120fa1258e25b32fe`, with the existing dirty
campaign plus diagnostic additions. This is not a claimed locomotion fix.

### 1. Requested cancellation is not imposed motor velocity

The actuator law uses position error and measured joint velocity. The outgoing
target velocity is not a velocity constraint or a velocity feedforward term in
that law. The 7.48 rad/s number defines the motor torque-speed envelope, not a
hard joint-velocity limit. Opposing target increments therefore do not make
existing, same-sign physical pitch rates cancel immediately.

An isolated critically damped position-PD ramp model using the original L3
errors/rates reproduces this limitation: equal-and-opposite target rates leave
the sum response unchanged when both receive a common scale. At one control
tick (5 ms) the actual sum remains above 6 rad/s, and at 100 ms above 1 rad/s,
for scales 0.25, 0.5, 0.85035 and 1. This is a diagnostic model, not a prediction
of the coupled, clipped and contact-loaded robot. It explains why simply
screening 8/9 rad/s is not a demonstrated remedy for stored tracking error.

### 2. Independent full-state capture separates free dynamics from contact

The original L3 tibia event was not recaptured. A new governed sequential run
instead captured an initiating R3 femur trip (leg 0); do not merge their signs
or winners. The schema-2 capture contains q, all velocities/torques, mass,
nonlinear/gravity terms, wire mapping, effective inertia, angular Jacobians and
incoming/free/post-contact vectors.

Frozen artifacts:

- `contact-snapshots/speed-limit-governed-kinematics-v1.json`
- `contact-snapshots/speed-limit-governed-kinematics-audit-v1.json`
- `contact-snapshots/speed-limit-kinematics-screen-v1.json`

Fixture SHA-256: `d24c77aeaf7f7611bbd42d1c4f6c6599b0babf908329c80744fc7519d9e687ce`.
The stored report recomputes exactly. Existing NCP snapshots and v16 are untouched.

| R3 femur WORLD_ALIGNED speed | rad/s |
| --- | ---: |
| Incoming | 10.001821464 |
| Unconstrained/free | 10.147139890 |
| After contact | 10.018896382 |

Contact reduces the winner's norm here. It is not the source of this step's
first increase. Dense `M^-1 (tau - h)` agrees with captured ABA acceleration to
`5.3433e-12` absolute infinity-norm error; all stored `J_angular v` vectors agree.
No ABA algebra or angular-frame mismatch is indicated by this fixture.

The femur error is +0.583 rad; torque +0.0539 N*m assists its +6.61 rad/s rate.
The tibia error is -0.0696 rad; its rate is only -0.228 rad/s despite an opposing
target ramp. Their requested pitch rates sum to zero but incoming physical
rates sum to +6.383 rad/s. This independent capture is not the original event
where both pitch motors were braking.

The femur's diagonal torque/inertia estimate is +33.88 rad/s^2. The full inverse
mass own-motor contribution is +127.72, other motors -4.49 and Coriolis +9.59,
giving +132.82 rad/s^2 free acceleration. Thus a scalar servo estimate materially
underestimates the coupled response. These are acceleration components at fixed
q, not a complete integrated-energy or contact prediction. Uniform gravity has
negligible free angular contribution in this floating-base snapshot.

### 3. Guard gap at pose integration — closed

Production previously validated link velocities at `(q_old, v_after)` before
integration, then wrote `(q_new, v_after)` without recomputing the bound. The
angular Jacobian depends on q, so that old-pose check is not a bound at the new
pose.

A deterministic real-rig kinematic regression starts all links below 10 rad/s,
with legal individual rates (coxa magnitude 5, each pitch about 4.30 rad/s) and
chassis angular speed 1 rad/s. At dt = 1/600 s, manifold angular integration with
unchanged generalized velocity raises the tibia from 9.999999 to 10.007164954
rad/s. That invented state is not a production contact solve.

The production path now validates the proposed integrated pose before any body
is written (`writeValidatedState`). Unsafe candidates keep the existing
SpeedLimit retry/rollback policy; there is no velocity clamp. Unsafe or
non-finite external corrections restore the previous validated last-good state
and cannot replace it. Caps remain 2 m/s and 10 rad/s. The kinematic regression
rejects that candidate and leaves the robot unchanged.

### 4. Bounded publication screen and checks

An opt-in test-wrapper audit reconstructs angular speeds from successful bridge
reads at the received pose. It does not change bus_ok, validity or targets. A
5-run governed sequential screen recorded zero received over-limit samples and
zero initiating SpeedLimit snapshots: 2 complete passes, 2 turn-drift failures,
and 1 forward NCP hold. No SpeedLimit event occurred, so this screen cannot
establish the frequency or live provenance of the guard gap.
Record: `contact-snapshots/published-link-speed-screen-v1.json`.

Nine Pinocchio CTests and two server governor/motion-FK tests pass. The new
Python response/validator suite has four tests, including six malformed-fixture
variants. The model snapshot test checks the new diagnostic schema. Full verify,
promotion, long green-gate campaigns and parameter selection were not run.

### Recommended next implementation, not applied in this diagnosis

The first two items below were applied in the follow-up implementation record.
They remain here as the diagnosis-time recommendation.

1. Preserve the existing pre-integration guard and also validate the proposed
   integrated state before writing/publishing or updating last-good state. Audit
   external-correction and restoration boundaries too. Keep the 2/10 caps and
   invalid-sample handling unchanged; test rollback and accepted-state invariants.
2. Then evaluate a tracking-aware reference predictor using measured q/v,
   tracking error and full coupled `M^-1`, rather than instantaneous target rates
   or only diagonal inertia. Validate predicted free response against frozen ABA
   state before considering a command change. Contact prediction remains separate.
3. Any actuator/reference experiment must retain common-direction intent without
   snapping targets, and remeasure canonical aggressive (two strides), straight,
   turn and sequential before expanding to established-green regressions.

Fixing the guard invariant may expose rather than eliminate additional holds;
that is not permission to relax the cap. NCP convergence and turn drift remain
separate residuals. The current governor stays explicit opt-in, not validated.

## Integrated-state guard and predictor — 2026-09-16

HEAD remains `055940da3a5bb7bc3983e5b120fa1258e25b32fe`. SolverMode=1, 24
iterations, 0.14 m, last-resort 2×, caps 2/10. Mode 2 is not promoted. The
opt-in swing-rate governor is not the default. Full `./scripts/verify.sh` is
not a production-green claim while rigid sequential walk-distance is red.

### Guard implementation

`PinocchioHexapodModel::writeValidatedState` checks the proposed q/v against
every WORLD_ALIGNED body bound before writing. `stepProximal` uses it after
`pinocchio::integrate`. `synchronizeAfterExternalCorrection` restores last-good
when the raw world or reconstructed state is unsafe; a rejected correction
cannot become the rollback baseline. Raw `writeState` remains available for
mapping oracles and fault injection.

Focused tests: `test_pinocchio_hexapod_model` (rejected candidates leave q/v
and last-good unchanged), `test_pinocchio_link_angular_velocity` (transport
candidate rejected as Tibia SpeedLimit; world left at the legal old pose).

### Predictor evaluation (test-local)

`tools/predict_speed_limit_response.py` reproduces the frozen governed R3 femur
capture: torque error 1.11e-16, free-velocity error 9.33e-15. Common-scale
probes 1/0.75/0.5/0.25/0 on the winner leg keep other references unchanged.
Free max angular speed falls from 10.147 to 9.997 at scale 0; every candidate
has `physical_gates_passed=false`. The fixed-contact probe reuses the captured
delta; it does not re-solve contact. Record:
`contact-snapshots/coupled-speed-predictor-v1.json`.

**Decision: do not change commands from this predictor.** It is an offline
oracle, not a runtime governor.

### Guard remesure (governor off unless noted)

Machine-readable reports:
[`contact-snapshots/integrated-state-guard-and-predictor-v1.json`](contact-snapshots/integrated-state-guard-and-predictor-v1.json).

| Screen | Result | Class |
| --- | --- | --- |
| aggressive, baseline | 3/3, 2 strides, 0 holds | pass |
| turn, baseline | 5/5 | pass |
| sequential, baseline | 2/5 | reverse NCP and/or old-pose SpeedLimit |
| straight, baseline | 0/3 | SpeedLimit 708; recovered NCP stay-WALK; NCP hold=1 |
| aggressive, governor 10 | 2/3 | one NCP BUS_TIMEOUT (1 hold, 0 SpeedLimit, 2 strides) |

Zero `[proximal-integrated-speed-limit]` traces in these screens. Live
SpeedLimit events that did fire were the original pre-integration guard. Small
samples do not establish a live-rate change from the invariant fix.

**Guard decision: keep.** Caps, retry, and HeldLastGood handling are unchanged.
Straight 0/3 and sequential 2/5 do not justify reverting a rejected-state
write, and they are not permission to raise 10 rad/s.

**Governor decision: still not production-ready.** Retain explicit opt-in only.

Do not expand to WAVE/slow-forward/tripod/stand/v16 or full verify while
sequential remains short of 5/5. Remaining live leftovers are NCP convergence
and turn drift; commanded-rate coupling still explains old-pose SpeedLimit.
A command-side predictor would still need a contact re-solve and those
locomotion gates before it is a candidate.

## Reverse initiating SpeedLimit census — 2026-09-16

HEAD remains `055940da3a5bb7bc3983e5b120fa1258e25b32fe`. SolverMode=1, cap 24,
0.14 m, governor off, caps 2/10. `--screen reverse` added to
`tools/run_swing_link_rate_screen.py`. Existing SpeedLimit, NCP, and v16
fixtures were not overwritten. Schema-2 files
`speed-limit-reverse-v1.json` / `speed-limit-straight-v1.json` were not written
because this hunt produced no initiating snapshot.

### Hunt

| Screen | Result | SpeedLimit snapshots |
| --- | --- | ---: |
| isolated reverse ×5 | 5/5 pass, recovered NCP only, peak ω 7.82–9.07 | 0 |
| sequential ×5 | 1/5 | 0 |
| isolated straight ×3 | 3/3 | 0 |

Sequential failures: turn net 0.231 / 0.210 / 0.214 m vs 0.21 (zero held), and
one reverse NCP hold (5 contacts, dual 0.007279, complementarity 0.002408, 48
last-resort iters, `max_link_w` 2.77). That NCP sample is the known reverse
Class A family after cold retries. `reverse-failure-v3.json` stays the frozen
first-reject fixture; it was not recaptured.

### Classification without a new freeze

The earlier integrated-guard sequential run 4 still supplies the only
full-gain reverse SpeedLimit trace (`leg_4_tibia_body`, swing, `winner_w`
10.151, chassis 0.646, NCP dual 6.7e-4, 4 contacts, then 1255 holds). Joint
rates: coxa vin −7.07, femur 4.78, tibia 1.49; contact Δv +0.04 / −0.16 /
+0.17. Composed `|vin|` sum 13.34. Contact does not dominate. That is the
same commanded/composed swing-tibia family as `speed-limit-rigid-v1.json`,
not v2 contact-amplified femur/tibia (+4.89 / −9.69) and not a new
recovery-slew or stance mis-tag.

**Decision: no production lever.** Phase 2 skipped. Governor remains explicit
opt-in. Predictor stays test-local. Mode 2 is not promoted.

Record: [`contact-snapshots/speed-limit-reverse-census-v1.json`](contact-snapshots/speed-limit-reverse-census-v1.json).

## Sequential turn vs 0.21 m — 2026-09-16

Walk-distance turn JSON now records command construction, first WALK-frame
governor planar request, stand-end body velocity, equivalent radius, and
path-per-radian. The 0.21 m net gate is unchanged. Prefix screens
`turn-after-reverse` and `turn-after-reverse-straight` share one plant.

### Census (Mode 1, cap 24, 0.14 m, governor off)

| Screen | Result | Turns that ran |
| --- | --- | --- |
| isolated turn ×5 | 5/5 | net 0.168–0.187 m, radius 0.095–0.108 m |
| turn after reverse ×3 | 2/3 | 0.193 / 0.187 m; one reverse SpeedLimit abort |
| turn after reverse+straight ×3 | 1/3 | 0.174 m; reverse/straight NCP aborts |
| sequential ×5 | 1/5 | the one turn that ran was 0.188 m, radius 0.106 m |

Every scored turn: `cmd_yaw`/`raw_wz` 0.45, raw planar 0, `yaw_dominant` true,
first-walk requested planar 0, Φ≈0 (`phase[0]` 0.002). Held 0. Historical
sequential nets 0.210–0.231 m did not reproduce. Failures this screen were
reverse NCP and one forward swing-tibia SpeedLimit, not a remaining command
construction bug.

**Decision: no production lever.** Do not lengthen STAND, respawn between
cases, retune gait/`walk_entry_blend_s`, loosen 0.21 m, or reuse anti-windup.

Record: [`contact-snapshots/turn-sequential-census-v1.json`](contact-snapshots/turn-sequential-census-v1.json).

## Commanded swing-tibia SpeedLimit hunt — 2026-09-16

`--screen slow` added. Isolated `slow_forward` ×5 and aggressive ×5 captured
nothing. Sequential ×5 with `--capture-speed-limit` froze one schema-2
snapshot on forward_walk run 4. Existing v1/v2/governed/reverse/v16 fixtures
were not overwritten.

### Classification

Auditor replay of
[`contact-snapshots/speed-limit-sequential-contact-amplified-v1.json`](contact-snapshots/speed-limit-sequential-contact-amplified-v1.json):
winner `leg_0_femur_body`, `speed_in` 2.90 → `speed_after` 11.21,
`incoming_over_cap` empty, composed `|vin|` 1.80, femur/tibia contact Δv
+12.15 / −15.06, NCP accepted (dual 3.6e-4, 4 contacts, 11 iters),
`recovery_slew_active` false. That is the v2 contact-amplified family, not
commanded composed swing-tibia. Do not restack SpeedLimit retry gain.

Turn-census sequential run 3 still supplies a commanded-tibia TRACE
(`leg_0_tibia_body`, `winner_w` 10.13) without a schema-2 freeze. No new
unused lever.

**Decision: no production lever.** Measured-q/v reference probe skipped.
Governor remains explicit opt-in. Predictor stays test-local. Mode 2 is not
promoted.

Record: [`contact-snapshots/speed-limit-commanded-tibia-census-v1.json`](contact-snapshots/speed-limit-commanded-tibia-census-v1.json).
