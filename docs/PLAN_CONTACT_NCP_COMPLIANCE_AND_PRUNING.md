# Plan: Redundant Frictional Contact and Compliance Investigation

Date: 2026-09-15  
Owner: physics-sim / hexapod-server  
Status: Investigation plan. Last-resort NCP CCP recovery is in production Mode 1;
session Mode 2 is still not promoted. 
Related: [`PLAN_PINOCCHIO_DEFAULT_SWITCH.md`](PLAN_PINOCCHIO_DEFAULT_SWITCH.md), [`PLAN_PINOCCHIO_ADMM_DELASSUS_RETRIES.md`](PLAN_PINOCCHIO_ADMM_DELASSUS_RETRIES.md), [`SEQUENTIAL_WALK_DISTANCE_LEFTOVERS.md`](SEQUENTIAL_WALK_DISTANCE_LEFTOVERS.md)

## Purpose

Record and resolve the intermittent rigid-contact failures that remain after the
Pinocchio proximal solver became the WSL default. The named symptom is a
`HeldLastGood` sample during locomotion when a five-contact frictional NCP does
not meet the production residual floor. The broader question is whether the
production model should remain a rigid Coulomb NCP, use principled contact
selection, or add an explicit compliant-contact mode.

This plan is deliberately separate from the default-switch plan. It does not
change the WSL solver default, loosen a gate, or authorize a silent Mode 2
session fallback. Logged last-resort cone-QP recovery after a rigid NCP miss
is the authorized leftover for 5-contact holds.

## Current status

| Item | State |
| --- | --- |
| WSL default | `pinocchio-proximal`, 0.14 m body height, solver cap 24 |
| Remaining Class A issues | Mix recorded in [`rigid-flake-census-v1.json`](contact-snapshots/rigid-flake-census-v1.json): sequential NCP hold, sequential SpeedLimit hold, isolated reverse NCP hold, turn translation vs 0.20/0.21 m, tilt path-before-rate |
| Stand / replay / WAVE / slow-fwd / tripod | Green on the recorded baselines |
| Production contact model | Rigid point contacts, Coulomb cones, Pinocchio `DelassusOperatorRigidBody` + proximal ADMM |
| Last-resort policy | Two cold `dt/2` steps, 2x iteration cap, all contacts retained; then logged cone-QP recovery on NCP miss |
| Experiments already rejected | Global/scoped spectral changes, contact omission, graze filtering, `dt/4`, higher last-resort iterations, contact-friction ramps |
| Production changes in this plan | Last-resort NCP CCP recovery (2026-09-16); Mode 2 still not promoted |

## Evidence and working diagnosis

The first-held snapshots show five unique tibia contacts, often including shallow
15–80 µm contacts. Typical rejected snapshots retain approximately
`ncp_dual = 0.002–0.007` and `ncp_comp = 0.0026–0.0039` after cold retries.

The Delassus diagnostics are positive in the sampled failures: eigenvalue minima
are roughly `0.31–0.53`, maxima roughly `60`, and estimated condition numbers
roughly `113–191`. This rules out a simple zero-rank or NaN mass-matrix failure.
It does not rule out non-unique admissible impulses, active-set changes, or poor
finite-iteration convergence of the frictional NCP.

Smaller substeps improve some frozen snapshots but can move the residual into
leftover distal-joint velocity. Removing a contact can make the NCP pass while
allowing the omitted tibia to acquire excessive angular velocity. Therefore,
penetration depth alone is not a safe contact-selection rule.

The literature supports this interpretation:

- Rigid contact can be overconstrained and force-indeterminate even without
  friction; Coulomb friction adds difficult, sometimes non-existent or
  multiple-solution NCPs. See [Drake contact modeling](https://drake.mit.edu/doxygen_cxx/group__drake__contacts.html).
- Carpentier, Montaut, and Le Lidec describe proximal regularisation as making
  the intermediate problem well-defined; the proximal effect cancels only as
  iterations progress. See [From Compliant to Rigid Contact Simulation](https://arxiv.org/abs/2405.17020).
- Anitescu/Hart time-stepping bounds constraint infeasibility with timestep size,
  but the hard-contact subproblem remains difficult. See [Constraint-Stabilized
  Time-Stepping](https://www.mcs.anl.gov/papers/P1002.pdf).
- Zapolsky and Drumwright document force indeterminacy and possible torque
  chatter in rigid-contact control. See [Inverse Dynamics with Rigid Contact and Friction](https://positronicslab.github.io/assets/pdfs/inverse-dynamics.pdf).
- MuJoCo and Drake use explicitly softened/convex contact formulations to trade
  exact rigidity for better-posed optimization and robustness. See [MuJoCo's
  contact model](https://github.com/google-deepmind/mujoco/blob/main/doc/overview.rst)
  and [Drake discrete contact models](https://drake.mit.edu/doxygen_cxx/group__mbp__discrete.html).

## Questions this plan must answer

1. Is the failure caused by a genuinely redundant contact set, or by a bad
   contact Jacobian / frame / inertia mapping?
2. Is the rigid NCP infeasible at the sampled state, or merely not solved to the
   configured finite-iteration acceptance floor?
3. Can a principled contact basis preserve the required chassis wrench without
   producing distal-joint speed cascades?
4. Does an explicit compliant convex model improve locomotion without hiding
   penetration, energy, or tracking errors?
5. Which model should be production default: rigid proximal, compliant contact,
   or a documented mode-dependent combination?

## Non-goals and frozen constraints

Do not, on this evidence alone:

- raise the healthy cap, last-resort multiplier, proximal parameter, friction,
  stall torque, or plant gains;
- publish `HeldLastGood` as a valid bus sample or lengthen the bus timeout;
- omit a contact solely because it has small penetration or a particular ID;
- extend the static reduced-support gain into moving gait;
- loosen the walk-distance, height, tilt-path, rate, penetration, or energy gates;
- recapture the frozen v16 replay fixture;
- silently fall back between rigid and compliant equations on a healthy first
  attempt (logged last-resort CCP after a rigid NCP miss is the leftover).

Every experiment below is opt-in, reproducible, and compared with the unchanged
production baseline.

## Phase 0 — Snapshot and algebra audit

**Goal:** separate model/mapping errors from contact-model limitations.

### Work

- Add a deterministic frozen-snapshot artifact containing `q`, `v`, applied
  servo torques, contact IDs, contact poses/normals, penetration, friction,
  `J`, `G`, free velocity, and the production solver settings.
- Recompute the Delassus matrix with the dense oracle and compare it with
  `DelassusOperatorRigidBody` elementwise and in generalized-velocity effect.
- Report SVD rank, singular values, eigenvalues, symmetry error, and condition
  estimates for both the full generalized contact map and the chassis wrench
  map.
- Verify that every contact row maps to the intended tibia, frame, normal, and
  protocol leg index.
- Replay the snapshot without collision detection, controller timing, or UDP.

### Exit criteria

- Dense and articulated Delassus agree within the existing oracle tolerance.
- No frame, sign, inertia, or contact-ID mismatch is found.
- The snapshot reproduces the production accept/reject decision.

A failure here is a model bug and blocks all contact-selection or compliance
experiments.

## Phase 1 — Contact-basis census (diagnostic only)

**Goal:** determine whether a smaller support basis can represent the same
physical load without arbitrary omission.

### Candidate sets

Evaluate, without integrating production state:

1. all detected contacts;
2. contacts with persistent IDs and meaningful normal impulse;
3. a rank-revealing basis selected from the chassis wrench map;
4. a basis selected from the full articulated `J`/`G` map;
5. the three-foot support set used by the gait controller, when available.

For each candidate, solve with the dense oracle and record normal/friction cone
feasibility, chassis wrench error, generalized velocity delta, energy delta, and
which contacts carry impulse. A candidate is not valid merely because ADMM
converges: it must preserve the required wrench and not create distal velocity.

### Hysteresis rules for any future implementation

- A new contact must persist for a configured dwell before entering a rigid basis.
- A load-bearing contact cannot be removed on penetration alone.
- Contact removal must preserve wrench feasibility and pass a velocity bound.
- Basis changes must be logged by manifold ID and never occur silently inside a
  substep.

### Exit criteria

A candidate basis must reproduce the dense all-contact chassis wrench within
`1e-6` relative error, keep penetration within the existing gate, and avoid any
new speed-limit or energy failure across 100 deterministic seeds. Otherwise it
remains diagnostic only.

## Phase 2 — Frozen rigid-vs-compliant comparison

**Goal:** test whether compliance resolves the sampled failure class without
changing locomotion code.

Implement an opt-in solver mode in a separate branch or feature flag. It must
use the same Pinocchio state, collision contacts, servo torques, and telemetry.
It must not be selected automatically after a rigid solve fails.

### Candidate compliant formulations

- Normal Kelvin–Voigt / Hunt–Crossley contact with regularised tangential
  friction and an implicit or convex velocity solve.
- A MuJoCo/SAP-style convex contact approximation with explicit stiffness or
  relaxation-time parameters.
- Pinocchio proximal compliance only as a control experiment; distinguish it
  from physical contact compliance because its regularisation is numerical and
  iteration-dependent.

### Required telemetry

Penetration and penetration rate per contact, normal/friction force, contact
work, total mechanical energy, solver residual, contact-set changes, chassis
tracking, stance-foot drift, and distal joint speed. Report the compliance
parameters with every result.

### Exit criteria

The compliant mode must pass a frozen-snapshot suite and then demonstrate:

- no held samples in 100 deterministic gait seeds;
- penetration below 3 mm;
- no speed-limit or non-finite-state violations;
- body-height and stance-drift gates unchanged;
- energy error bounded and explainable by the selected dissipation;
- p99 step time below 4 ms at 240 Hz.

If it passes only by allowing unacceptable penetration, slip, or energy creation,
it is not a replacement.

## Phase 3 — Locomotion A/B campaign

Run the unchanged production mode and each candidate mode on the same captured
command streams:

- stand 60 s;
- forward, reverse, slow-forward, straight, and turn-in-place;
- WAVE and tripod support;
- exact replay v16 at 120/200/240/480 Hz;
- 100 perturbed initial-pose/contact-order seeds;
- ten-minute randomized feasible-gait soak.

Collect pass/fail plus distributions, not only averages. A mode that improves
reverse while regressing turn, stand, or cadence is rejected.

## Phase 4 — Decision and rollout

Choose one of three outcomes:

### A. Keep rigid proximal contact

Use this if the snapshot/oracle work shows the residual is rare and bounded,
contact selection is not robust, and compliant contact introduces larger physical
errors. Record the reverse NCP as a known Class A limitation and improve
observability only.

### B. Add principled contact selection

Use this only if rank/wrench-preserving basis selection is deterministic,
contact-hysteretic, and passes all gates. It becomes an explicit solver mode,
never an error-path omission.

### C. Add compliant-contact production mode

Use this if the compliant solver passes the complete acceptance suite and has
calibratable physical parameters. Keep rigid proximal as a comparison mode for
at least one release; do not switch defaults until the full campaign is green.

## Acceptance gates

All existing gates remain in force. Additional gates for this plan are:

| Gate | Requirement |
| --- | --- |
| Algebra | Dense and articulated `G` agree within existing oracle tolerance |
| Contact feasibility | Non-negative normal forces and valid friction cones |
| Redundancy | No unexplained contact-force chatter or basis thrashing |
| Safety | Zero non-finite, speed-limit, held-state, or unsupported-island samples in the acceptance campaign |
| Contact geometry | Penetration < 3 mm; no contact teleportation |
| Locomotion | At least 70% commanded translation; turn translation and yaw gates unchanged |
| Height/stability | Existing ±10 mm height, tilt, foot-drift, and stance gates unchanged |
| Energy | No unexplained energy creation; dissipation reported per contact |
| Performance | Physics-step p99 < 4 ms at 240 Hz |
| Determinism | Equivalent outcomes at 120/200/240/480 Hz within existing tolerances |

## Experiment log

Append one entry per run or code change:

```text
Date:
Commit / tree state:
Mode and parameters:
Fixture / command stream:
Contact set and selection rule:
Result:
Worst residual / contact ID:
Peak speeds and energy delta:
Gates affected:
Decision: keep / reject / follow-up
```

## Current next action

Phase 0 and the captured reverse/turn fixture audits are complete. Dense candidate re-solves and the fixture-only compliant prototype are now recorded below; simulator integration remains gated on a passing frozen candidate. Keep production contact selection and solver mode unchanged.

## Implementation record — Phase 0/1 first capture (2026-09-15)

The first diagnostic milestone is implemented without changing the rigid production solver:

- The optional contact snapshot environment setting captures the first solver rejection as versioned JSON. The snapshot contains q, v, servo torques, timestep/settings, contact IDs and geometry, J, articulated/dense/Cholesky Delassus matrices, wrench map, impulses, and solver residuals. Capture is one-shot and diagnostic-only.
- The capture path refreshes Pinocchio CRBA data before evaluating the Cholesky oracle; this avoids comparing the oracle against stale mass data.
- Captured fixture: docs/contact-snapshots/reverse-failure-v1.json (five contacts, rejected at the rigid ADMM iteration budget). The independent dense matrix agrees with the articulated matrix to approximately 1e-13; Cholesky agrees within the configured 1e-10 regularisation. This closes the Phase 0 mapping/algebra check for this state.
- Offline census: docs/contact-snapshots/reverse-failure-v1.census.json. The all-contact chassis-wrench rank is 4 and the articulated Jacobian rank is 15. Four contacts carry non-negligible frozen impulse. The deterministic four-contact chassis-wrench basis has relative wrench error 8.27e-4, above the 1e-6 runtime-selection gate. Therefore no basis is enabled in production; the result is retained as a diagnostic rejection and motivates compliant-contact experiments only after additional frozen-state replay coverage.

The fixture is deliberately separate from the frozen v16 exact-replay fixture. No solver mode, contact selection rule, gate, or fallback behavior was changed by this work.

### Additional Phase 1 captures

- turn-failure-v2.json captures a five-contact rejected turn-in-place state. Its dense/articulated Delassus difference is about 4e-14 and the Cholesky difference is about 1e-10. The four-contact wrench basis has relative frozen-wrench error 9.96e-3, so it also fails the 1e-6 preservation gate.
- turn-failure-v2.census.json confirms that the articulated basis must retain all five contacts for zero frozen-wrench error; the smaller wrench basis is not equivalent. This is further evidence against runtime pruning based only on penetration or impulse magnitude.
- Forward walk did not reject during the capture run, so no forward fixture was created; the capture trigger remains available for the next intermittent failure.

## Implementation record — replayable audit, census, and compliant fixture prototype (2026-09-15)

The reusable Phase 0/1/2 diagnostic tooling is now checked into tools/ and wired
into the Pinocchio CTest configuration:

- tools/finalize_contact_snapshot.py adds tree/protocol revision metadata and a stable SHA-256 checksum to raw captures before validation.
- tools/contact_snapshot_audit.py validates schema version, required fields,
  finite values, dimensions, duplicate IDs, symmetry, and dense/articulated/
  Cholesky Delassus agreement. The reverse and turn fixtures both pass.
- tools/contact_basis_census.py performs deterministic all-contact, non-negligible
  impulse, chassis-wrench-basis, and articulated-Jacobian-basis candidate solves
  using the frozen dense Delassus matrix. It reports cone, complementarity,
  residual, impulse, work, velocity, rank, and wrench-preservation metrics. The
  reduced chassis-wrench candidates fail the 1e-6 relative wrench gate on both
  captured failures, so runtime selection remains disabled.
- tools/compliant_contact_sweep.py is a test-local SAP-like projected convex
  contact prototype. It sweeps 27 bounded compliance/damping/tangential
  regularisation combinations per fixture and records penetration, contact work,
  energy proxy, projected-gradient KKT residuals, iterations, and impulses.
  Both captured failures meet the frozen physical gates (projected residual
  approximately 7.3e-7 and 7.6e-7; sub-millimetre penetration; dissipative
  energy proxy), so the candidate is eligible for simulator-only A/B testing.
  Raw rigid-NCP dual residual is intentionally not used as the compliant gate.
- pinocchio_hexapod.cpp now has an explicit test-only
  HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT environment path. It applies
  a regularized dense contact solve with cone projection and direct warm-start
  initialization, but does not change the wire protocol or production default.
  The experiment accepts bounded test-only overrides through
  HEXAPOD_PINOCCHIO_COMPLIANT_NORMAL_COMPLIANCE,
  HEXAPOD_PINOCCHIO_COMPLIANT_TANGENTIAL_REGULARIZATION, and
  HEXAPOD_PINOCCHIO_COMPLIANT_ITERATIONS (defaults 1e-5, 1e-6, and 256).
- The tools are registered as test_pinocchio_contact_snapshot_audit,
  test_pinocchio_contact_basis_census, test_pinocchio_compliant_contact_sweep,
  and test_pinocchio_contact_snapshot_negative_cases. The negative cases cover
  duplicate IDs, checksum tampering, and contact-order permutation handling.
  Together with the existing model tests, all targeted CTests pass.

Artifacts:

docs/contact-snapshots/audit-report.json
docs/contact-snapshots/census-report-v1.json
docs/contact-snapshots/compliance-sweep-v1.json
docs/contact-snapshots/compliant-sim-campaign-v1.json

Decision: keep the rigid Pinocchio proximal solver as the production/default
mode. Contact pruning remains diagnostic-only. The compliant path is now an
explicit simulator-only experiment and must not be enabled through WSL config or
server protocol until the complete A/B campaign passes.

Initial simulator A/B results with the experiment flag are encouraging: the
60-second stand, full walk-distance sequence, WAVE height, slow-forward height,
tripod-support, exact replay, and canonical aggressive-governor checks produced
zero held samples and no speed-limit faults. Stand p99 experiment iterations are
1 and exact-replay p99 step time is approximately 2.65 ms. This is not yet a
production decision: cadence and 100-seed health checks are green, but the
long-walk stress expectation remains open. The ten-minute randomized
feasible-gait soak, explicit compliant energy telemetry, and
protocol/configuration rollout checks remain required. tilt_safety_trip remains
separate.

Follow-up clock audit (2026-09-16) found that the regression-suite motion runner
was refreshing intent timestamps from wall time while treating every sample as
a fixed 5 ms control step. Solver speed therefore changed gait phase, command
filtering, and the apparent distance/yaw accumulated in a nominally deterministic
test. Refreshed motion phases now receive a fixed-cadence synthetic timestamp;
the deliberately stale command-timeout scenario remains on wall time.

This removes the apparent compliant turn shortfall without changing physics or
the 0.25 rad gate: compliant turn-in-place now reaches 2.62 rad and rigid
Pinocchio reaches 2.39 rad, both with no fault. The previous 0.19--0.24 rad
tangential-regularisation comparison was clock-confounded and is retained only
as historical evidence against tuning the solver from that test.

The deterministic long-walk test also replaces the earlier apparent 0.50 m
safe-stall result. It now reaches 3.41 m in compliant mode before a SpeedLimit
HeldLastGood/BUS_TIMEOUT at step 3451; rigid mode reaches 2.08 m before the same
fault class at step 4103. This is a genuine remaining speed-limit/recovery issue,
not rigid NCP non-convergence and not a reason to promote compliance.

The command-timeout scenario was still using a historical 0.20 m stand pose even
though the production physics campaign uses 0.14 m. At 0.20 m, the nominally
zero-motion freshness test generated a 2.36 m/s femur-frame linear-speed trip
before the timeout. Keeping the scenario at the production 0.14 m height makes
both rigid and compliant modes reach COMMAND_TIMEOUT cleanly with no held
sample; the timeout policy and gates are unchanged.

The low-support 80 mm tracking failure was also a window mismatch. Its 90.7 mm
maximum occurred at sample 4 during initial STAND settling. During WALK, the
maximum is 56.4 mm in compliant mode and 45.8 mm in rigid mode. The regression
metrics now retain the all-phase maximum and separately report
max_walk_contact_tracking_error_m; only the low-support walk gate uses the walk
window. The 80 mm limit is unchanged.

With those corrections, all seven canonical cases pass in compliant experiment
mode. Rigid production passes six consistently; turn-in-place produced one
207 mm translation result against the unchanged 200 mm gate, while three
immediate repeats passed at 191--199 mm. That narrow rigid turn variability
remains a contact-indeterminacy follow-up rather than grounds for loosening the
gate.

Speed-limit tracing now ranks the reported frame by normalized linear/angular
limit severity and includes both winning speeds. This fixes misleading traces
where an angularly fast frame was reported even though a different frame's
linear speed caused rejection; guard thresholds and acceptance behaviour are
unchanged.

A coordinated per-leg target-rate prototype bounded the L1 sum of the three
joint target rates by one motor no-load-speed envelope. It removed every held
sample from the deterministic 60-second compliant stress run and allowed 2.39 m
of travel, confirming that composed serial-link rate is the SpeedLimit source.
It was rejected and reverted because canonical aggressive_governor dropped from
two strides to one. A blanket coordinated cap is therefore too restrictive;
any future swing shaping must be local to the approaching speed envelope and
must preserve the established cadence/stride gates.

Validation after the simulator rebuild: all six Pinocchio audit/model/sweep
CTest cases pass, Python tools compile cleanly, and git diff --check is clean.

## Implementation record — live compliant acceptance (2026-09-16)

The live experiment had two defects relative to the frozen-fixture evidence:

1. It capped projected iterations at 20 and then marked every iterate converged,
   which on a two-contact transition injected hundreds of rad/s.
2. It stored rigid ADMM contact velocity next to the replacement compliant
   impulse, so the warm-start and the applied wrench disagreed.

The current test-only path therefore:

- starts projected-gradient from the already-bounded rigid ADMM impulse, not
  from an unconstrained LDLT solve;
- uses a 256-iteration ceiling with early exit at a 1e-3 live projected
  residual (the frozen 1e-5 KKT gate rejected walking steps whose NCP dual
  was already ~1e-6);
- accepts a step only when the impulse is finite, the peak impulse is at most
  1.0 N·s (walking peaks ~0.06–0.10), and either the live residual or the
  production NCP floor is met;
- refuses unconverged iterates and does not warm-start from them.

Results with `HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT=1`
(see [`compliant-sim-campaign-v2.json`](contact-snapshots/compliant-sim-campaign-v2.json)):

- Sequential walk-distance 3/3, every case stay-WALK, zero held samples, zero
  speed-limit faults, peak normal impulse 0.08–0.10 N·s.
- Isolated reverse under the old 1e-5 KKT gate failed with 1732 held
  `SolverNotConverged` samples; the same reverse passes after the live
  accept rule.
- Canonical isolated gait-transition 3/3 and tilt 2/3. One combined canonical
  pass still showed gait tracking 121 mm vs 120 mm and tilt path 91 mm vs
  100 mm; those remain measurement/envelope flakes, not solver holds.
- Long-walk stress 1/3 reached `TIP_OVER` with zero holds; 1/3 finished 12 000
  samples with no safety trip; 1/3 produced a single swing-tibia SpeedLimit
  at 10.16 rad/s (composed serial-link guard, not the previous impulse
  blow-up). This case is not a stable contact-solver screen.
- Production rigid sequential reverse in the same rebuild recovered 108
  NCP rejects with zero holds, then failed turn-in-place at 216 mm vs the
  unchanged 210 mm gate.

Decision unchanged: keep rigid Pinocchio proximal as the WSL/production
default. Compliance stays an explicit opt-in experiment for the **session**.
Do not enable runtime pruning. Do not promote Mode 2 until energy/penetration
telemetry and a non-flaky long-walk health metric exist. Five-contact
hyperstatic NCP holds are addressed in production by last-resort cone-QP
recovery (path C), not by changing the 2× NCP last-resort pair or the WSL
solver default.

## Implementation record — contact-health gates, telemetry, rigid census (2026-09-16)

Long-walk was split into two jobs. `long_walk_observability` remains
`--profile stress` and still expects a late `TIP_OVER`/`BODY_COLLAPSE`.
`long_walk_contact_health` is canonical: it shares a moderate 25 s soak
(no 0.6 m/s slam), requires a stay-WALK window and path length, and fails
on `HeldLastGood`/`BUS_TIMEOUT` caused by `SolverNotConverged` or non-finite
impulse/velocity, or on peak normal impulse above 1.0 N·s. It does not
require a safety fault. SpeedLimit is counted, not treated as a contact-solver
fail.

Walk-distance and locomotion JSON now emit peak impulse, max/p99 penetration,
max |energy delta|, actuator work, and max/p99 compliant projected residual.
`StateResponse` appends `solver_compliant_projected_residual` (0 on rigid).
`ProximalStepDiagnostics::compliantProjectedResidual` is filled on the
experiment path and on protocol `SolverMode = 2`.

Experiment results (`HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT=1`):

- `long_walk_contact_health` 3/3, zero holds, peak impulse 0.057 N·s,
  max penetration 0.44–0.48 mm, p99 projected residual 0.001 (live gate,
  not exploding).
- Sequential walk-distance 3/3, zero holds. Aggregates:
  p99 projected residual 0.0009998, max penetration 0.93 mm, peak impulse
  0.105 N·s, max |energy delta| 0.045. See
  [`compliant-walk-distance-p99-v1.json`](contact-snapshots/compliant-walk-distance-p99-v1.json).

Rigid production census, experiment unset, fixed 5 ms clock, proximal 24 /
0.14 m ([`rigid-flake-census-v1.json`](contact-snapshots/rigid-flake-census-v1.json)):

| Screen | Pass |
| --- | --- |
| Sequential walk-distance ×5 | 1/5 (NCP hold, swing SpeedLimit, turn net 0.226 m vs 0.21 m) |
| Isolated reverse ×5 | 3/5; two NCP holds at 5 contacts, dual ~0.0049–0.0050, comp ~0.0016–0.0028 |
| Canonical turn-in-place ×5 | 2/5 vs 0.20 m (3/5 would pass 0.21 m); nets 0.192–0.221 m |
| Canonical tilt_safety_trip ×5 | 4/5; one path-before-rate flake (0.053 m vs 0.10 m, rate 0.99 vs 0.45) |

Production Class A is still a mix, not a single reverse-NCP class. Last-resort,
omit/graze/`dt/4`, and gates were not changed. Runtime pruning stays off
(reduced bases still fail the 1e-6 wrench gate). `SolverMode = 2` was not
added in that census batch. `./scripts/verify.sh` is not a production-green
claim while sequential walk-distance remains red without the experiment flag.

Stop condition for a later explicit non-default protocol mode: contact-health
3/3 on the experiment path (met), sequential walk-distance 3/3 with bounded
impulse/penetration/energy on the experiment path (met), projected residual
p99 reported and bounded (met), rigid census recorded (met). Promotion still
requires sequential walk-distance green **without** the experiment flag.

## Implementation record — census NCP fixtures and SolverMode=2 (2026-09-16)

New frozen snapshots from the current rigid plant (experiment unset, proximal
24 / 0.14 m, first NCP reject at the healthy 24-iter budget):

- [`reverse-failure-v3.json`](contact-snapshots/reverse-failure-v3.json): checksum `3775b172bd8cb780`, six unique tibia contacts, `ncp_dual` 0.0487, dense/articulated 5.0e-14, Cholesky 1.0e-10.
- [`forward-failure-v1.json`](contact-snapshots/forward-failure-v1.json): checksum `3112f0f916ac7291`, six unique tibia contacts, `ncp_dual` 0.0497, dense/articulated 2.8e-14, Cholesky 1.0e-10.

Both pass snapshot audit. Chassis-wrench reduced bases fail the 1e-6 gate
(relative error 0.58 reverse / 0.49 forward), so runtime pruning stays off.
The fixture-only compliant sweep still meets frozen physical gates
(projected residual ~1e-17, penetration 75 µm / 43 µm). SpeedLimit
and turn/tilt flakes remain census-only; they are not Delassus rejects.

`PhysicsSolverMode::PinocchioProximalCompliant = 2` is now a protocol/config
value. Parser bounds are `0..2` with default `1`. WSL
`config.physics-sim-wsl.txt` stays `SolverMode = 1`. Serve mode runs the
existing PGD path when mode is 2; the environment flag remains a local
override. There is no rigid-to-compliant fallback inside a substep.

Protocol remesure (sequential walk-distance ×3, `HEXAPOD_WALK_TEST_SOLVER_MODE=pinocchio-compliant`, experiment unset): **2/3**. Projected residual p99 ~0.001 and zero recovered NCP on every case, so the wire is live. Run 2 failed stay-WALK on `straight_walk` with femur SpeedLimit (peak link 10.53 rad/s, 656 held, impulse 0.10 N·s, penetration 0.74 mm). That is the leftover serial-link class, not an NCP miss and not a reason to promote.

Promotion still requires sequential walk-distance green on rigid mode 1.
`./scripts/verify.sh` is not a production-green claim.

## Next implementation batch — slew/contact coupling (2026-09-16)

Verified the newer handover against source and `speed-limit-rigid-v2.json`.
WSL remains rigid mode 1 / cap 24; mode 2 stays opt-in. Snapshot audit,
offline census, compliant sweep, and negative-fixture CTests pass.

The rate census supports a saturated slew path with large following error,
but the trip also exposes an acceptance shortcut: ADMM reports converged while
the independently recomputed NCP dual is 0.00397 (above the 0.001 floor) and
`ncp_physically_converged=false`. Production currently accepts either result.
This must be isolated before attributing all contact amplification to gait.

Experiment ladder (one lever at a time):

1. **Strict physical acceptance A/B.** Opt-in diagnostic
   `HEXAPOD_PINOCCHIO_STRICT_NCP_ACCEPT=1` requires the existing independently
   recomputed rigid-NCP residuals; default acceptance is unchanged. Measure
   sequential walk-distance, initiating SpeedLimit, NCP retries/holds, and
   canonical aggressive-governor. Reject as a production change if it merely
   trades SpeedLimit for more NCP held samples.
2. **Following-error anti-windup, only if required.** Prototype a measured-state
   target lead governor that stops increasing an already-lagging target, rather
   than a blanket L1 joint-rate sum. Preserve inward/braking requests, stance
   load support, and the individual MG996R torque-speed envelope. Derive and
   record any lead bound from the configured servo response; do not guess a
   global gait gain. Keep it opt-in until aggressive-governor retains two strides.
3. **Contact coupling audit if neither closes the failure.** Compare pre/post
   contact kinetic energy and articulated velocity contribution on accepted
   initiating trips. Check impulse/velocity consistency before changing damping
   or armature. No contact pruning, impulse clipping, or post-ABA velocity clamp.

Keep any candidate only after rigid sequential walk-distance is 5/5 with mode 1
and the compliant experiment unset, aggressive-governor passes unchanged, and
stand 60 s, WAVE/slow-forward height, tripod tracking, and frozen v16 cadences
remain green. Turn/tilt gates and production speed caps remain unchanged.
No full verification-green claim is made while rigid sequential walking is red.

## Implementation record — SpeedLimit initiating-trip census (2026-09-16)

Diagnosis only. Caps stay 2 m/s and 10 rad/s on every body. Retry gain stays 0.5.
No `vNew` clamp, no gait retune, no pruning, no mode-2 promotion. WSL stays
`SolverMode = 1`.

`HEXAPOD_PINOCCHIO_SPEED_LIMIT_SNAPSHOT_PATH` writes the first **full-gain**
SpeedLimit trip (`pd_gain = 1`). It is not a Delassus fixture and does not fire
on NCP reject or on the 0.5-gain retry.

Rigid sequential walk-distance landed
[`speed-limit-rigid-v1.json`](contact-snapshots/speed-limit-rigid-v1.json) during
`slow_forward_walk`: swing tibia `leg_0_tibia_body`, winner_w 10.15 rad/s,
chassis_w 0.53, NCP accepted (dual 8.4e-4, 5 contacts, impulse 0.011 N·s).
Winner-leg `vin` already high (femur 7.07, coxa 4.83 rad/s). ABA added ~3% on
femur. Contact `dv` did not dominate `vnew`. **Class: commanded** composed
serial-link joint rate. That case recovered (held=0); the same sequential run
later failed stay-WALK on `straight_walk`.

Mode 2 (`pinocchio-compliant`, experiment unset) did **not** reproduce an
initiating SpeedLimit in 28 sequential runs, 8 isolated `straight_walk`, and one
traced sequential (0 `[proximal-speed-limit]` lines). Hunt recorded in
[`speed-limit-mode2-v1.json`](contact-snapshots/speed-limit-mode2-v1.json). Prior
mode-2 sequential 2/3 femur/stance hold (max_link_w 10.53, 656 held) stays
telemetry-only: no `vin`/`vaba`/`contact_dv`. Rigid swing vs mode-2 stance mix
is recorded, not explained away.

See [`speed-limit-census-v1.json`](contact-snapshots/speed-limit-census-v1.json).
No production lever. Later work depends on commanded-rate shaping, not raising
the 10 rad/s cap.

## Implementation record — commanded-rate snapshot (2026-09-16)

Diagnosis only. Did not raise `maxAngularSpeed`, revive L1, retune gait, clamp
`vNew`, or promote mode 2. WSL stays `SolverMode = 1`. v1 fixture left in place.

The SpeedLimit snapshot now includes `target_angle`, `raw_server_target`,
`target_rate_radps` (servo `targetVelocity`), `requested_tau` / `available_tau` /
`tau_saturated`, `recovery_slew_active`, `composed_vin_abs_sum`, and
`no_load_speed`.

Rigid recapture [`speed-limit-rigid-v2.json`](contact-snapshots/speed-limit-rigid-v2.json)
on sequential `forward_walk` (attempt 18): swing femur `leg_0_femur_body`,
winner_w 11.39, chassis_w 2.05, then 1281 SpeedLimit holds. Femur/tibia
`target_rate` sit on the per-joint no-load slew (±7.48) with PD error 0.78 / 0.70
rad and `tau_saturated` (available torque 0). `recovery_slew_active` is false.
`composed_vin_abs_sum` is 22.43. **Class: slew_path.** The ω trip itself is
**contact-amplified**: femur `vin` 7.53 → `vnew` 12.39 (`contact_dv` +4.89);
tibia `vin` −8.23 → `vnew` −17.83. Not commanded-trajectory (error not small),
not PD-lag-only (target rate not modest), not composed-only (tibia `|vin|` exceeds
no-load).

Mode 2 budget 5 sequential: 4 pass, 1 `straight_walk` speed-band miss, **0**
SpeedLimit snapshots. Hunt file unchanged.

See [`speed-limit-rate-census-v1.json`](contact-snapshots/speed-limit-rate-census-v1.json).
No L1 cap. Any later rate-shaping change must remesure `aggressive_governor`.
`./scripts/verify.sh` is not a production-green claim.

## Implementation record — slew/contact coupling experiments (2026-09-16)

Verified the handover, then tested four opt-in diagnostic changes on the dirty
tree at `055940d`. Every runtime change below was **rejected and reverted**.
Mode 1 / cap 24 remains production; mode 2 remains opt-in. Speed, turn, tilt,
distance, and stride gates were not changed.

| Experiment | Aggressive governor | Rigid sequential screen | Decision |
| --- | --- | --- | --- |
| Require recomputed physical NCP residuals, not ADMM-stop OR physical | Fail, 0 strides; 859 NCP held samples | Forward failed at step 23; 2332 NCP held samples, no SpeedLimit | Reject: trades amplification risk for a rigid-NCP hold cascade |
| Swing following-error target anti-windup, all joints | Pass, 2 strides | 0/5 full sequences; zero holds/SpeedLimit in all five, but turn drift 0/4 and straight direction 4/5 | Reject: safety benefit distorts coordinated foot motion |
| Same anti-windup only on slew-saturated requests | Pass, 2 strides | 0/3; no SpeedLimit, but NCP holds in 2/3 and straight-direction miss | Reject: does not close rigid walking |
| Same anti-windup only on swing femur/tibia, coxa untouched | Pass, 2 strides | 0/3; turn net 0.418 m in one, 2400 SpeedLimit holds in another, forward NCP hold in third | Reject: distal-only clipping is not a coordinated solution |

The following-error bound was derived from the nominal critically damped
position-servo ramp lag `2*zeta*vmax/omega_n` (about 0.598 rad), not a raised
stall torque or guessed gait gain. Inward requests and stance support were
unchanged. Tests covered outward backlog, inward motion, stance exclusion,
and invalid-measurement exclusion. Passing aggressive-governor is necessary
but demonstrably insufficient; it did not justify keeping any variant.

Machine-readable record:
[`slew-contact-coupling-experiments-v1.json`](contact-snapshots/slew-contact-coupling-experiments-v1.json).
The source was restored through reverse patches, preserving the newer campaign
and mode-2 work. No rejected experiment flag remains in production source.

### Next bounded implementation

Use the articulated **rotational link-velocity map**, not an L1 sum or separate
joint clipping, to constrain outgoing swing target rates before position PD.
Opposing femur/tibia rates may cancel physically; L1 unnecessarily counts both,
while independent following-error caps destroy their cancellation. A candidate
must minimize target-rate distortion subject to every affected link's mapped
angular envelope, include measured chassis angular velocity, retain individual
motor limits, and report the projection/active bound. Never clamp `vNew`.

First validate the map against Pinocchio frame velocities and the initiating
trip; then keep the prototype opt-in and screen aggressive-governor plus rigid
sequential walking. Contact can still amplify a bounded target rate, so a
predicted-rate bound is not a substitute for the existing pre-integration guard.
If it only trades SpeedLimit for NCP/collection failures, reject it too.
Do not promote mode 2 or claim full verification from this batch.

## Implementation record — explicit regression solver selection (2026-09-16)

Found and fixed a test-harness coverage gap while verifying the restored tree:
`test_locomotion_regression_suite` always constructed rigid mode 1 and did not
honor `HEXAPOD_WALK_TEST_SOLVER_MODE`. That environment variable is correctly
handled by `test_physics_sim_walk_distance`; its historical mode-2 reports are
not invalidated. Earlier suite runs with the compliant experiment override
also genuinely exercised the experimental path.

The regression suite now accepts explicit
`--solver-mode pinocchio-proximal|pinocchio-compliant`, defaults to mode 1 /
cap 24, and records `solver_mode` plus `compliant_experiment_override` in
stdout and summary JSON. Unknown modes and missing values are rejected before
launching the simulator. README and testing instructions document the selector
and require the experiment override to be unset for protocol-mode A/B runs.

Verified with the override unset:

- Default mode 1 aggressive-governor: pass, two strides, no holds.
- Explicit mode 2 contact-health: 2/2 pass, 5000 samples / 11 strides each,
  no faults or holds, peak normal impulse 0.057 N·s, max penetration
  0.441--0.444 mm, p99 projected residual 0.001.
- Explicit mode 2 aggressive-governor: pass, two strides, no holds.
- Local override metadata: correctly reports protocol mode 1 plus override
  true; aggressive-governor passes with two strides and nonzero compliant
  residual. This is not mislabelled as protocol mode 2.

After all runtime prototype reverts, both components rebuilt, the motion/IK/FK
unit test passed, all six Pinocchio model/audit/census/sweep tests passed, JSON
records validate, and `git diff --check` is clean. Full verification and
promotion remain blocked by the documented rigid sequential residuals.

### 2026-09-16 — coordinated swing-rate prototype

The next-batch plan and progress are recorded in
[`PLAN_COORDINATED_SWING_RATE_GOVERNOR.md`](PLAN_COORDINATED_SWING_RATE_GOVERNOR.md).
The rotational map agrees with Pinocchio to 1.59e-14 rad/s; the direction-preserving
governor is implemented as simulator-only explicit opt-in. The clean-stream rigid
sequential screen remains 1/5 for both baseline and prototype. It is not retained
as a validated fix or enabled by default.

An initiating L3 tibia trip shows requested pitch cancellation but reinforcing
actual pitch rates: actuator reversal/tracking lag defeats the instantaneous
request-rate assumption. This is not the old contact-amplified femur snapshot.
NCP holds and turn drift remain independently red. Reports are frozen in
`contact-snapshots/coordinated-swing-rate-screen-v1.json`; existing snapshots/v16
and all gates remain unchanged. Full verify and promotion were not run.

### 2026-09-16 — requested-cancellation clue investigated

The full investigation is appended to `PLAN_COORDINATED_SWING_RATE_GOVERNOR.md`.
An independent R3 femur speed capture confirms dense/ABA acceleration agreement
to 5.34e-12 and shows contact reducing, not creating, that step's speed excess.
Position-PD tracking lag and inverse-mass coupling invalidate the instantaneous
requested-rate cancellation assumption. A separate deterministic kinematic test
proves that an old-pose speed check need not bound the integrated pose; its live
causal role is not yet established. No production lever applied. Diagnostic
publication screen: 2/5 complete passes, no SpeedLimit events, remaining turn
drift/NCP hold. Rigid mode 1, opt-in governor/mode 2 and all gates remain unchanged.

### 2026-09-16 — last-resort NCP CCP recovery

Goal: no more 5-contact `HeldLastGood` when a legal cone-QP impulse exists.
Healthy Mode 1 steps still never flip the contact law. After rigid last-resort
(two cold `dt/2` at 2×) still misses Signorini, two further cold `dt/2`
half-steps run the Mode 2 PGD overlay (`forceCompliantContact`). Apply only if
projected residual, 1.0 N·s impulse cap, and 10 rad/s / 2 m/s guards pass.
Status is `RecoveredRetry` with `ncpCcpRecovery`; `[proximal-ncp-ccp-recovery]`
is always logged. Disable with `HEXAPOD_PINOCCHIO_DISABLE_NCP_CCP_RECOVERY=1`.
Does not run on `SpeedLimit` first-fails. WSL stays `SolverMode = 1`. Do not
treat this as Mode 2 promotion. Isolated `reverse_walk` remesure after `-j1`
rebuild: **5/5 pass**, `solver_held=0` on every run, RecoveredRetry NCP 83–99
per run, peak link ω 7.62–9.46 (under the 10 rad/s guard). One sequential
walk-distance process after that rebuild: **pass** (not a production-green
claim). All five cases `ok`, `solver_held=0`, no SpeedLimit frames. Two logged
`[proximal-ncp-ccp-recovery] accept=1` events (5-contact then 3-contact,
projected 8.8e-4 / 9.3e-4, peak impulse 0.015 / 0.024 N·s, `max_link_w` 2.36 /
2.74). Sequential turn `net_horiz=0.1967 m` vs 0.21 m, `yaw_dominant` true,
`cmd_yaw`/`raw_wz` 0.45, `held=0`. Five more sequential processes (same
binary, Mode 1): **1/5 pass**. Combined with the earlier process: **2/6**.
Not production-green. Scored split:

| Run | Result | Class |
| --- | --- | --- |
| prior | pass | turn net 0.197 m |
| 1 | fail turn stay-WALK | `held=2`, net 0.188 m (would pass 0.21). No CCP log → not an NCP last-resort miss |
| 2 | fail reverse | femur SpeedLimit cascade (`max_link_w` 9.87 → 11.49, 1233 swing femur). CCP `accept=0` twice, `reason=10` (SpeedLimit), 4-contact, projected ~1e-3. Guard refused the write |
| 3 | pass | two CCP `accept=1` on 5-contact; turn net 0.207 m |
| 4 | fail turn net | 0.213 m vs 0.21 m, `held=0`, yaw-dominant, cmd 0.45 |
| 5 | fail turn net | 0.211 m vs 0.21 m, `held=0`; two CCP `accept=1` (4- then 5-contact) |

Isolated reverse 5/5 and sequential NCP holds are no longer the dominant sequential red. Remaining sequential leftovers are **turn 0.21 m** (3 of 5 fails this batch) and **SpeedLimit v2 femur**. Do not loosen 0.21 m or 10 rad/s.

### 2026-09-16 — SpeedLimit v2 lambda-scale recovery (reverted)

Goal: a valid solved state when contact Δω trips 10 rad/s but ABA was under
the cap. After the speed guard rejects `vNew`, binary-search
`α ∈ [0.05, 1)` so `v_α = v_ABA + α (vNew − v_ABA)` passes old-pose and
integrated-pose guards, then `writeValidatedState`. Scale stored contact
warm-starts by `α`. Status is `RecoveredRetry`. Always log. Skip when ABA is
already over the cap. Not a `vNew` clamp, not 10→25, not PD 0.5→0.25.

Hunt capture [`speed-limit-lambda-scale-hunt-sequential-r3.json`](contact-snapshots/speed-limit-lambda-scale-hunt-sequential-r3.json)
is ABA-over-cap tibia (`speed_in` 9.69, `speed_free` 10.36, tiny `contact_dv`);
scale must no-op. Implement class remains frozen
[`speed-limit-sequential-contact-amplified-v1.json`](contact-snapshots/speed-limit-sequential-contact-amplified-v1.json)
and sequential reverse femur CCP `accept=0 reason=10`.

**Remesure (Mode 1, `-j1` rebuild):** isolated reverse with λ-scale on was
**4/5 then 4/5** (femur SpeedLimit stay-WALK, `solver_held` 12–902,
`max_link_w` 10.10→10.37). CCP-only baseline was 5/5, `solver_held=0`.
Plan revert trigger fired. Production λ-scale, diagnostic field, disable env,
and always-on log were removed.

After revert: isolated reverse **5/5**, `solver_held=0`, peak ω 7.70–9.72.
Sequential ×5 **1/5**. Reverse in the shared plant 5/5, no SpeedLimit femur
cascade this batch. All four sequential fails are turn 0.21 m
(`net` 0.222 / 0.219 / 0.215 / 0.257, one pass 0.181, `held=0`, `cmd_yaw`
0.45). CCP `accept=1` still fires (seq 4: 2, seq 5: 1). Sequential remains
scored-red on turn; do not treat `./scripts/verify.sh` as production-green.
Turn 0.21 m is out of this leftover.

### 2026-09-16 — Sequential turn plant-state census (no production lever)

CCP-plant recensus on **new** hunt paths (did not overwrite
[`turn-sequential-census-v1.json`](contact-snapshots/turn-sequential-census-v1.json)).
Record: [`turn-plant-state-census-v1.json`](contact-snapshots/turn-plant-state-census-v1.json).

| Screen | Scored turns | Nets / radii | Notes |
| --- | --- | --- | --- |
| Isolated turn ×5 | 5/5 pass | net 0.169–0.196 m, r 0.095–0.110 m | `stand_end_v=0`, abs slip 0.0035–0.0048 |
| After reverse ×5 | 4 scored, all pass | net 0.150–0.198 m, r 0.084–0.118 m | 1 reverse stay-WALK abort (CCP `accept=0` twice) |
| After reverse+straight ×5 | 5 scored, 3/5 pass | fail 0.264 / 0.235 m (r 0.153 / 0.134); pass 0.210 / 0.194 / 0.207 | Prefix that grows r |
| Sequential ×5 | 1 scored pass 0.185 m | 4/5 abort on **straight** SpeedLimit stay-WALK | Do not mix aborts into radius stats |

Every scored turn: `cmd_yaw`/`raw_wz` 0.45, raw planar 0, `yaw_dominant` true, Φ≈0.002.
Residual `stand_end` speed ≤ 0.006 m/s. Loaded-stance slip does **not** track net
(after-reverse passes had higher slip than the 0.264 m fail).

**Class 3: prefix-grown radius after reverse+straight**, not reverse alone.
STAND does not call `resetWarmStarts`, but after-reverse (same carry) stays
under 0.21 m, so a STAND warm-start wipe is not evidenced. No CRBA/1.85×
signal. **No production lever.** Do not loosen 0.21 m, lengthen STAND, or
respawn.


