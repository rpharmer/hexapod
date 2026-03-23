# Next Steps (March 23, 2026)

## Phase 1 (1 week) — Core maintainability

1. Refactor `SimpleHardwareBridge` request/decode duplication.
2. Split `RobotRuntime::controlStep` into pure decision helpers.
3. Add/adjust focused unit tests for freshness gate edge cases.

## Phase 2 (1 week) — Firmware dispatch cleanup

1. Replace forwarding wrappers in `command_dispatch.cpp`.
2. Ensure payload policy behavior remains unchanged.
3. Extend firmware host-test coverage for invalid payload lengths.

## Phase 3 (1 week) — Protocol metadata unification

1. Create shared command metadata source in `hexapod-common`.
2. Migrate host command-name telemetry to shared metadata.
3. Add a consistency test ensuring command IDs and route entries remain aligned.

## Ongoing

- Run scenario sweep in sim mode before hardware validation.
- Track command failure and freshness-stale counters over longer test runs.
- Keep documentation synchronized with implementation changes.

## Phase 4 (design spike) — Flat-surface servo calibration + speed dynamics probe

Goal: when the robot is standing on a known flat surface, run a guided routine that intentionally touches each foot to several XY spots and uses contact timing/geometry residuals to jointly (a) validate/refine `ServoCalibration` offsets/signs and (b) identify per-joint speed dynamics so touchdown can be predicted and verified from sensed contact.

### 4.1 Probe routine and telemetry

1. Add a host-side calibration mode runner (safe, low speed, one leg at a time) that executes a fixed probe pattern per leg while keeping a stable support polygon with the other five legs.
2. Extend touch telemetry from static geometry tuples to time series:
   - command timestamp `t_cmd`, per-joint commanded servo angle `q_cmd(t)`, and commanded foot target `p_cmd_body(t)`;
   - sampled joint state `q_meas(t)` and filtered velocity estimate `dq_meas/dt`;
   - foot-contact bit with edge timestamps (`contact_rise`, `contact_fall`);
   - estimated body pose and body velocity from the estimator.
3. Record each probe attempt as a segment: `approach -> touchdown -> hold -> unload`.

### 4.2 Calibration fit (height residual)

1. Fit per-joint calibration correction terms by minimizing ground-plane height error at contact events (`z_foot ~= 0` on the known plane), with robust rejection for missed/slipped contacts.
2. Weight contact samples by confidence:
   - higher weight when contact edge persists for `N` consecutive samples (debounced);
   - lower weight when tangential slip is inferred from changing XY while in-contact;
   - zero weight for events that violate force/current/contact plausibility checks.
3. Keep updates bounded and operator-reviewed: emit a proposed calibration delta report first, then require explicit apply/commit step.

### 4.3 Servo speed dynamics model

1. Add a per-joint first-order speed model during probe execution:
   - `dq/dt = sat((q_cmd - q)/tau_j, vmax_j)` or equivalent discrete form;
   - identify `tau_j` (response time constant) and `vmax_j` (speed limit) per servo from command/measurement traces.
2. Include gravity/load dependence by fitting separate parameter sets for:
   - downward approach (loaded/compressive direction);
   - upward unload (lifting direction).
3. Model latency budget to touchdown:
   - controller + transport delay `t_delay`;
   - dynamic travel time from current angle to predicted contact angle from IK + calibration;
   - expected touchdown time `t_touch_pred = t_cmd + t_delay + t_travel`.
4. Track residuals:
   - `e_touch_time = t_touch_meas - t_touch_pred`;
   - `e_touch_height = z_foot(t_touch_meas) - z_plane`;
   - per-leg distributions used to detect sticky servos, asymmetric friction, or calibration drift.

### 4.4 Destination reached logic using foot contact

1. For descending probe moves, treat foot destination as reached when **both** conditions hold:
   - geometric proximity: `||p_foot - p_target|| < eps_pos` (or joint-space equivalent);
   - contact-confirmed touchdown: rising edge + debounce window satisfied.
2. If contact occurs before geometric proximity threshold:
   - mark as early-contact event;
   - back-solve implied ground intersection to update calibration residual set.
3. If geometric target is reached without contact by timeout:
   - classify as miss/soft-ground/failed-contact;
   - exclude from calibration fit and flag operator warning.
4. For lateral in-contact motion, destination reached should require maintained contact (or expected controlled loss) to avoid counting swing-leg pass-through as a successful touch.

### 4.5 Safety and validation

1. Add safety interlocks: abort on contact-count anomalies, over-current, timeout, or IK infeasible target; automatically return to neutral stance on abort.
2. Add simulation + hardware acceptance checks:
   - sim: inject known offset/sign errors and known `tau_j`/`vmax_j`, verify fitter recovers both within tolerance;
   - sim: inject synthetic contact jitter/latency and verify debounce + residual gating reject outliers;
   - hardware: repeatability check across two runs and two surface locations, and compare identified speed parameters across runs.

Deliverables:
- design note for probe state machine, dynamics-identification method, and telemetry schema;
- scenario(s) that exercise nominal probe, early-contact, no-contact timeout, and safety abort paths;
- test coverage for estimation/math utilities used by calibration + touchdown-time fitting;
- operator-facing report format with calibration deltas, `tau_j`/`vmax_j`, and touchdown residual summaries.
