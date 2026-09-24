# Locomotion progress and actuator-capacity campaign

Started: 2026-09-23
Starting baseline: `24cf1c0`; the visualiser and command-channel work is included in this checkpoint.
Plant: built-in hexapod, 0.14 m body command, Pinocchio proximal solver mode 1.
Scope: more useful forward/reverse/strafe progress at a given input without losing the recently recovered support and swing clearance. Keep the existing architecture and safety limits.

This campaign follows [the support and clearance repair](SEQUENTIAL_WALK_DISTANCE_LEFTOVERS.md#326-planted-foot-reach-and-measured-contact-swing-clearance-2026-09-22). Its benchmark is the **measured body motion**, not the requested step length. It also follows the developing [visualiser command channel](VISUALISER_COMMAND_CHANNEL.md); existing visualiser changes belong to their author and are preserved.

## First evidence and interpretation

One rebuilt, isolated `forward_walk` run at the normal motor setting gave:

| Signal | Value | Interpretation |
|---|---:|---|
| Requested / mean governed / mean actual forward speed | 0.200 / 0.162 / 0.0885 m/s | Body collected about 55% of governed speed in this run. |
| Mean gait step length / frequency | 42.3 mm / 0.802 Hz | A larger requested step does not establish a larger planted stroke. |
| Mean planted stroke used, legs 0–5 | 30.6, 35.7, 31.0, 30.4, 40.1, 37.5 mm | The stroke latch allowed about 97–100 mm, but substantially less was used while loaded. This is a per-sample mean, not a completed-stride displacement. |
| Loaded-stance workspace limiting, legs 0–5 | 0%, 0%, 80.7%, 80.4%, 92.1%, 95.7% | The middle/rear four legs frequently reach the current geometric boundary. Confirm direction and remaining reach before shifting nominal foot positions. |
| Raw contact during planned swing, legs 0–5 | 21.7%, 9.9%, 52.3%, 48.2%, 27.5%, 30.1% | In particular, the middle pair often remains in contact during scheduled swing. The phase census below locates this mostly near liftoff and touchdown, not at apex. |
| Peak servo torque / stall torque | 54.6% | One peak over 18 joints and a whole run. It is **not** available torque reserve: assisting torque falls with measured joint speed, and the metric does not report requested-versus-available saturation. |
| Maximum commanded joint rate | 7.480 rad/s | The target rate touches its no-load cap. This does not say that an actual joint reached that speed. |
| Solver held samples | 0 | This run passed its named gate. |

A second isolated run with *only* simulator stall torque multiplied by `0.73` passed: actual forward speed 0.0883 m/s, net horizontal distance 1.067 m versus 1.073 m normally, and zero held samples. Its peak fraction of the **reduced** stall torque was 72.6%. This single pair suggests stall torque is not the main forward-progress constraint at this command. It does not establish a safe hardware reserve or a result for turns, transitions, or higher speed. The same 7.480 rad/s no-load speed remained in both runs.

The simulator currently uses 1.471 N·m and 7.480 rad/s for its nominal MG996R. [TowerPro's MG996R page](https://towerpro.com.tw/product/mg996R/) lists 11 kg·cm (about 1.079 N·m) and 0.15 s/60° (about 6.98 rad/s) at 6 V. The nominal simulator stall torque is about 36% above that listing. Servo variant, supply voltage, gearing, and actual hardware still need identification; this comparison is not authority to change the plant constants during a gait experiment.

The machine-readable baseline, binary hashes, all six per-leg rates, and both process outcomes are in [locomotion-capacity-baseline-20260923.json](contact-snapshots/locomotion-capacity-baseline-20260923.json). These are two isolated observations, not a repeatability campaign.

## Batch A1 — directional reach and swing phase (2026-09-23)

The rebuilt live test now reports four equal swing-phase bins, raw and fused contact per bin, FK foot height relative to the previous stance contact, and fixed-height directional reach along the commanded stance stroke. An opt-in `HEXAPOD_WALK_CAPACITY_TRACE=1` emits one JSON record per leg and control sample with phase, contacts, foot pose, reach, and joint target/measured angle and measured rate. This is test instrumentation; the production gait and physics defaults are unchanged. The reach query tests the femur–tibia annulus only, not servo angle limits, ground clearance, or achievable force.

Three isolated forward repeats, plus reverse and turn controls, passed their existing named gates with zero held samples. The [census JSON](contact-snapshots/locomotion-capacity-census-20260923.json) records the binary hashes, all six legs, and every phase bin.

| Loaded stance (forward) | R3/L3 (body +X pair) | R2/L2 (middle) | R1/L1 (body −X pair) |
|---|---:|---:|---:|
| Mean remaining geometric travel in stroke direction | 192–204 mm | 9.2–9.9 mm | 0.10–0.14 mm |
| At stance entry | ≥250 mm probe cap | 67–69 mm | 0 mm |
| At the same XY but a hypothetical −0.14 m body-frame foot height, on valid geometric samples | roughly 175 mm | roughly 35–37 mm | roughly 41–43 mm |
| Mean *commanded* loaded-stance foot z | roughly −0.128 m | roughly −0.146 m | roughly −0.158 m |

Reverse mirrors the reach split: R3/L3 fall to roughly 0.1 mm; R1/L1 gain roughly 193–204 mm. This supports a directional workspace bottleneck, not a single bad leg or a frame-swap diagnosis. The hypothetical-height row excludes samples that are already outside the annulus at that counterfactual height; it isolates geometric sensitivity, not a realizable gait. It is **not** a proposal to lower the robot or bypass height hold. The rear pair's commanded foot height and XY together place it at the annulus boundary when forward stance begins. The body-pitch mean was roughly +0.054 rad, but this census cannot apportion the height difference among forward lean, measured-tilt feedback, sag compensation, and terrain bias.

Across the three forward repeats, raw contact in the R2/L2 planned swing was:

| Swing phase | R2 | L2 |
|---|---:|---:|
| 0–25% | 86.8% | 94.7% |
| 25–50% | 38.6% | 23.4% |
| 50–75% | 1.4% | 0% |
| 75–100% | 83.2% | 74.8% |

The middle pair does clear contact around the central swing, so “dragging for half the swing” would be misleading. It tends to leave late and contact again early. The FK-height trace is not the 18 mm contact sphere's bottom clearance, so do not turn these percentages into a clearance claim without the sphere/manifold trace.

Two single-variable screens were run and **reverted**: an inward neutral reach fraction of 0.50/0.45 raised isolated forward speed only about 2–4% and did not preserve useful rear reach after touchdown; reducing forward lean to 0.75/0.50 reduced speed from about 0.088 to 0.086/0.083 m/s. They did not reach the campaign's ≥10% selection target; no repeatability or wider safety qualification was claimed. Neither setting remains available in production.

Next geometric diagnostic: record the committed swing foothold before/after body rotation and reach projection, alongside the effective body-height/lean components. This will distinguish a foothold planted at the wrong XY from one constrained as the loaded Z target extends. Any placement screen must include the resulting support polygon and stance-foot slip.

## Batch A2 — actual motor envelope (2026-09-23)

The simulator now has an opt-in accepted-first-attempt trace of all 18 joints: error, measured joint speed, requested torque, speed-dependent available torque, applied torque, load-bearing classification, and timestep. The [motor-capacity report](contact-snapshots/locomotion-motor-capacity-20260923.json) is from one isolated forward case on the unchanged production motor model. Its analyzer checks the torque-speed formula, saturation clamp, and peak actuator impulse against `applied torque × dt`; the existing servo PD and torque-saturation unit tests pass.

Across 7,180 healthy substeps (129,240 joint samples), the trace found **zero torque-envelope clipping** and no zero-available-torque samples. The largest per-joint 99th-percentile `|requested| / available` was 0.537 (L2 coxa while load-bearing); the highest 95th-percentile actual joint speed was about 5.9 rad/s (R2 tibia while not load-bearing) versus the 7.48 rad/s model no-load speed. Twenty `RecoveredRetry` substeps, about 0.3% of the walk window, were excluded because their actuator equation differs from a healthy first attempt. The analyzer's worst torque-envelope, clamp, and torque-times-timestep impulse discrepancies were below 1e-10 in their respective units. This is strong evidence that the *simulated* servo torque limit is not what truncates ordinary forward strokes. It does not establish hardware current, temperature, voltage, or safe acceleration headroom, and it does not clear the skipped retry moments.

The controller target-rate maximum can still touch 7.48 rad/s while measured joint rates and torque requests remain well inside their operating envelopes. Target-rate saturation and motor-torque saturation must remain separate metrics. The next lever remains the touchdown XY/loaded-Z reach coupling; do not raise motor torque or cadence on this evidence.

## Batch A3 — where the foothold is lost (2026-09-23)

An opt-in `HEXAPOD_FOOTHOLD_REACH_TRACE=1` now emits each leg's current swing endpoint, target before reach projection, emitted target, pose/height terms, and contact/phase. [The analyzer](../scripts/analyze_foothold_reach.py) pairs consecutive samples at stance–swing boundaries. This is observation only when the flag is absent. The [boundary report](contact-snapshots/locomotion-foothold-boundary-20260923.json) records the paired screen and binary hashes.

The limiting rear R1/L1 pair in forward travel has a **56.8/57.8 mm mean one-tick target jump at liftoff**. The swing Bezier starts at `anchor + ideal stance velocity × duty/frequency`, but the preceding planted target was already projected onto its reach boundary. The ideal start is not the actual end of stance. At late swing, the mean planned endpoint has body X near −0.20 m; pose rotation and reach projection leave the emitted foot near −0.16 m. At the first stance sample it is near −0.148 m with body-frame Z near −0.158 m and another 12–13 mm XY projection. Thus extending the planned endpoint does not automatically buy the planted pair more stroke. Mean commanded pose pitch was about +0.198 rad while measured pitch was about +0.056 rad in this trace; these are different quantities, and the trace does not establish that pitch alone causes the loss.

One test-only screen latched the **last emitted reachable stance XY** as swing's starting XY. On the same binaries, the rear-pair liftoff jump fell to 7.4/7.4 mm. Isolated forward body speed rose 0.0886→0.0952 m/s and reverse magnitude 0.0891→0.0948 m/s, with zero held steps. But the forward limiting-pair planned-swing contact increased from 27/31% to 40/42%; reverse shows the mirrored front-pair increase from 27/29% to 40/42%. That trades trajectory continuity for more foot dragging, and the single-run 7.4/6.5% progress increases fall below the campaign's ≥10% selection target. The screen was **fully reverted**; there is no production gait change.

The next bounded candidate was to couple actual-start geometry to earlier physical liftoff. Batch A4 tested and rejected that simple coupling. The next experiment must reference **post-projection world contact height** during liftoff, not just reshape the nominal-height arc.

After removing the screen, the rebuilt default-off tree passed `foot_reachability`, `body_controller_height_hold`, `body_controller_velocity`, and the complete sequential `physics_sim_walk_distance` CTests (4/4). This verifies the new observation path did not change the default walking result. The longer visualiser and gait campaigns below remain open.

## Batch A4 — smooth earlier lift did not restore physical clearance (2026-09-23)

A second test-only candidate combined the actual reachable stance XY as swing start with a C1 vertical arc proportional to 16(t(1−t))² instead of 64(t(1−t))³. It retained the same zero-slope endpoints, apex height, and swing duration. On a same-bin single-run comparison, forward body speed rose 0.0887→0.1016 m/s and reverse magnitude 0.0891→0.1008 m/s, with zero held samples. These are promising *single observations*, not five-run medians. The [screen report](contact-snapshots/locomotion-swing-boundary-lift-screen-20260923.json) records them.

The physical result failed two more important checks. In forward travel, the rear R1/L1 feet were in raw contact through roughly **79% of the first swing quarter**, up from 21/30% on the default. Their mean measured lift relative to the previous stance contact fell from about +2.6 mm to slightly below zero. The earlier nominal arc did not help because the **emitted** early-swing body-Z target moved about 23 mm *down*, from roughly −0.136 m to −0.159 m. The former unreachable swing-start XY had been projected upward by the reach clamp, giving accidental immediate clearance; anchoring the swing at reachable XY removed that upward projection. A smooth profile referenced to the unloaded nominal anchor is therefore not equivalent to a clearance trajectory referenced to the last real contact. The middle pair did unload more cleanly, but this cannot excuse the limiting pair's extra drag.

The global candidate also made isolated turn-in-place rotate **against** the +0.45 rad/s command (yaw −3.03 rad versus +2.80 rad default). The command census retained the positive yaw command and the solver held zero samples, so this is a gait/contact outcome, not a command-sign or bus fault. It fails the unchanged turn gate and was fully reverted. A separate trace found zero early-contact samples forced to touchdown phase on either plant; the contact-response phase rule is not this early-liftoff cause. Do not change that rule on this evidence.

Next: compute a smooth liftoff clearance floor from the *last measured contact-sphere height*, after body-pose transform and reach projection, while preserving feasible XY and zero endpoint discontinuity. Measure sphere clearance, early joint tracking and available torque at that point, and the contact force of the remaining supports. Test forward/reverse and turn independently before any production proposal. Restricting the candidate to linear commands could avoid the turn failure, but would introduce a command-transition boundary; it requires its own transition regression rather than being treated as a free gate pass. To repeat the first-quarter target/measured census on the production settings, run one isolated walk-distance case with HEXAPOD_WALK_CAPACITY_TRACE=1 and pipe combined output to scripts/analyze_swing_liftoff.py.

After removal, the rebuilt default-off tree passed foothold_planner, foot_reachability, contact_foot_response, and the full sequential physics_sim_walk_distance CTests (4/4). No failed candidate behavior remains enabled.

## Batch A5 — contact-referenced clearance after reach projection (2026-09-23)

An opt-in screen, `HEXAPOD_SWING_CONTACT_CLEARANCE_SCREEN=1`, raises a swinging foot toward its last support-foot world height plus the existing swing lift, **after** the normal reach projection. It smooths the correction to zero at liftoff and touchdown, reprojects the result into the same reachable annulus, and leaves the default path unchanged. The reference is the measured-pose FK foot-tip centre retained at support, not a direct manifold measurement of the 18 mm contact sphere's bottom. This distinction matters when interpreting clearance.

The first global screen improved isolated forward/reverse progress but reversed turn-in-place yaw. The retained experiment therefore runs only for pure planar translation with exactly zero commanded yaw. This is a diagnostic restriction, **not** a production-ready mode boundary: turning or mixed commands can cross it mid-swing.

Five matched isolated runs per translation direction, on the same binaries, passed their named gates with zero held samples:

| Metric | Screen off | Screen on |
|---|---:|---:|
| Forward median body speed | 0.08928 m/s | 0.09958 m/s (+11.5%) |
| Reverse median body-speed magnitude | 0.08724 m/s | 0.09891 m/s (+13.4%) |
| Planned-swing raw-contact fraction, forward | about 30.6–31.6% | about 20.8–21.6% |
| Planned-swing raw-contact fraction, reverse | about 30.4–31.7% | about 21.1–21.7% |
| Minimum body height, forward | 0.1425–0.1442 m | 0.1392–0.1411 m |
| Minimum body height, reverse | 0.1426–0.1441 m | 0.1382–0.1406 m |

The complete sequential walk-distance test passed once with the translation-only screen, including turn-in-place. A separate opt-in run passed 12/12 selected tests: v16 exact replay at 200/120/240/480 Hz, walk entry, walk stability, oblique clearance, slow-forward and WAVE clearance, slow-forward contact loss, static tripod, and the canonical locomotion regression suite. The 60-second stand gate passed. Single paired full-profile strafe-left and strafe-right cases passed with zero held samples; net displacement was 0.132→0.148 m and 0.130→0.159 m, respectively. These are **one pair each**, not strafe repeatability evidence.

The opt-in **fails** the unchanged `locomotion_regression_suite_stress` gate. Its `long_walk_observability` scenario expects an eventual TIP_OVER or BODY_COLLAPSE within the 60-second stress window. Screen off passed with TIP_OVER at step 4236, path 1.587 m; screen on ran all 12,000 samples with no fault, path 3.814 m. Both runs had zero held physics samples. The separate 25-second `long_walk_contact_health` case passed with the screen on, no fault or held sample, and path 1.196 m. An absence of a fault is not a safety failure by itself, but it violates the observability test's deliberately fault-positive contract. Do not relabel it green or weaken the gate on this evidence. The screen remains **opt-in only**, and no production gait/default change is selected.

The [A5 machine-readable report](contact-snapshots/locomotion-contact-clearance-screen-20260923.json) records the paired outcomes and binary hashes. Before reconsidering this candidate, resolve the mixed-command transition boundary, measure actual sphere-bottom clearance and support forces (not FK alone), repeat strafe/diagonal and long-run comparisons, and decide separately how to verify safety detection without requiring a healthy candidate to topple on one particular stress trajectory. The last point is a test-contract decision, not permission to loosen the existing gate.

## Batch A6 — physical sphere/load audit and yaw boundary (2026-09-23)

The simulator now offers an opt-in `HEXAPOD_FOOT_PHYSICS_TRACE=1` record at each bus response. It reads each tibia's **actual compound collision sphere** centre and radius, computes signed clearance to the built-in ground plane, and reports the proximal normal impulse summed over that command's physics substeps. The analyzer pairs this with the controller's opt-in phase trace by stream order; 14,394 of 14,400 leg samples paired in each isolated forward run. A positive normal impulse can coexist with no *final* manifold contact because the impulse is accumulated during substeps and contact is sampled after the step. No wire field or default behavior changed.

One paired forward run confirmed real unloading, rather than only a commanded FK change:

| Physical signal during planned swing | Screen off | Screen on |
|---|---:|---:|
| R2 early contact / late contact | 79% / 89% | 56% / 55% |
| L2 early contact / late contact | 90% / 72% | 65% / 53% |
| R2 mid-swing mean sphere-to-plane clearance | 9.0 mm | 14.4 mm |
| L2 mid-swing mean sphere-to-plane clearance | 13.5 mm | 14.4 mm |
| L2 early mean normal impulse per bus command | 0.027 N·s | 0.0087 N·s |

The middle pair is **not fully unloaded** near liftoff/touchdown. R2 early mean impulse actually rose slightly despite fewer contact frames, so neither contact fraction nor mean impulse alone is a complete support metric. This is a single matched physical trace, not a five-run force census.

An explicit-only translation → mixed yaw → pure turn → translation probe then exposed a target discontinuity in the A5 exact-zero-yaw screen: a mid-swing foot target jumped 13.1 mm when yaw became 0.04 rad/s, versus 5.8 mm with the screen off. The opt-in screen now multiplies its correction by `1 − smoothstep(|yaw| / 0.20)`. In one repeated probe the boundary jump fell to 3.1 mm, near the 3.0 mm p95 ordinary per-tick target change; the full sequential walk-distance and canonical locomotion suites passed with this blend. Pure turn receives no correction because planar translation is zero. The new probe is explicit-only and does not silently add a new default CTest gate.

The fault-positive long-walk stress test still fails with the blended screen: 12,000 samples, path 3.525 m, no fault, zero held; default-off still passes its expected TIP_OVER test. In contrast, the existing explicit safety cases `tilt_safety_trip` and `tilt_safety_immediate` both passed with the screen on and reported TIP_OVER with zero held. That is evidence that detection still functions for those unsafe commands, **not** authorization to remove the unchanged long-walk expectation. Production remains screen-off. The [A6 measurement report](contact-snapshots/locomotion-physical-clearance-yaw-20260923.json) records the limited screen and checks.

Next qualification: repeat the physical sphere/load trace across seeds and directions; test mixed yaw over full strides; check the solver's 240 Hz p99 and ten-minute feasible-gait soak. The A7 isolated diagonal sweep and separate safety probe below are partial steps, not substitutes for those campaigns. Do not silently reinterpret the current stress failure or change the 0.45 rad/s rate/path policy.

## Batch A7 — directional qualification and long-walk safety split (2026-09-23)

Five matched opt-in-off/on pairs were run for each of forward, reverse, left/right strafe, and left/right diagonal travel on the same rebuilt physics binary. All **60 isolated named gates passed with zero held samples**. Median forward body speed rose 0.0891→0.0994 m/s (+11.5%); reverse magnitude rose 0.0878→0.0994 m/s (+13.2%). Median net displacement improved +18.9%/+20.0% for left/right strafe and +14.0%/+21.1% for left/right diagonal travel. Forward/reverse planned-swing raw-contact fraction fell from about 31–32% to 21%. The screen's worst minimum body height was lower by roughly 3–4 mm in these short walks but remained within their named gates. The strafe/diagonal cases do not expose an equivalent contact-sphere penetration metric; their zero-valued field must not be interpreted as zero penetration.

The long-walk blocker is more complicated than the earlier single-run “healthy walk fails a fault-positive gate” observation. Three repeated 60 s runs **per mode** under identical long-walk configuration produced: screen off, one no-fault completion and two `BUS_TIMEOUT` after physics holds; screen on, one no-fault completion, one `BUS_TIMEOUT` after holds, and one `TIP_OVER` that nominally passed the old gate but also recorded a failed physics read and ten held steps. Captured first failed solver reasons for the timeout runs were `SpeedLimit`. The old stress case therefore cannot establish healthy long-run robustness for either mode from one pass or one no-fault completion. The sample is small and not a controlled distribution estimate. No stress limit or fault expectation was changed.

An explicit-only `long_walk_injected_tilt_safety` probe now uses the same long-walk controller settings but presents one 0.90 rad measured-roll sample after 13.5 s. It requires `TIP_OVER` on that exact sample, with no failed read or held physics sample. Both off and on passed at step 2700. This verifies safety detection under the relevant configuration without requiring the physical plant to fall; it does not verify the plant's response to a physical push. A direct physical-pose injection was attempted but caused extreme penetration/held state, so it was reverted. The probe has its own CTest and is excluded from the canonical/stress profiles so it is not accidentally run twice; it does not replace the current stress gate.

**Decision:** screen stays opt-in. Before longer promotion campaigns, classify the shared long-walk SpeedLimit/hold failures separately from the candidate's clearance effect. A future test-contract split should require a *healthy* full-duration walk (zero held/read failures, existing motion/clearance gates) plus this explicit safety challenge; never count a BUS_TIMEOUT or a held-sample-associated `TIP_OVER` as safety success. The [A7 machine-readable report](contact-snapshots/locomotion-clearance-qualification-20260923.json) records the paired scorecard and the limited stress sample. Use `TMPDIR=/var/tmp` for subsequent WSL runs: earlier replay bundles filled the 3.9 GB `/tmp` tmpfs. Those bundles were moved intact to `/var/tmp/hexapod_locomotion_regression_archive_20260923` with a symlink retaining the old path; no bundles were deleted, and `/tmp` has free space again.

One further off/on long-walk pair with `TMPDIR=/var/tmp` completed the full 60 s with no fault or held sample in either mode; both still failed the existing fault-positive expectation. This rules out “every long walk fails physically,” but does not prove that the earlier intermittent holds were caused by the full temporary filesystem. Do not attribute causality without a controlled repetition.

## Batch A8 — honest 60-second health gate and first hold classification (2026-09-23)

The former `long_walk_observability` gate was fault-positive: it failed healthy no-fault completions and could pass a `TIP_OVER` that followed a failed physics read. Its requested motion also contained a sudden **0.6 m/s sideways tripod command**, originally intended to provoke instability. Keeping that input while demanding 60 s of healthy walking was not a sound qualification test: in an initial three-pair attempt under the corrected no-fault rule, production default passed 3/3 but the clearance screen passed 2/3, with one genuine no-hold `TIP_OVER`. In eight further production-default runs of the old input, four completed and four tipped without any held sample. Those are physical stress outcomes, not evidence that the health gate should ignore tilt.

The scored stress-profile case now uses [a separate 60-second feasible mixed-gait scenario](../hexapod-server/scenarios/05_long_walk_feasible_health.toml). It preserves the same phase durations, body height, gaits, directions and yaw requests, changing only the deliberately destabilising sideways speed from 0.6 to **0.10 m/s**. The evaluator requires *every* requested WALK sample (10,800 at the current 5 ms bus cadence), a return to STAND, zero faults, zero held solver samples, zero failed reads, and the existing path, stride, transition, stance-anchor, tracking and foot-height limits. The obsolete fault-time keys were removed from the manifest; none of the remaining numerical acceptance limits or plant/safety settings changed. The original scenario remains unchanged as explicit-only `--case long_walk_aggressive_diagnostic`. It is not silently dropped or counted as a healthy-walk pass.

Five matched default-off/on pairs on the new health scenario passed **10/10**: 60 s, 10,800 walking samples, 31 strides, no fault/read failure/hold. Median path was 2.896 m off and 3.022 m on; the worst reported contact penetration was 1.38 mm off and 1.44 mm on. The independent exact-sample measured-tilt CTest passed in both modes after the scenario split. These runs qualify the *test split*, not production promotion: there is no 100-seed, ten-minute soak or 240 Hz p99 result for this candidate.

The first captured hold on the original aggressive input was **not** a five-foot hyperstatic contact: two unique contacts (IDs 7 and 19, joints 7 and 19), no duplicates, with a swing femur as the speed-limit frame. At dt 1.667 ms the first attempt reached link ω 10.47 rad/s, the retry 10.85 rad/s, against the unchanged 10 rad/s guard; peak joint target error was 1.49 rad. The first attempt's 6 ADMM iterations and zero NCP dual residual do not indicate solver exhaustion. A trace-only addition now prints the winning leg's joint angles, rates, targets and errors if the next hold occurs. Eight subsequent runs on that same aggressive input produced four clean completions and four no-hold physical tilts, so no second held sample was captured with the expanded trace. Do not infer a specific leg or prescribe a torque/solver change from this record. The [A8 report](contact-snapshots/locomotion-long-walk-health-split-20260923.json) records the binary hashes and bounded outcomes.

**Decision:** keep the clearance screen opt-in. The new healthy long-walk gate and the separate safety challenge are ready for routine verification; the original aggressive input remains diagnostic. The swing-femur SpeedLimit hold is a distinct intermittent issue and the tilt behaviour of the aggressive input is a separate physical-stability question. Neither is resolved by this test-contract repair.

Reproduce from the repository root, using the already-built live simulator:

```bash
source scripts/lib/pinocchio_env.sh
export TMPDIR=/var/tmp
export HEXAPOD_PHYSICS_SIM_EXE="$PWD/hexapod-physics-sim/build/hexapod-physics-sim"
unset HEXAPOD_WALK_TEST_SOLVER_MODE
ctest --test-dir hexapod-server/build-tests \
  -R '^locomotion_regression_suite_(stress|long_walk_tilt_safety)$' --output-on-failure
HEXAPOD_SWING_CONTACT_CLEARANCE_SCREEN=1 ctest --test-dir hexapod-server/build-tests \
  -R '^locomotion_regression_suite_(stress|long_walk_tilt_safety)$' --output-on-failure
hexapod-server/build-tests/test_locomotion_regression_suite \
  --case long_walk_aggressive_diagnostic --emit-metrics-json
```

## Batch A9 — live seeded rare-failure qualification (2026-09-23)

The live regression binary now accepts `--perturbation-seed N`. It uses the
existing exact-replay seed channels at scale 0.25 to perturb the initial body
pose by at most 0.75 mm horizontally, 0.375 mm vertically, 0.1875° in roll or
pitch, and 0.25° in yaw, and seeds simulator contact ordering in the child
process. This exercises *live controller output*; it does not change the frozen
v16 command fixture. The two-process live run remains timing-sensitive: the
same seed on the historical aggressive input produced healthy completion, a
no-hold `TIP_OVER`, and a `SpeedLimit` hold across repetitions.

On identical test and simulator binaries, seeds **0–99** each ran the feasible
60-second mixed walk with the clearance screen off and on. **200/200** runs
completed all 10,800 requested WALK samples and returned to STAND: zero
faults, failed reads, or held solver samples in either mode. Maximum reported
contact penetration was 1.69 mm off and 1.49 mm on; maximum roll was 0.112 and
0.130 rad. The scored mixed-route path median was 2.894 m off and 3.036 m on;
the screened path was longer in all 100 pairs, with mean on/off ratio 1.050.
Because the main sequence always ran off first, five further seeds were run
in **on-then-off** order. All ten runs passed and the screened path remained
longer in all five pairs. Path length is total travel around the mixed route,
not proof of a 10% useful-direction capacity gain.

A traced repeat of the old 0.6 m/s sideways input gives a separate causal
sequence. Body roll crossed 0.3 rad at step 3197. R2 entered planned stance
without raw foot contact at step 3217, and the roll crossed 0.5 rad at 3227;
that planned-support/contact mismatch lasted 77 samples. The foot's world Z
crossed 0.1 m while planned for stance at step 3232. Its Cartesian
tracking error first crossed 0.1 m at 3667, before the failed read at 3739.
The largest R2 sideways target speed was 1.30 m/s, versus 0.248 m/s in a
healthy feasible run. The eventual held sample rejected a swing R2 tibia at
10.0015 rad/s (retry 10.0494) against the unchanged 10 rad/s WORLD_ALIGNED
guard; the femur was already at −8.66 rad/s with −0.975 rad target error.
There were five unique contacts, but only 8 and 6 solver iterations on the
held attempts. A preceding NCP failure did recover; this held decision was
`SpeedLimit`, not ADMM exhaustion. The contact set may amplify the rate, but
this capture does not show redundant contact *initiating* the failure: large
roll and failed load transfer preceded the large tracking error and hold.

Both named `locomotion_regression_suite_stress` and
`locomotion_regression_suite_long_walk_tilt_safety` CTests passed with the
screen off and on after the
seeded campaign. The [A9 machine-readable report](contact-snapshots/locomotion-rare-failure-seeded-20260923.json)
records hashes, acceptance metrics and the preserved raw replay paths. The
first-hold filter and replay timeline analyzer are documented in the
[server README](../hexapod-server/README.md).

**Decision:** leave the clearance screen opt-in and the rigid Pinocchio solver
as the production default. Do not raise the angular-speed guard or retune
friction, torque, gait or safety thresholds to make the deliberately
aggressive input complete. The remaining qualification is the ten-minute
feasible-gait soak and 240 Hz p99, plus a useful-direction progress measure;
the 100-seed result does not substitute for those gates. If the aggressive
diagnostic is revisited, investigate a capacity-aware bound on lateral
swing-target velocity/load transfer as a separate one-lever screen.

## Batch A10 — matched useful-direction progress (2026-09-24)

The user chose to skip the ten-minute soak for now. To answer whether the
clearance screen improves *useful* travel rather than merely increasing total
path length, the full-profile compass cases now emit a diagnostic-only
`directional_progress` object. For consecutive valid WALK samples within one
phase, it projects measured world-body displacement onto the requested
body-frame heading rotated by current body yaw. It also records governor-scaled
requested distance, absolute orthogonal travel, and the fraction of planned
swing samples with raw foot contact. This is not a new pass/fail gate; the
contact fraction is a proxy, not a measured toe-drag distance.

Five matched screen-off/on runs per direction used the same rebuilt binaries,
with off/on order alternated between repetitions. All **60/60** short cases
passed existing gates, with zero held samples and no faults. In every one of
the 30 pairs, the screened case made more command-aligned progress. The
[machine-readable A10 report](contact-snapshots/locomotion-directional-progress-20260924.json)
contains all paired observations, binary hashes, and metric definitions.

| Direction | Median useful progress off → on | Median gain | Planned-swing raw contact off → on |
|---|---:|---:|---:|
| Forward | 0.185 → 0.207 m | +11.9% | 33.4% → 30.5% |
| Reverse | 0.186 → 0.207 m | +11.6% | 33.8% → 30.3% |
| Strafe left | 0.148 → 0.167 m | +12.8% | 31.2% → 27.6% |
| Strafe right | 0.144 → 0.166 m | +15.2% | 31.2% → 30.4% |
| Diagonal forward-left | 0.174 → 0.200 m | +14.9% | 34.0% → 29.4% |
| Diagonal forward-right | 0.172 → 0.198 m | +15.4% | 33.6% → 29.1% |

Median governor-scaled requested distance stayed near 0.377–0.379 m in
every direction and did not increase with the screen. Thus the measured gain
is not explained by a larger governed request. The smallest paired gain was
8.3%, so the ≥10% campaign criterion is met by **direction medians**, not by
every individual run. All existing foot-height and tracking gates passed;
the worst observed contact-tracking error was 94.0 mm and minimum measured
foot world Z was 16.7 mm. Strafe-right median absolute orthogonal travel
increased slightly (14.5 → 14.7 mm), despite greater useful progress.

The frozen v16 exact-command replay at 240 Hz also passed: 720 frames, zero
held/unsupported/read failures, and 0.820 ms physics-step p99 against the
4 ms limit. This timing measures the simulator on frozen commands; the
controller clearance screen is **not** exercised by that replay, so it is not
an end-to-end screened-controller performance result.

**Decision:** the useful-direction selection screen is positive, but keep the
clearance feature opt-in. The skipped endurance soak leaves its default-
promotion reliability gate unverified; neither the solver mode nor any
production gait or safety threshold changes in this batch. A later release
decision should explicitly accept that remaining risk or run the soak, rather
than silently treating short-case and frozen-replay passes as equivalent.

## Batch A11 — default selection (2026-09-24)

At the user's direction, the post-reach contact-referenced swing-clearance
correction is now **enabled when the environment variable is unset**. An exact
`HEXAPOD_SWING_CONTACT_CLEARANCE_SCREEN=0` retains the former path for A/B
diagnosis; `=1` also enables the new default. The change is a default switch,
not a new trajectory, solver, actuator, or safety limit. Earlier batch entries
describe the defaults *at the time of those experiments*; A11 supersedes their
opt-in decision.

This promotion accepts a specific evidence gap: the user chose not to run the
ten-minute feasible-gait soak. The 100-seed, 60-second feasible-walk campaign
and A10 short-run directional gains support the switch, but they cannot prove
the missing endurance gate. Keep the old 0.6 m/s stress input separate from
feasible walking and do not relabel a fault-positive test to hide a miss.

With the variable **unset**, seven focused CTests passed: sequential
`physics_sim_walk_distance`, `body_controller_velocity`, canonical locomotion,
long-walk stress, tilt-safety, and both motion-performance tiers. A one-run
forward comparison passed in all three settings with zero held samples:
unset progress 0.208 m, explicit `=1` progress 0.210 m, and opt-out `=0`
progress 0.187 m. Those live numbers vary slightly between runs; they confirm
the default follows the enabled path, not bit-for-bit replay equivalence.
Eleven further default-on CTests passed: v16 exact replay at 200/120/240/480 Hz,
60-second stand, walk entry, walk stability, oblique clearance, slow-forward
and WAVE clearance, and static tripod support. Thus the focused default-on
set is **18/18** green.
The full repository verification suite and the ten-minute soak were not run
for this default switch.
The focused run used motion-suite SHA-256 `7c7e99037b68206f7efcde9dc058280e46bae8ebd1168cc87cb2fcb72edc2167`,
walk-distance SHA-256 `cf6be2afc8479364ec9f3c9c4fe6beaea04b4d6b63be5334b999faedf4583872`,
canonical-suite SHA-256 `c9d0e64442170aabfffadff1dace472fba6747211d8f4842fb022f766cd82676`,
and simulator SHA-256 `386db54c80feb2926530645002634fbaef6feb0f793f61465aaf4231489bd317`.

## Batch A12 — start live visualiser command qualification (2026-09-24)

The campaign checklist was reconciled with A1–A11 before starting this batch:
the A10 ≥10% useful-progress selection target is complete, the stride/cadence
sweep is a separate optional capacity experiment, and the post-default full
verification/ten-minute soak remain explicitly unrun. A1/A6 already measured
phase contact and physical sphere clearance, so the open swing item is a
*causal classification on the current default*, not a repeat of those traces.

The first command-channel defect was a `select()` wait of up to 750 ms inside
each visualiser button handler. The client now sends immediately, polls its
nonblocking socket during each frame, and accepts only `command_result`
packets from the configured endpoint whose `ref` matches an outstanding
request. It distinguishes pending from applied/rejected, ignores stale replies,
and reports a two-second timeout. A waypoint draft is cleared only after a
matching successful apply reply and only if it has not been edited since send.
The server already emitted apply-result replies; no server protocol change was
needed.

All **11/11** visualiser CTests pass, including a new fake-UDP test for
nonblocking send, stale-ref rejection, apply rejection and timeout. A local
headless interactive server in `sim` mode returned matching live socket replies
for `scenario.list`, `scenario.stop`, and `motion.set` (idle). This verifies
the transport and apply path, **not** a user click inside the WSLg window.
Remaining A12 work: visually exercise List/Run/Stop, click-to-goal, waypoints,
and idle motion in the local stack; verify authority changes and gamepad
restoration; confirm the UI stays responsive while an endpoint is absent.
The separate [Control Room usability campaign](VISUALISER_CONTROL_ROOM_CAMPAIGN.md)
tracks these UI gates and the confirmed unsupported Stop-motion payload.

## Questions to resolve, in order

1. **Where is reach consumed?** For each planted foot, measure its signed remaining horizontal travel in the commanded direction at the requested height, before and after the reach projection. Inspect full stance spells and reverse/strafe as well as forward. A high `workspace_limited` fraction does not by itself prove that moving the nominal foot inward improves motion or preserves support geometry.
2. **When does the scheduled swing actually unload?** Record raw/fused contact, normal impulse, planned phase, foot height above the last planted contact, and actual joint tracking from liftoff through touchdown. Separate early contact, delayed liftoff, recontact, and deliberately held stance. Compare the middle pair with the rear pair.
3. **How much motor effort is available at operating speed?** Record for every joint and substep: target/actual angle and speed, PD requested torque, speed-dependent available torque, applied torque, whether the request was clipped, and actuator work. Summarize p50/p95/p99 and the fraction clipped separately for stance and swing. Display both `|τ_applied|/τ_stall` and `|τ_requested|/τ_available(v)|`; neither alone is a hardware margin.
4. **What share is command/governor loss versus mechanical slip?** Keep requested, governed, and actual body speed, completed touchdown span, used planted stroke, contact mismatch, and world-foot slip in the same time window. In the first run, body forward speed was 55% of governed speed, but per-leg slip and phase-dependent loading require a time-aligned explanation.

## Work batches

### A. Measurement and visualiser readiness

- [x] Capture the first paired forward baseline with the rebuilt current test and Pinocchio mode 1. No production setting changed.
- [x] Add directional annulus-reach and phase-binned contact census to the live test; verify against the default forward/reverse/turn cases and a unit test. The new trace is opt-in.
- [x] Add an opt-in, per-joint simulator motor-envelope trace with requested, speed-available, and applied torque, explicit wire indices and units. Validate torque and impulse accounting against the existing motor tests; recovered-retry and physical hardware capacity remain separate.
- [x] Trace swing start, endpoint, body-pose transform, final reach projection, and stance–swing discontinuities. A last-emitted-start-only screen closed the jump but increased swing contact; it was reverted.
- [x] Screen actual-start plus earlier smooth lift; reject because the limiting feet still drag in early swing and turn-in-place reverses yaw. Preserve the first-quarter target/measured clearance analyzer.
- [x] Screen a post-reach, contact-referenced clearance correction. Forward/reverse progress and swing-contact metrics improved, but the translation-only opt-in fails the existing fault-positive long-walk stress gate; no default change.
- [x] Measure actual collision-sphere clearance and per-leg normal impulse, and probe a mid-swing yaw transition. A smooth opt-in yaw blend removed the observed hard switch; long-walk stress remains red.
- [x] Repeat isolated forward/reverse/left/right strafe/left/right diagonal screens five times each and add an exact-sample measured-tilt safety CTest under the long-walk configuration. All short gates pass, but long-walk SpeedLimit/hold flakiness exists with the screen both off and on.
- [x] Measure command-aligned body progress in five matched full-profile compass runs per direction. All six direction medians improve by 11.6–15.4%, without a larger governor-scaled request; 60/60 short cases pass. The user subsequently chose the A11 default switch with the ten-minute soak explicitly unrun.
- [ ] Add an observation panel to the interactive visualiser: planned versus measured contacts and feet, remaining reach by leg, requested/governed/actual speed, target versus measured joint rate, and motor envelope headroom. Mark stale/missing data rather than displaying it as zero. The server owns all estimates and authority decisions.
- [ ] Finish the visualiser command round trip in a local interactive stack. A12 removed the render-thread UDP wait, matches `command_result.ref`, synchronized the command-channel documentation, and passed 11 visualiser tests plus a headless live-server socket check. The remaining gate is actual GUI click/command/apply and authority/gamepad restoration under WSLg, including an unavailable-endpoint responsiveness check.

### B. Recover usable planted stroke

- [ ] Use the reach trace to construct a per-leg, per-direction workspace map at the loaded 0.14 m stance height. Compare feasible stroke **and** support polygon/contact force for candidate nominal foot positions.
- [ ] Screen a small, test-only shift of the middle/front foot placements toward usable travel. Keep cadence, stride request, body height, actuator model, and contact solver identical to baseline. Try forward and reverse first, then lateral and turning commands; reject a shift that trades one direction or stability for another.
- [ ] Require less reach clipping, increased measured stance stroke/body progress, and unchanged support/lift/height gates before making a production placement change.

### C. Restore clean swing transfer

- [ ] Finish the causal swing-transfer classification if residual drag warrants another gait change. A1/A6 already measured phase-binned contact, actual collision-sphere clearance and normal impulse; remaining work is to distinguish late liftoff, early touchdown, recontact and gait hold on the **current default** across full swing spells, including command-apex tracking.
- [ ] Only if that census identifies a persistent issue after any reach recentering, screen one targeted trajectory/phase change against the A11 default. Compare unloaded time and later plant quality; keep contact-debounce grace and safety support checks.

### D. Speed and stride sweep

- [ ] Optional *further-capacity* experiment, not a prerequisite for the A11 default: after geometric/contact bottlenecks are measured, sweep stride request alone (+5%, +10%) at fixed cadence and command, then cadence alone at fixed stride. Score achieved speed, used stroke, slip, joint speed, envelope clipping, clearance and support margin; do not combine candidates before standalone gates pass.
- [x] Meet the campaign's ≥10% median useful-progress selection target without sacrificing the named direction and clearance gates. A10 measured +11.6–15.4% across six directions in five matched pairs each; A11 switched that same path on by default. This is not a changed product gate.
- [ ] Finish *post-default* qualification if a release requires the original full gate: A11 passed 18 focused CTests, earlier work passed 100 perturbed 60 s seeds, and frozen 240 Hz physics p99 was 0.820 ms. Full `verify.sh` and the ten-minute feasible-gait soak were **not** run; the user explicitly chose to proceed without the latter. Do not claim a full-campaign pass or hide unrelated legacy-PGS results.

### E. Physical-servo calibration

- [ ] Identify the fitted servo variant and measure voltage under walking load, current, no-load speed and a safe short-duration torque/speed point. Compare these to the manufacturer's listing and the model. Record uncertainty and thermal/current limits. Change the simulator motor constants only in a separately scored plant-model campaign; revalidate all walking results after any such change.

## Experiment record

For each screen append a row here or attach a JSON report under `docs/contact-snapshots/`:

| Date / tree and binary hashes | One changed variable | Cases and repetitions | Progress; used stroke; contact and reach | Torque envelope; speed; energy | Existing gates / decision |
|---|---|---|---|---|---|
| 2026-09-23, `24cf1c0` + visualiser/command dirty tree; hashes in baseline JSON | Simulator torque scale 1.00 versus 0.73 | Isolated forward, one each | 0.08847 versus 0.08830 m/s forward; four legs 80–96% workspace limited; middle-pair swing contact about 48–52% | Peak applied/stall 54.6% versus 72.6%; target cap 7.48 rad/s; no held samples | Both named cases pass. Keep as diagnosis; start batch A. |
| 2026-09-23, same base; hashes in census JSON | No changed plant variable; phase/reach observation | Forward ×3, reverse ×1, turn ×1 | Forward rear pair <0.2 mm mean reach; reverse flips the pair. Middle pair has late liftoff and early recontact, not apex contact. | Exact speed-dependent torque headroom still unmeasured | All five named cases pass, zero held. Keep instrumentation; diagnose touchdown XY/Z coupling. |
| 2026-09-23, temporary screen builds, then reverted | Inward nominal reach 0.50/0.45; separately forward-lean scale 0.75/0.50 | Isolated forward, one run per setting | Inward neutral shift gave ~2–4% forward-speed lift but rear stance reach still depleted; lower lean slowed progress | No torque-envelope conclusion | Reject both as production levers on current evidence. No default change. |
| 2026-09-23, same base; hashes in motor report | Opt-in exact Pinocchio motor trace | Isolated forward, one run; 7,180 healthy substeps | Directional reach remains the measured constraint | Zero clipping over 129,240 healthy-joint samples; maximum group p99 request/available 0.537; 20 recovered substeps excluded | Named walk gate passes. Keep diagnostics; do not raise torque. |
| 2026-09-23, same base; hashes in boundary report | Test-only actual stance endpoint as swing start, then reverted | Forward/reverse one run each, plus 14,400 forward per-leg trace samples per setting | Limiting-pair liftoff jump 57–58→7 mm; forward/reverse body speed +7.4/+6.5%; limiting-pair swing contact rose ~12–14 points | Zero held on the paired runs; no new motor limit | Reject standalone lever; pair continuity with early lift in a separate bounded screen. |
| 2026-09-23, same base; hashes in lift-screen report | Test-only reachable start plus C1 early lift, then reverted | Forward/reverse/turn one run each; paired forward phase and full-rate clearance traces | Forward/reverse body speed +~14/+~13% but rear early contact 21/30→79/79%; early commanded Z lowered ~23 mm | Zero held; early-contact phase never forced to touchdown | Reject: turn yaw +2.80→−3.03 rad and real liftoff clearance worsened. |
| 2026-09-23, same base; hashes in A5 report | Opt-in post-reach contact-referenced clearance, pure planar translation only | Forward/reverse five matched pairs; selected CTests, stand, strafe left/right one pair, long-walk stress | Median speed +11.5/+13.4%; raw planned-swing contact about 31→21%; strafe net displacement improved in one pair | All paired isolated walks had zero held; stress also zero held but avoided the expected fault | Keep opt-in only: 12/12 selected checks pass, but fault-positive long-walk stress fails; mixed-command boundary and physical clearance unqualified. |
| 2026-09-23, same base; hashes in A6 report | Opt-in physical sphere/normal-impulse trace; smooth yaw attenuation replaces exact-zero gate | Forward physical trace one matched pair; explicit yaw transition; sequential and canonical CTests; two safety probes | Middle-pair early/late physical contact fell; mid-swing sphere clearance rose; yaw-boundary target jump 13.1→3.1 mm | Zero held in named checks; both deliberate unsafe cases still TIP_OVER | Keep screen opt-in: fault-positive long-walk still fails without a fault. Do not change default or stress gate. |
| 2026-09-23, same base; hashes in A7 report | No new plant lever; qualify existing clearance screen across directions and separate safety detection | 30 matched off/on directional pairs, 3 off + 3 on long stress, one injected safety pair | Isolated forward/reverse median speed +11.5/+13.2%; strafe/diagonal displacement +14–21%; 60/60 short gates pass | Short cases zero held; long stress intermittent on both modes, including SpeedLimit→BUS_TIMEOUT; injected tilt detected at exact sample without holds | Keep opt-in. Long-run robustness and existing stress gate remain unresolved; do not promote. |
| 2026-09-23, same base; hashes in A8 report | Split fault-positive 0.6 m/s stress input from feasible 60 s health and exact-sample tilt safety | Five matched off/on healthy long walks, old-input trace census, safety CTest each mode | Healthy path medians 2.896/3.022 m; all 10 full walk windows complete | Healthy case 0 held/read failures; old input had swing-femur SpeedLimit hold on two contacts and separate no-hold tilts | Keep screen opt-in; retain old input explicitly for diagnosis, do not tune plant or solver from one hold. |
| 2026-09-23, same base; hashes in A9 report | Live seeded initial pose/contact order; no production lever | 100 matched off/on feasible 60 s pairs, five reversed-order pairs, traced historical stress repeat | 200/200 main passes; median scored path 2.894/3.036 m, all 100 path pairs positive; reversed five also positive | Zero main held/fault/read failures; aggressive repeat had roll/contact-loss before tibia SpeedLimit hold | Keep screen opt-in. Feasible seed gate green; old 0.6 m/s stress is a separate capacity/load-transfer diagnostic. |
| 2026-09-24, `24cf1c0` dirty tree; hashes in A10 report | Diagnostic command-aligned progress metric; existing clearance screen off/on | Five matched pairs in each of six compass headings, alternating run order; frozen v16 timing at 240 Hz | All 30 pairs positive; useful-progress direction medians +11.6–15.4%; planned-swing raw-contact medians lower in all six | 60/60 short passes, zero held; frozen physics-step p99 0.820 ms (<4 ms), not a screened-controller timing result | Selection target met by direction medians. Keep opt-in: user skipped ten-minute endurance soak; no default or gate change. |
| 2026-09-24, same tree; A11 | Make the previously qualified contact-clearance path the unset-environment default; retain exact `=0` opt-out | Eighteen focused default-on CTests; one forward run each unset / `=1` / `=0` | Forward useful progress 0.208 / 0.210 / 0.187 m; all three passed with zero held | 18/18 CTests passed, including stress, tilt-safety, stand, replay, and clearance; ten-minute soak and full verify skipped | Promote correction to default with the endurance evidence gap disclosed; leave solver and safety limits unchanged. |

### Reproduce the first baseline

From the repository root, after rebuilding `hexapod-server/build-tests/test_physics_sim_walk_distance` and the physics simulator:

```bash
source scripts/lib/pinocchio_env.sh
export HEXAPOD_PHYSICS_SIM_EXE=/home/volly/pico/hexapod/hexapod-physics-sim/build/hexapod-physics-sim
unset HEXAPOD_WALK_TEST_SOLVER_MODE HEXAPOD_SWING_CONTACT_HEIGHT HEXAPOD_WALK_LOAD_PHASE
HEXAPOD_WALK_TEST_CASE=forward_walk HEXAPOD_SERVO_TORQUE_SCALE=1 \
  hexapod-server/build-tests/test_physics_sim_walk_distance --emit-metrics-json
HEXAPOD_WALK_TEST_CASE=forward_walk HEXAPOD_SERVO_TORQUE_SCALE=0.73 \
  hexapod-server/build-tests/test_physics_sim_walk_distance --emit-metrics-json
```

The `0.73` setting is a diagnostic simulator sensitivity check. No gait, protocol, visualiser, servo, or safety default is changed by this document.

For an opt-in full-rate reach/contact/joint trace from the repository root, add `HEXAPOD_WALK_CAPACITY_TRACE=1` to one isolated test invocation above. JSON records named `walk_capacity_trace` go to stderr; `--emit-metrics-json` continues to emit the aggregate census on stdout. This controller-side trace has no motor torque; use the simulator-side trace below for that quantity.

For actual motor effort on one isolated case, from the repository root with rebuilt simulator and server test binaries:

```bash
set -o pipefail
source scripts/lib/pinocchio_env.sh
export HEXAPOD_PHYSICS_SIM_EXE=/home/volly/pico/hexapod/hexapod-physics-sim/build/hexapod-physics-sim
HEXAPOD_WALK_TEST_CASE=forward_walk HEXAPOD_WALK_TEST_CHILD_STDIO=1 \
HEXAPOD_PROXIMAL_TRACE_SERVO_CAPACITY=1 \
  hexapod-server/build-tests/test_physics_sim_walk_distance --emit-metrics-json 2>&1 \
  | python3 scripts/analyze_locomotion_capacity.py
```

The analyzer reads only samples between the test's walk-window markers. It ignores stand warmup and records recovered/held steps as skipped rather than mislabeling them healthy.

For the foothold boundary census, run one isolated forward case with `HEXAPOD_FOOTHOLD_REACH_TRACE=1` and pipe combined output to `python3 scripts/analyze_foothold_reach.py`. The report groups each leg's late swing and stance entry, plus paired target jumps at both phase boundaries. It is a diagnostic on the production settings; the rejected actual-start screen is not left in the tree.

For the physical sphere/load audit, build the simulator and the walk-distance test together, then run one isolated forward case with `HEXAPOD_WALK_TEST_CHILD_STDIO=1 HEXAPOD_FOOT_PHYSICS_TRACE=1 HEXAPOD_FOOTHOLD_REACH_TRACE=1`. Pipe combined output to `python3 scripts/analyze_foot_physics.py`. Repeat with `HEXAPOD_SWING_CONTACT_CLEARANCE_SCREEN=0` and `=1`; do not compare a trace from one binary to a differently built plant. The foot trace is deliberately absent from legacy-only builds.

For the yaw-boundary audit, run `test_locomotion_regression_suite --case clearance_yaw_transition_probe --emit-metrics-json` with `HEXAPOD_LOCOMOTION_CHILD_STDIO=1 HEXAPOD_FOOTHOLD_REACH_TRACE=1` and pipe combined output to `python3 scripts/analyze_contact_screen_transitions.py`. The case is explicit-only so it does not alter canonical or stress profile membership.
