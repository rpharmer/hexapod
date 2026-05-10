# Physics Sim Robustness Follow-Up

## Summary

The original unsupported-articulation blow-up appears substantially improved by the recent
`hexapod-physics-sim` fixes, but later investigation while tuning server locomotion still exposed
some runs with suspiciously unphysical behavior. This note records the observed symptoms so they can
be revisited separately from the current server-side gait work.

## What Was Observed

- Some aggressive server-side gait experiments produced motion that did not look like a plausible
  fall or trip, including:
  - extremely large path length / displacement values relative to the scenario intent
  - body-rate saturation near the sim-side caps
  - apparent runaway motion after the controller had already entered a bad support state
- These behaviors were not the same as the original "robot launches into the air immediately"
  failure, but they still suggest the sim may become numerically or physically fragile once the
  controller drives it into a pathological regime.

## Important Distinction

- The current evidence suggests the **primary remaining issue** is on the server side:
  contact semantics and fast-tripod gait / transition behavior can still drive the robot into a
  later genuine sparse-support failure.
- Separately, the sim may still have a **secondary robustness issue** because some bad-controller
  cases appear able to trigger motion that looks nonphysical rather than like a bounded collapse.

## Example Investigation Artifacts

- `test_locomotion_regression_suite` runs that produced suspicious behavior during server tuning:
  - `/tmp/longwalk-tipover-touchdown-recovery`
  - `/tmp/longwalk-tipover-touchdown-floor`
  - `/tmp/longwalk-tipover-tripod-bias-2`
- These are useful as "bad controller drives sim into questionable state" replay cases, even if the
  associated server changes are not meant to land.

## Suggested Later Investigation

- Re-run the suspicious replay cases against the current fixed sim baseline and inspect:
  - body-rate saturation
  - planar speed saturation
  - contact counts and fused support around the onset of runaway motion
  - body/foot world positions for obviously nonphysical integration
- Determine whether the remaining issue is:
  - a real server instability that only looks dramatic but is still physically plausible, or
  - a sim robustness problem that should degrade into a bounded collapse instead of runaway motion
- Keep this work separate from the current server `TIP_OVER` / gait-transition fixes so the two
  problems do not get conflated again.
