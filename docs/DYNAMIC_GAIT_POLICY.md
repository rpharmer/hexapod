# Dynamic Gait Policy

## Purpose

This document defines the command-space regions and gait-selection policy used to map a normalized locomotion command into a stable gait family, target contact ratio, and safety envelope.

The policy is designed for online use in the control loop and should be deterministic for any given `(speed, yaw_rate)` command pair.

---

## 1. Command Regions

Let:

- `v` = normalized translational speed command in `[0.0, 1.0]`
- `w` = normalized signed yaw command in `[-1.0, 1.0]`
- `|w|` = yaw magnitude

The mode classifier assigns one of three regions:

### `arc`

Used when translation dominates over turning.

**Entry condition**

- `v >= 0.20`
- `|w| <= 0.55`

**Intent**

- Forward locomotion with gradual/medium curvature.
- Maximize stability at low/moderate speed and throughput at high speed.

### `pivot`

Used when turning dominates and translational intent is low.

**Entry condition**

- `v <= 0.25`
- `|w| >= 0.60`

**Intent**

- In-place or near-in-place heading changes.
- Keep COM projected over support polygon with conservative duty factor.

### `reorientation`

Blend region between `arc` and `pivot` to avoid abrupt gait/mode switches.

**Entry condition**

- All commands not matching `arc` or `pivot`.

**Intent**

- Transitional turning while preserving forward progress.
- Smoothly blend contact ratio and gait family selection.

### Region Hysteresis

To prevent chattering at boundaries, use hysteresis on region transitions:

- `arc -> reorientation` only when `|w| > 0.60` or `v < 0.16`
- `pivot -> reorientation` only when `|w| < 0.52` and `v > 0.18`
- `reorientation -> arc` when `v >= 0.22` and `|w| <= 0.52`
- `reorientation -> pivot` when `v <= 0.22` and `|w| >= 0.60`

---

## 2. Gait-Family Map (Wave / Ripple / Tripod)

After region classification, choose gait family from the map below. If two rules match, pick the one listed first.

### In `arc`

1. **Wave gait** when `v < 0.35`
2. **Ripple gait** when `0.35 <= v < 0.70`
3. **Tripod gait** when `v >= 0.70` and `|w| <= 0.35`
4. **Ripple gait fallback** when `v >= 0.70` and `|w| > 0.35`

Rationale: high-speed curved motion is less robust in pure tripod than in ripple, so ripple is preferred for high yaw load.

### In `pivot`

1. **Wave gait** default
2. **Ripple gait** allowed only when `v >= 0.15` and `|w| <= 0.85`

Tripod is disallowed in pivot mode.

### In `reorientation`

1. **Wave gait** when `v < 0.30` or `|w| > 0.75`
2. **Ripple gait** otherwise

Tripod is disallowed in reorientation mode.

---

## 3. Desired Contact Ratio vs Speed and Yaw

Define desired contact ratio `CR*` as the target fraction of cycle time each leg remains in stance.

- Higher `CR*` => more stability, lower speed.
- Lower `CR*` => more dynamic movement, higher speed.

Compute:

- `s = clamp(v, 0.0, 1.0)`
- `y = clamp(|w|, 0.0, 1.0)`

Baseline contact ratio:

- `CR_base = 0.78 - 0.24 * s`

Yaw stability premium:

- `CR_yaw = 0.10 * y`

Desired:

- `CR* = clamp(CR_base + CR_yaw, 0.50, 0.88)`

Examples:

- Fast straight motion (`s=1.0`, `y=0.0`) -> `CR* = 0.54`
- Slow sharp turn (`s=0.2`, `y=0.9`) -> `CR* = 0.82`
- Mid-speed arc (`s=0.5`, `y=0.4`) -> `CR* = 0.70`

### Family-Specific Contact Ratio Clamps

After selecting gait family, apply family clamp:

- **Wave:** `CR = clamp(CR*, 0.72, 0.88)`
- **Ripple:** `CR = clamp(CR*, 0.60, 0.80)`
- **Tripod:** `CR = clamp(CR*, 0.50, 0.68)`

If family clamp saturates by more than `0.10`, prefer downshifting family:

- `tripod -> ripple`
- `ripple -> wave`

---

## 4. Per-Mode Safety Limits and Fallback Behavior

Safety checks run each control tick. The first violated condition triggers fallback.

## 4.1 Arc Mode Safety

### Limits

- Max normalized speed command: `v <= 1.00`
- Max yaw for tripod execution: `|w| <= 0.35`
- Max body roll/pitch estimate for nominal operation: `8 deg`
- Minimum foothold confidence: `0.70`

### Fallbacks

1. If roll/pitch exceeds `8 deg` for `>150 ms`: **tripod/ripple -> wave**, reduce step length by `30%`.
2. If foothold confidence `<0.70`: increase `CR` by `+0.08` (clamped), reduce swing height by `15%`.
3. If confidence `<0.50` for `>250 ms`: transition to **reorientation/wave** at `v=0.20` cap.

## 4.2 Pivot Mode Safety

### Limits

- Tripod forbidden.
- Max effective yaw command: `|w| <= 0.90` (clip above this).
- Max allowed translational bleed-through: `v <= 0.30`
- Max body roll/pitch estimate: `10 deg`

### Fallbacks

1. If roll/pitch exceeds `10 deg` for `>100 ms`: cut yaw command by `40%`, force **wave**.
2. If slip detector triggers on `>=2` legs in one cycle: pause progression for one half-cycle and re-phase.
3. If repeated slip (`3` consecutive detections): command **safe stop** and require operator reset or automatic recovery gate.

## 4.3 Reorientation Mode Safety

### Limits

- Allowed families: wave/ripple only.
- Max normalized speed: `v <= 0.60`
- Max yaw: `|w| <= 0.85`
- Max body roll/pitch estimate: `9 deg`

### Fallbacks

1. If roll/pitch exceeds `9 deg`: force **wave**, `CR >= 0.76`, reduce both step length and yaw by `25%`.
2. If command oscillates across signs of yaw at `>3 Hz` for `>300 ms`: hold yaw at `0` for one cycle and remain in wave.
3. If estimator freshness timeout occurs: enter **degraded hold** (stance lock, zero swing), publish fault, and wait for freshness recovery.

---

## 5. Global Fallback Ladder

When any mode-specific fallback cannot restore constraints within `500 ms`, escalate:

1. **Stability fallback:** wave gait, high contact ratio (`CR=0.84`), reduced stride.
2. **Degraded locomotion:** cap `v <= 0.15`, `|w| <= 0.25`.
3. **Safe stop:** settle all legs to stance and freeze gait phase.
4. **Faulted hold:** require explicit supervisor clear to re-enable dynamic gait.

---

## 6. Implementation Notes

- Apply region/gait decisions at a fixed cadence (recommended `50-100 Hz`).
- Slew-limit changes in contact ratio (recommended `|dCR/dt| <= 0.8 s^-1`).
- On gait-family changes, phase-align over one cycle rather than hard-switching phase.
- Telemetry should emit region, family, `CR*`, clamped `CR`, and active safety fallback state for observability.
