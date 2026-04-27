Canonical latest baselines (current tuned config):
  LATEST_hexapod_stability_baseline.txt
      — alias of latest stability baseline
  LATEST_resource_profile.txt
      — alias of latest full resource profile
  LATEST_hexapod_stability_baseline_40iter_2substep_stride3.txt
      — explicit config-tagged stability baseline (same content as alias)
  LATEST_resource_profile_40iter_2substep_stride3.txt
      — explicit config-tagged resource profile (same content as alias)

Historical snapshots:
  Keep dated/config-tagged files for comparisons (iter/substep/stride sweeps).
  Avoid using date-only or mismatched config names as canonical aliases.
  Prefer: `<artifact>_<config>_<YYYY-MM-DD>.txt` (e.g. `..._iter40_2026-04-26.txt`).

Notes:
  `resource_profile_hexapod_20iter_1substep_2026-04-26.txt` and
  `hexapod_stability_baseline_2026-04-26.txt` were retired because their names no
  longer matched the contained 40iter/2substep/stride3 data.
