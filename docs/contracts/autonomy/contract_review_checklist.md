# Autonomy Contract Review Checklist

Use this checklist before accepting any `v1.x` contract updates.

- [ ] Fields use unambiguous SI units (or explicit exception documented).
- [ ] Every geometric/pose field carries explicit frame metadata.
- [ ] Timestamp policy is explicit (`timestamp_ms`, optional UTC, monotonicity assumptions).
- [ ] `sample_id` and `correlation_id` behavior documented.
- [ ] QoS class and stale-data TTL behavior documented.
- [ ] Failure semantics define severity and downstream expected behavior.
- [ ] Arbitration/priority behavior is explicit when module competes for motion control.
- [ ] Backward-compatibility statement included (`v1` vs `v1.x`).
