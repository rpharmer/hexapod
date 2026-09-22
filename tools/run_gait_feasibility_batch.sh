#!/usr/bin/env bash
# A/B batch for the gait-feasibility screens. One process per run, separate
# stdout/stderr, never overwrites an existing run file.
#
# Usage: tools/run_gait_feasibility_batch.sh <out-dir> <runs> <arm> [arm...]
set -u

root=/home/volly/pico/hexapod
out=${1:?output dir}
runs=${2:?runs}
shift 2
arms=("$@")
mkdir -p "$out"

for screen in reverse_walk turn_in_place sequential; do
  for arm in "${arms[@]}"; do
    for i in $(seq 1 "$runs"); do
      stem="$out/$screen-$arm-$i"
      [ -f "$stem.stdout" ] && continue
      "$root/tools/run_gait_feasibility_screen.sh" "$out" "$screen" "$arm" \
        > "$stem.stdout" 2> "$stem.stderr"
      status=$?
      printf '%s\n' "$status" > "$stem.exit"
      echo "done $screen $arm $i exit=$status"
    done
  done
done

for arm in "${arms[@]}"; do
  stem="$out/aggressive_governor-$arm-1"
  [ -f "$stem.stdout" ] && continue
  "$root/tools/run_gait_feasibility_screen.sh" "$out" aggressive_governor "$arm" \
    > "$stem.stdout" 2> "$stem.stderr"
  status=$?
  printf '%s\n' "$status" > "$stem.exit"
  echo "done aggressive_governor $arm exit=$status"
done
