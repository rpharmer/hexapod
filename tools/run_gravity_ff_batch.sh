#!/usr/bin/env bash
# A/B batch for the walk-distance gravity-FF screen. One process per run,
# separate stdout/stderr, never overwrites an existing run file.
set -u

root=/home/volly/pico/hexapod
out=${1:?output dir}
runs=${2:-5}
mkdir -p "$out"

for screen in reverse_walk turn_in_place sequential; do
  for arm in baseline candidate; do
    for i in $(seq 1 "$runs"); do
      stem="$out/$screen-$arm-$i"
      [ -f "$stem.stdout" ] && continue
      "$root/tools/run_gravity_ff_screen.sh" "$out" "$screen" "$arm" \
        > "$stem.stdout" 2> "$stem.stderr"
      echo "done $screen $arm $i exit=$?"
    done
  done
done

for arm in baseline candidate; do
  stem="$out/aggressive_governor-$arm-1"
  [ -f "$stem.stdout" ] && continue
  "$root/tools/run_gravity_ff_screen.sh" "$out" aggressive_governor "$arm" \
    > "$stem.stdout" 2> "$stem.stderr"
  echo "done aggressive_governor $arm exit=$?"
done
