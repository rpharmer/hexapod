#!/usr/bin/env bash

# Intended to be sourced by project scripts. It scopes dependency paths to the
# current process and deliberately does not edit ~/.bashrc or another profile.
PINOCCHIO_ENV_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PINOCCHIO_ENV_ROOT="$(cd "$PINOCCHIO_ENV_SCRIPT_DIR/../.." && pwd)"

pinocchio_prefixes=()
if [[ -d /opt/openrobots ]]; then
  pinocchio_prefixes+=(/opt/openrobots)
fi
if [[ -d "$PINOCCHIO_ENV_ROOT/.deps/pinocchio/cmeel.prefix" ]]; then
  pinocchio_prefixes+=("$PINOCCHIO_ENV_ROOT/.deps/pinocchio/cmeel.prefix")
fi
while IFS= read -r prefix; do
  pinocchio_prefixes+=("$prefix")
done < <(find "$PINOCCHIO_ENV_ROOT/.deps/pinocchio" -type d \
  -path '*/site-packages/cmeel.prefix' -print 2>/dev/null | sort)

for prefix in "${pinocchio_prefixes[@]}"; do
  export CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export PKG_CONFIG_PATH="$prefix/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
  export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
done

while IFS= read -r site_packages; do
  export PYTHONPATH="$site_packages${PYTHONPATH:+:$PYTHONPATH}"
done < <(find "$PINOCCHIO_ENV_ROOT/.deps/pinocchio" -type d \
  -path '*/site-packages' -print 2>/dev/null | sort)

unset PINOCCHIO_ENV_SCRIPT_DIR PINOCCHIO_ENV_ROOT prefix site_packages
unset pinocchio_prefixes
