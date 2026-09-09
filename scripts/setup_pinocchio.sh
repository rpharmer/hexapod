#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PINOCCHIO_VERSION="4.1.0"

# robotpkg publishes binary packages for the supported Ubuntu LTS releases. Keep
# those packages under /opt/openrobots, as documented upstream. Development WSL
# images can be newer than robotpkg; for those, use Pinocchio's official PyPI
# distribution in a repository-local prefix instead of modifying the shell profile.
ubuntu_version=""
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  ubuntu_version="${VERSION_ID:-}"
fi

case "$ubuntu_version" in
  20.04|22.04|24.04)
    sudo apt-get update
    sudo apt-get install -y curl lsb-release
    sudo install -d -m 0755 /etc/apt/keyrings
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
      | sudo tee /etc/apt/keyrings/robotpkg.asc >/dev/null
    printf 'deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub %s robotpkg\n' \
      "$(lsb_release -cs)" \
      | sudo tee /etc/apt/sources.list.d/robotpkg.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y 'robotpkg-py3*-pinocchio'
    ;;
  *)
    dependency_dir="$ROOT_DIR/.deps/pinocchio"
    mkdir -p "$dependency_dir"
    python3 -m pip install --upgrade --target "$dependency_dir" \
      "pin==$PINOCCHIO_VERSION" cmeel-eigen cmeel-urdfdom-headers
    ;;
esac

# Verify the version from whichever supported installation path was selected.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/pinocchio_env.sh"
python3 - <<'PY'
import pinocchio

parts = pinocchio.__version__.split('.')
if len(parts) < 2 or parts[:2] != ['4', '1']:
    raise SystemExit(f"Pinocchio 4.1.x required, found {pinocchio.__version__}")
print(f"Pinocchio {pinocchio.__version__} ready")
PY
