#!/usr/bin/env bash
# Diagnose WSLg GUI health for hexapod-opengl-visualiser.
# Exit 0 = OK, 1 = COPY MODE / broken shared memory (windows often invisible).
set -uo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

if ! grep -qi microsoft /proc/version 2>/dev/null; then
  echo "Not WSL; nothing to check."
  exit 0
fi

echo "uname: $(uname -r)"
echo

echo "=== /mnt/shared_memory (user distro) ==="
findmnt /mnt/shared_memory 2>/dev/null || echo "(not a separate mount — often means virtiofs never attached here)"
ls -la /mnt/shared_memory 2>&1 | head -10
echo

echo "=== weston COPY / shared-memory lines ==="
if [[ -f /mnt/wslg/weston.log ]]; then
  grep -iE 'shared_memory|copy_warning|rdp_allocate|use_gfxredir' /mnt/wslg/weston.log | tail -20 || true
else
  echo "no /mnt/wslg/weston.log"
fi
echo

echo "=== system-distro virtiofs probe (requires wsl.exe --system) ==="
if command -v wsl.exe >/dev/null 2>&1; then
  wsl.exe --system -- bash -c 'findmnt /mnt/shared_memory; ls /mnt/shared_memory 2>&1 | head' || true
else
  echo "wsl.exe not on PATH"
fi
echo

if wslg_copy_mode_active; then
  warn_if_wslg_copy_mode
  echo
  echo "Additional check from Windows PowerShell (system distro):"
  echo "  wsl --system -- ls /mnt/shared_memory"
  echo "If that prints 'Function not implemented', WSLg shared-memory is broken."
  echo
  echo "Workarounds:"
  echo "  1) Update WSL:  wsl --update   (or Microsoft Store pre-release)"
  echo "  2) Bypass WSLg with an external X server (VcXsrv/X410):"
  echo "       - Install VcXsrv, start with 'Multiple windows' + Disable access control"
  echo "       - In .wslconfig set:  guiApplications=false"
  echo "       - wsl --shutdown, reopen, then:"
  echo "           export DISPLAY=\$(ip route show default | awk '/default/ {print \$3}'):0.0"
  echo "           ./scripts/run_visualiser.sh --skip-build -- --udp-port 9870"
  echo "  3) Focus helpers cannot fix invisible COPY MODE surfaces."
  exit 1
fi

echo "No COPY MODE warning in weston.log — WSLg GUI path looks healthy."
exit 0
