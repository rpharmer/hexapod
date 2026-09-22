#Requires -Version 5.1
<#
.SYNOPSIS
  Bind/attach the Xbox Wireless Adapter (045e:02e6) into WSL2 for hexapod-server.

.DESCRIPTION
  Daily helper for the usbipd + xone path documented in docs/WSL_XBOX_CONTROLLER.md.

  CRITICAL: power the Xbox pad OFF before running this script. If Windows still has
  an active wireless session, attach fails with "Device busy". After attach succeeds,
  turn the pad back ON — pairing stored on the dongle should reconnect under xone.

.PARAMETER BusId
  usbipd bus id (default: auto-detect 045e:02e6).

.PARAMETER Force
  Use `usbipd bind --force` when Windows owns the adapter.

.EXAMPLE
  # From an elevated PowerShell (bind --force needs admin):
  .\scripts\attach_xbox_wsl.ps1

.EXAMPLE
  .\scripts\attach_xbox_wsl.ps1 -BusId 1-1 -Force
#>
[CmdletBinding()]
param(
    [string]$BusId = "",
    [switch]$Force
)

$ErrorActionPreference = "Stop"

function Get-XboxAdapterBusId {
    $lines = usbipd list 2>&1 | Out-String
    foreach ($line in ($lines -split "`r?`n")) {
        if ($line -match '^\s*(\d+-\d+)\s+045e:02e6\b') {
            return $Matches[1]
        }
    }
    return $null
}

function Show-List {
    Write-Host ""
    usbipd list
    Write-Host ""
}

Write-Host "=== Xbox Wireless Adapter → WSL ==="
Write-Host "1) Turn the Xbox controller OFF (hold Xbox button until it powers down)."
Write-Host "2) This script will bind/attach the dongle into WSL."
Write-Host "3) After 'Attached', turn the controller ON again."
Write-Host ""

Show-List

if (-not $BusId) {
    $BusId = Get-XboxAdapterBusId
    if (-not $BusId) {
        throw "No 045e:02e6 Xbox Wireless Adapter found in 'usbipd list'."
    }
    Write-Host "Auto-detected bus id: $BusId"
}

$stateLine = (usbipd list 2>&1 | Out-String) -split "`r?`n" | Where-Object { $_ -match "^\s*$([regex]::Escape($BusId))\s+" } | Select-Object -First 1
Write-Host "Current: $stateLine"

if ($stateLine -match '\bAttached\b') {
    Write-Host "Already attached to WSL."
} else {
    $needsBind = $stateLine -match 'Not shared'
    $forceBind = $Force -or ($stateLine -match 'Shared \(forced\)') -or ($stateLine -match '\bShared\b' -and $stateLine -notmatch 'Attached')

    if ($needsBind -or $Force) {
        $bindArgs = @("bind", "--busid", $BusId)
        if ($Force -or $forceBind) {
            $bindArgs += "--force"
            Write-Host "Running: usbipd $($bindArgs -join ' ')  (admin may be required)"
        } else {
            Write-Host "Running: usbipd $($bindArgs -join ' ')  (admin may be required)"
        }
        & usbipd @bindArgs
        if ($LASTEXITCODE -ne 0) {
            throw "usbipd bind failed (exit $LASTEXITCODE). Re-run from elevated PowerShell, or pass -Force."
        }
    }

    Write-Host "Running: usbipd attach --wsl --busid $BusId"
    & usbipd attach --wsl --busid $BusId
    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "Attach failed. Most common cause: controller still ON / Windows holding the dongle."
        Write-Host "  - Power the pad OFF completely"
        Write-Host "  - Re-run:  .\scripts\attach_xbox_wsl.ps1 -Force"
        Write-Host "  - If still busy, unplug/replug the dongle, then retry"
        throw "usbipd attach failed (exit $LASTEXITCODE)."
    }
}

Show-List

Write-Host "Waiting briefly for xone probe in WSL..."
Start-Sleep -Seconds 3

$wslCheck = @'
set -e
echo "kernel: $(uname -r)"
echo "modules:"; lsmod | grep -E '^xone' || true
echo "dmesg (xone/gip):"
dmesg 2>/dev/null | grep -iE 'xone|gip0|Microsoft Xbox' | tail -20 || true
echo "input nodes:"
ls -l /dev/input 2>/dev/null || echo "  /dev/input not present yet (turn the pad ON)"
if [ -r /proc/bus/input/devices ]; then
  echo "devices:"
  awk '
    BEGIN{RS=""; FS="\n"}
    /Microsoft Xbox|Xbox Controller|xpad/ {
      for (i=1;i<=NF;i++) if ($i ~ /^N:|^H:/) print $i
      print "---"
    }
  ' /proc/bus/input/devices
fi
'@

wsl.exe -d Ubuntu -- bash -lc $wslCheck

Write-Host ""
Write-Host "Next:"
Write-Host "  - Turn the Xbox controller ON"
Write-Host "  - Confirm event node:  wsl -d Ubuntu -- ls -l /dev/input"
Write-Host "  - Server:  --controller-device /dev/input/eventN"
Write-Host "  - Details: docs/WSL_XBOX_CONTROLLER.md"
