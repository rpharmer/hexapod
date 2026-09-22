#Requires -Version 5.1
<#
.SYNOPSIS
  Find a WSLg/RDP-hosted window by title substring and force it to the foreground.

.DESCRIPTION
  WScript.Shell AppActivate only matches titles that *start with* the given string.
  WSLg often prefixes GUI windows with "[WARN:COPY MODE] ...", so activating
  "Hexapod OpenGL Visualiser" fails even when the window exists under msrdc.exe.

  This helper enumerates top-level windows, matches a substring, then
  ShowWindow(SW_RESTORE) + BringWindowToTop + SetForegroundWindow.

  Use -HoldSeconds so focus is re-asserted after the launching terminal / server
  steals it back (common with scripts/run_physics_stack.sh).
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$TitleSubstring,

    [int]$Attempts = 80,

    [int]$DelayMs = 250,

    # After the first successful raise, keep re-raising for this many extra seconds.
    [double]$HoldSeconds = 0,

    [string]$LogFile = ""
)

$ErrorActionPreference = "Stop"

function Write-RaiseLog([string]$Message) {
    $line = "[{0}] {1}" -f (Get-Date -Format "HH:mm:ss.fff"), $Message
    if ($LogFile) {
        Add-Content -Path $LogFile -Value $line
    }
    Write-Output $line
}

Add-Type @"
using System;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;

public class WslgWindowRaise {
  public delegate bool EnumProc(IntPtr hWnd, IntPtr lParam);

  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr lParam);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hWnd);
  [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr hWnd);
  [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr hWnd, StringBuilder sb, int max);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
  [DllImport("user32.dll")] public static extern bool BringWindowToTop(IntPtr hWnd);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);
  [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
  [DllImport("user32.dll")] public static extern void keybd_event(byte bVk, byte bScan, uint dwFlags, UIntPtr dwExtraInfo);

  public const int SW_RESTORE = 9;
  public const int SW_SHOW = 5;
  public static readonly IntPtr HWND_TOPMOST = new IntPtr(-1);
  public static readonly IntPtr HWND_NOTOPMOST = new IntPtr(-2);
  public const uint SWP_NOMOVE = 0x0002;
  public const uint SWP_NOSIZE = 0x0001;
  public const uint SWP_SHOWWINDOW = 0x0040;
  public const byte VK_MENU = 0x12;
  public const uint KEYEVENTF_KEYUP = 0x0002;

  public static List<IntPtr> FindVisible(string needle) {
    var hits = new List<IntPtr>();
    EnumWindows((h, l) => {
      if (!IsWindowVisible(h)) return true;
      var sb = new StringBuilder(512);
      GetWindowText(h, sb, sb.Capacity);
      var title = sb.ToString();
      if (!string.IsNullOrEmpty(title) &&
          title.IndexOf(needle, StringComparison.OrdinalIgnoreCase) >= 0) {
        hits.Add(h);
      }
      return true;
    }, IntPtr.Zero);
    return hits;
  }

  // AttachThreadInput trick + ALT keybd_event helps bypass focus-stealing locks.
  public static bool Raise(IntPtr h) {
    if (IsIconic(h)) {
      ShowWindow(h, SW_RESTORE);
    } else {
      ShowWindow(h, SW_SHOW);
    }
    SetWindowPos(h, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
    // Synthetic ALT press allows SetForegroundWindow from a non-foreground process.
    keybd_event(VK_MENU, 0, 0, UIntPtr.Zero);
    keybd_event(VK_MENU, 0, KEYEVENTF_KEYUP, UIntPtr.Zero);
    BringWindowToTop(h);
    bool ok = SetForegroundWindow(h);
    SetWindowPos(h, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
    return ok || GetForegroundWindow() == h;
  }

  public static bool IsForeground(IntPtr h) {
    return GetForegroundWindow() == h;
  }
}
"@

$needle = $TitleSubstring
$firstRaiseAt = $null
$everRaised = $false
$deadline = [datetime]::UtcNow.AddMilliseconds($Attempts * [math]::Max($DelayMs, 1))
if ($HoldSeconds -gt 0) {
    # Allow hold window beyond the attempt budget.
    $deadline = [datetime]::UtcNow.AddSeconds([math]::Max($Attempts * $DelayMs / 1000.0, $HoldSeconds) + $HoldSeconds)
}

Write-RaiseLog "Looking for window containing '$needle' (attempts=$Attempts delayMs=$DelayMs holdSeconds=$HoldSeconds)"

for ($i = 0; $i -lt $Attempts -or ($HoldSeconds -gt 0 -and [datetime]::UtcNow -lt $deadline); $i++) {
    $hwnds = [WslgWindowRaise]::FindVisible($needle)
    if ($hwnds.Count -eq 0) {
        Start-Sleep -Milliseconds $DelayMs
        continue
    }

    foreach ($h in $hwnds) {
        $raised = [WslgWindowRaise]::Raise($h)
        $fg = [WslgWindowRaise]::IsForeground($h)
        if ($raised -or $fg) {
            $everRaised = $true
            if (-not $firstRaiseAt) {
                $firstRaiseAt = [datetime]::UtcNow
                Write-RaiseLog "Raised hwnd=$h (foreground=$fg); holding for $HoldSeconds s"
            }
            if ($HoldSeconds -le 0) {
                Write-RaiseLog "Done after first raise"
                exit 0
            }
            # Keep re-raising until hold elapses in case the terminal steals focus back.
            $holdDeadline = $firstRaiseAt.AddSeconds($HoldSeconds)
            while ([datetime]::UtcNow -lt $holdDeadline) {
                Start-Sleep -Milliseconds ([math]::Max($DelayMs, 200))
                if (-not [WslgWindowRaise]::IsForeground($h)) {
                    [void][WslgWindowRaise]::Raise($h)
                }
            }
            Write-RaiseLog "Hold complete; final foreground=$([WslgWindowRaise]::IsForeground($h))"
            exit 0
        }
    }
    Start-Sleep -Milliseconds $DelayMs
}

if ($everRaised) {
    Write-RaiseLog "Exiting after raise attempts"
    exit 0
}

Write-RaiseLog "No focusable window matching '$needle'"
Write-Error "No focusable window matching '$needle' after $Attempts attempts"
exit 1
