# Record a desktop window (e.g. RViz2 shown through WSLg) to MP4 with the ffmpeg bundled in the project venv.
# WSLg windows belong to msrdc.exe and are not visible as process main windows, so top-level windows are enumerated.
# usage: powershell -File scripts/record_window.ps1 -Out demo.mp4 [-Title "RViz"] [-Seconds 30] [-WaitSeconds 0]
param(
  [Parameter(Mandatory = $true)][string]$Out,
  [string]$Title = "RViz",
  [int]$Seconds = 30,
  [int]$WaitSeconds = 0,
  [int]$Fps = 15
)
Add-Type @"
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
public static class WinFind {
  public delegate bool EnumProc(IntPtr h, IntPtr l);
  [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc cb, IntPtr l);
  [DllImport("user32.dll")] static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
  public static List<Tuple<IntPtr, string>> Find(string part) {
    var res = new List<Tuple<IntPtr, string>>();
    EnumWindows((h, l) => {
      if (!IsWindowVisible(h)) return true;
      var sb = new StringBuilder(512); GetWindowText(h, sb, 512);
      var t = sb.ToString();
      if (t.IndexOf(part, StringComparison.OrdinalIgnoreCase) >= 0) res.Add(Tuple.Create(h, t));
      return true;
    }, IntPtr.Zero);
    return res;
  }
}
"@
$ffmpeg = & "$PSScriptRoot\..\.venv\Scripts\python.exe" -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())"
$deadline = (Get-Date).AddSeconds([Math]::Max($WaitSeconds, 1))
$found = $null
do {
  $found = [WinFind]::Find($Title) | Select-Object -First 1
  if ($found) { break }
  Start-Sleep -Milliseconds 500
} while ((Get-Date) -lt $deadline)
if (-not $found) { Write-Output "window '$Title' not found"; exit 1 }
$rect = New-Object WinFind+RECT
[void][WinFind]::GetWindowRect($found.Item1, [ref]$rect)
Add-Type -AssemblyName System.Windows.Forms
$screen = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
# clamp to the screen (maximised windows may start at negative coordinates)
$x0 = [Math]::Max(0, $rect.Left); $y0 = [Math]::Max(0, $rect.Top)
$x1 = [Math]::Min($screen.Width, $rect.Right); $y1 = [Math]::Min($screen.Height, $rect.Bottom)
$w = ($x1 - $x0) - (($x1 - $x0) % 2)
$h = ($y1 - $y0) - (($y1 - $y0) % 2)
Write-Output "recording '$($found.Item2)' at $x0,$y0 ${w}x${h}"
& $ffmpeg -y -loglevel error -f gdigrab -framerate $Fps -offset_x $x0 -offset_y $y0 -video_size "${w}x${h}" -i desktop -t $Seconds -c:v libx264 -preset veryfast -pix_fmt yuv420p $Out
Write-Output "saved $Out"
