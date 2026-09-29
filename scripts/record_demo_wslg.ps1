# Record a live demo on Windows + WSLg: detector and RViz2 run in WSL, the RViz2 window is captured with ffmpeg (gdigrab).
# The RViz2 window must stay on top and unobstructed while recording (gdigrab captures the screen region).
# usage: powershell -File scripts/record_demo_wslg.ps1 -Bag /home/dev/data/for_hackathon/doubleT_obstacle -Out docs/video/obstacle.mp4
#          [-Seconds 22] [-Rate 1.0] [-Offset 0] [-WarmupSeconds 10]
param(
  [Parameter(Mandatory = $true)][string]$Bag,
  [Parameter(Mandatory = $true)][string]$Out,
  [int]$Seconds = 22,
  [double]$Rate = 1.0,
  [double]$Offset = 0.0,
  [int]$WarmupSeconds = 10
)
$ErrorActionPreference = "Stop"
$env:WSL_UTF8 = "1"
$ros = "source /opt/ros/humble/setup.bash && source ~/lct_ws/install/setup.bash && export OMP_WAIT_POLICY=PASSIVE OMP_NUM_THREADS=4"

function Stop-Demo {
  # bracket patterns do not match the pkill command line itself
  wsl.exe -d Ubuntu-22.04 -- bash -c "pkill -f '[r]os2 bag play'; pkill -f '[r]viz2'; pkill -f '[d]etector_node'; pkill -f '[r]os2 launch'; true" | Out-Null
}

Stop-Demo
Write-Output "starting detector + RViz2"
$launchCmd = "$ros && ros2 launch tunnel_obstacle_detector detector.launch.py rviz:=true > /tmp/demo_launch.log 2>&1"
# Windows PowerShell does not quote ArgumentList items: pass one pre-quoted string
$launch = Start-Process -FilePath wsl.exe -WindowStyle Hidden -PassThru -ArgumentList "-d Ubuntu-22.04 -- bash -lc `"$launchCmd`""
Start-Sleep -Seconds $WarmupSeconds

$outPath = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $Out))
New-Item -ItemType Directory -Force -Path (Split-Path $outPath) | Out-Null
$recorder = Start-Process -FilePath powershell.exe -WindowStyle Hidden -PassThru -ArgumentList (
  "-NoProfile -ExecutionPolicy Bypass -File `"$PSScriptRoot\record_window.ps1`" -Out `"$outPath`" -Title RViz " +
  "-Seconds $Seconds -WaitSeconds 30")
Start-Sleep -Seconds 2

Write-Output "playing $Bag"
wsl.exe -d Ubuntu-22.04 -- bash -lc "$ros && timeout $($Seconds + 5) ros2 bag play '$Bag' --rate $Rate --start-offset $Offset > /tmp/demo_play.log 2>&1; true"

$recorder.WaitForExit()
Stop-Demo
wsl.exe -d Ubuntu-22.04 -- bash -c "grep -E 'fps|DANGER|forward axis' /tmp/demo_launch.log | tail -6"
Write-Output "saved $outPath"
