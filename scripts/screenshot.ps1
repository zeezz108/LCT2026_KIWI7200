# Capture the primary screen after a delay (used to document live RViz2 demos).
# usage: powershell -File scripts/screenshot.ps1 -Out <png> [-DelaySeconds 20] [-Scale 0.7]
param(
  [Parameter(Mandatory = $true)][string]$Out,
  [int]$DelaySeconds = 20,
  [double]$Scale = 0.7
)
Start-Sleep -Seconds $DelaySeconds
Add-Type -AssemblyName System.Windows.Forms, System.Drawing
$b = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
$bmp = New-Object System.Drawing.Bitmap $b.Width, $b.Height
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($b.Location, [System.Drawing.Point]::Empty, $b.Size)
$small = New-Object System.Drawing.Bitmap $bmp, ([int]($b.Width * $Scale)), ([int]($b.Height * $Scale))
$small.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
Write-Output "saved $Out"
