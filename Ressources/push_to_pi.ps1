# Upload the local au516_nav folder to ~/ros2_ws/src/AU516/au516_nav on the Pi
# Local copy: <repo>\Raspberry\au516_nav (resolved relative to this script)
# Existing files on the Pi are overwritten; files that only exist on the Pi are kept.
# Requires PuTTY (pscp.exe) installed in C:\Program Files\PuTTY

$PiHost       = "pi@192.168.5.1"
$PiPass       = "8AphJtr?"
$RemoteParent = "/home/pi/ros2_ws/src/AU516"
$Pscp         = "C:\Program Files\PuTTY\pscp.exe"

$LocalDir = Join-Path $PSScriptRoot "..\Raspberry\au516_nav"

if (-not (Test-Path $Pscp)) {
    Write-Host "pscp.exe not found at `"$Pscp`". Install PuTTY." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

if (-not (Test-Path $LocalDir)) {
    Write-Host "Local folder `"$LocalDir`" not found. Run pull_from_pi.ps1 first." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "Uploading `"$LocalDir`" to ${PiHost}:$RemoteParent/au516_nav ..."
& $Pscp -r -pw $PiPass $LocalDir "${PiHost}:$RemoteParent/"
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "Upload FAILED." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host ""
Write-Host "Done. Remember to rebuild on the Pi: cd ~/ros2_ws && colcon build --packages-select au516_nav" -ForegroundColor Green
Read-Host "Press Enter to exit"
