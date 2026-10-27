# Download ~/ros2_ws/src/AU516/au516_nav from the Pi and REPLACE the local copy
# Local copy: <repo>\Raspberry\au516_nav (resolved relative to this script)
# Requires PuTTY (pscp.exe) installed in C:\Program Files\PuTTY

$PiHost    = "pi@192.168.5.1"
$PiPass    = "8AphJtr?"
$RemoteDir = "/home/pi/ros2_ws/src/AU516/au516_nav"
$Pscp      = "C:\Program Files\PuTTY\pscp.exe"

$LocalParent = Join-Path $PSScriptRoot "..\Raspberry"
$LocalDir    = Join-Path $LocalParent "au516_nav"

if (-not (Test-Path $Pscp)) {
    Write-Host "pscp.exe not found at `"$Pscp`". Install PuTTY." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "This will DELETE `"$LocalDir`" and replace it with the Pi's version."
$answer = Read-Host "Continue? [Y/N]"
if ($answer -notmatch '^[Yy]') { exit 0 }

if (Test-Path $LocalDir) { Remove-Item -Recurse -Force $LocalDir }
if (-not (Test-Path $LocalParent)) { New-Item -ItemType Directory -Force $LocalParent | Out-Null }

Write-Host "Downloading from ${PiHost}:$RemoteDir ..."
& $Pscp -r -pw $PiPass "${PiHost}:$RemoteDir" $LocalParent
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "Download FAILED." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host ""
Write-Host "Done. Files are in `"$LocalDir`"" -ForegroundColor Green
Read-Host "Press Enter to exit"
