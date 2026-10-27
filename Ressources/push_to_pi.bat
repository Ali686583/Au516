@echo off
REM Upload the local au516_nav folder to ~/ros2_ws/src/AU516/au516_nav on the Pi
REM Local copy: <repo>\ros2_ws\src\AU516\au516_nav
REM Existing files on the Pi are overwritten; files that only exist on the Pi are kept.
REM Requires PuTTY (pscp.exe) installed in C:\Program Files\PuTTY

setlocal
set "PI_HOST=pi@192.168.5.1"
set "PI_PASS=8AphJtr?"
set "REMOTE_PARENT=/home/pi/ros2_ws/src/AU516"
set "PSCP=C:\Program Files\PuTTY\pscp.exe"

set "LOCAL_DIR=%~dp0..\ros2_ws\src\AU516\au516_nav"

if not exist "%PSCP%" (
    echo pscp.exe not found at "%PSCP%". Install PuTTY.
    pause
    exit /b 1
)

if not exist "%LOCAL_DIR%" (
    echo Local folder "%LOCAL_DIR%" not found. Run pull_from_pi.bat first.
    pause
    exit /b 1
)

echo Uploading "%LOCAL_DIR%" to %PI_HOST%:%REMOTE_PARENT%/au516_nav ...
"%PSCP%" -r -pw "%PI_PASS%" "%LOCAL_DIR%" "%PI_HOST%:%REMOTE_PARENT%/"
if errorlevel 1 (
    echo.
    echo Upload FAILED.
    pause
    exit /b 1
)

echo.
echo Done. Remember to rebuild on the Pi: cd ~/ros2_ws ^&^& colcon build --packages-select au516_nav
pause
