@echo off
REM Download ~/ros2_ws/src/AU516/au516_nav from the Pi and REPLACE the local copy
REM Local copy: <repo>\ros2_ws\src\AU516\au516_nav
REM Requires PuTTY (pscp.exe) installed in C:\Program Files\PuTTY

setlocal
set "PI_HOST=pi@192.168.5.1"
set "PI_PASS=8AphJtr?"
set "REMOTE_DIR=/home/pi/ros2_ws/src/AU516/au516_nav"
set "PSCP=C:\Program Files\PuTTY\pscp.exe"

set "LOCAL_PARENT=%~dp0..\ros2_ws\src\AU516"
set "LOCAL_DIR=%LOCAL_PARENT%\au516_nav"

if not exist "%PSCP%" (
    echo pscp.exe not found at "%PSCP%". Install PuTTY.
    pause
    exit /b 1
)

echo This will DELETE "%LOCAL_DIR%" and replace it with the Pi's version.
choice /M "Continue"
if errorlevel 2 exit /b 0

if exist "%LOCAL_DIR%" rmdir /S /Q "%LOCAL_DIR%"
if not exist "%LOCAL_PARENT%" mkdir "%LOCAL_PARENT%"

echo Downloading from %PI_HOST%:%REMOTE_DIR% ...
"%PSCP%" -r -pw "%PI_PASS%" "%PI_HOST%:%REMOTE_DIR%" "%LOCAL_PARENT%"
if errorlevel 1 (
    echo.
    echo Download FAILED.
    pause
    exit /b 1
)

echo.
echo Done. Files are in "%LOCAL_DIR%"
pause
