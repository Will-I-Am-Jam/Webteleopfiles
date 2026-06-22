@echo off
set "HAND_BUSID=%~1"
set "CAMERA_BUSID=%~2"

if "%HAND_BUSID%"=="" set "HAND_BUSID=1-11"
if "%CAMERA_BUSID%"=="" set "CAMERA_BUSID=1-3"

powershell -ExecutionPolicy Bypass -File "C:\aero_launcher\launch_aero_hand.ps1" -HandBusId "%HAND_BUSID%" -CameraBusId "%CAMERA_BUSID%"
pause
