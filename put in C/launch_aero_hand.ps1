param(
    [string]$HandBusId = "1-11",
    [string]$CameraBusId = "1-3",
    [string]$Distro = "Ubuntu-22.04"
)

# ===== USER SETTINGS =====
$Workspace = "/home/will/chestnut/aero-hand-open/ros2"

Write-Host "Using hand BUSID:   $HandBusId"
Write-Host "Using camera BUSID: $CameraBusId"
Write-Host "Using WSL distro:   $Distro"

# ===== ATTACH USB DEVICES TO WSL =====
Write-Host "Attaching Aero Hand USB device..."
usbipd attach --wsl --busid $HandBusId

Write-Host "Attaching camera USB device..."
usbipd attach --wsl --busid $CameraBusId

Start-Sleep -Seconds 2

# ===== LAUNCH ROS INSIDE WSL =====
$LinuxCommand = @"
cd $Workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Temporary permissions after USB reattach
sudo chmod a+rw /dev/ttyUSB* 2>/dev/null || true
sudo chmod a+rw /dev/ttyACM* 2>/dev/null || true
sudo chmod a+rw /dev/video* 2>/dev/null || true

# Optional camera format fix
if command -v v4l2-ctl >/dev/null 2>&1 && [ -e /dev/video0 ]; then
    v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG --set-parm=30 || true
fi

ros2 launch src/launch_files/webcam_teleop_launch/webcam_teleop.launch.py right_hand_port:=auto left_hand_port:=auto feedback_frequency:=50.0
"@

# Remove Windows CRLF characters before sending text to bash.
$LinuxCommand = $LinuxCommand -replace "`r", ""

wsl -d $Distro -- bash -lc $LinuxCommand

# Detach devices when ROS launch exits.
usbipd detach --busid $CameraBusId
usbipd detach --busid $HandBusId
