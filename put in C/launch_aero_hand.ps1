# ===== USER SETTINGS =====
$Distro = "Ubuntu-22.04"

# Change these to match usbipd list
$HandBusId = "1-11"
$CameraBusId = "1-3"

$Workspace = "/home/will/chestnut/aero-hand-open/ros2"

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

# Optional temporary permissions, useful after USB reattach
if ls /dev/serial/by-id/usb-Espressif* 1> /dev/null 2>&1; then
    PORT=`$(readlink -f /dev/serial/by-id/usb-Espressif*)
    sudo chmod a+rw "`$PORT"
fi

if ls /dev/video* 1> /dev/null 2>&1; then
    sudo chmod a+rw /dev/video*
fi

# Optional camera format fix
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG --set-parm=30 || true

ros2 launch src/launch_files/webcam_teleop_launch/webcam_teleop.launch.py right_hand_port:=auto left_hand_port:=auto feedback_frequency:=50.0
"@

$LinuxCommand = $LinuxCommand -replace "`r", ""

wsl -d $Distro -- bash -lc $LinuxCommand