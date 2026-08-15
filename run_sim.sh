#!/bin/bash
set -e

# ========================================================
# Cleanup Function: Uses process grouping and names
# ========================================================
cleanup() {
  echo ""
  echo "==== Terminating all simulation instances... ===="
  
  # Kill ROS 2 and Gazebo processes
  pkill -f "mavros" || true
  pkill -f "parameter_bridge" || true
  kill $GZ_PID 2>/dev/null || true
  
  # Kill SITL launchers
  pkill -f "sim_vehicle.py" || true
  
  # NEW: Kill the actual ArduPilot binaries, MAVProxy, and xterm windows
  pkill -f "arducopter" || true
  pkill -f "mavproxy" || true
  pkill -x "xterm" || true
  
  echo "Done!"
  exit 0
}

# Trap signals for clean exit
trap cleanup SIGINT SIGTERM

echo "========================================================"
echo " Running Pre-Launch Cleanup & Network Config"
echo "========================================================"
# Added arducopter, mavproxy, and xterm to the pre-launch kill list
killall -9 gz sim_vehicle.py python3 mavros_node parameter_bridge arducopter mavproxy xterm 2>/dev/null || true

export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1
sleep 2

echo "========================================================"
echo " Booting Native Gazebo + ArduPilot GUI Stack"
echo "========================================================"

echo "-> Starting Gazebo Client Engine with NVIDIA GPU..."
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only gz sim -r robofest.sdf &
GZ_PID=$!
sleep 10

echo "-> Spawning ArduPilot Flight Controller windows (Swarm)..."

# Drone 1: Joy (Instance 0)
gnome-terminal --title="ArduPilot SITL - Joey" -- bash -c \
  "source ~/venv-ardupilot/bin/activate && /home/nikhil/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I 0 --sysid 1 --model JSON --out=udp:127.0.0.1:14555 -N; exec bash" &
sleep 3
echo "========================================================"
echo " Waiting 15s for SITL MAVLink stack to initialize..."
echo "========================================================"
sleep 15   

echo "========================================================"
echo " Launching MAVROS & Gazebo Bridge..."
echo "========================================================"

# Launch the Multi-MAVROS ROS 2 script in its own terminal

# Launch the Multi-MAVROS ROS 2 script in its own terminal
gnome-terminal --title="ROS 2 MAVROS Swarm" -- bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 launch mavros apm.launch fcu_url:=\"udp://127.0.0.1:14555@\" tgt_system:=1; exec bash" &

# --------------------------------------------------------
# Gazebo -> ROS 2 Bridges (Joy)
# --------------------------------------------------------
ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/Joey/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/iris_runway/model/Joey/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /world/iris_runway/model/Joey/link/lidar_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &


# Keep the script alive
wait
