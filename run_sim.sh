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
  
  # Kill the SITL terminal window and its child processes
  SITL_PID=$(pgrep -f "ArduPilot SITL Core")
  if [ -n "$SITL_PID" ]; then
    pkill -P $SITL_PID # Kill child (sim_vehicle.py)
    kill $SITL_PID     # Kill the terminal window
  fi
  
  # Final sweep
  pkill -f "sim_vehicle.py" || true
  echo "Done!"
  exit 0
}

# Trap signals for clean exit
trap cleanup SIGINT SIGTERM

echo "========================================================"
echo " Running Pre-Launch Cleanup & Network Config"
echo "========================================================"
killall -9 gz sim_vehicle.py python3 mavros_node parameter_bridge 2>/dev/null || true

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

echo "-> Spawning ArduPilot Flight Controller window..."
# Launch in a new terminal, title used for pgrep in cleanup
echo "-> Spawning ArduPilot Flight Controller window..."
# Use the absolute path to sim_vehicle.py
gnome-terminal --title="ArduPilot SITL Core" -- bash -c \
  "source ~/venv-ardupilot/bin/activate && /home/vaibhav/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --out=udp:127.0.0.1:14555; exec bash" &
echo "========================================================"
echo " Waiting 15s for SITL MAVLink stack to initialize..."
echo "========================================================"
sleep 15   

echo "========================================================"
echo " Launching MAVROS & Gazebo Bridge..."
echo "========================================================"


# Launch background processes
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 use_sim_time:=true


ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image &

ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo &

ros2 run ros_gz_bridge parameter_bridge \
 /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
 /scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
 /sensors/marker@visualization_msgs/msg/Marker[gz.msgs.Marker \
 /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Keep the script alive
wait
