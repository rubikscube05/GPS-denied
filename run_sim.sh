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
  
  # Kill gnome-terminals spawned for SITL and MAVROS
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

echo "-> Spawning ArduPilot Flight Controller windows (Swarm)..."

# Drone 1: Joy (Instance 0)
gnome-terminal --title="ArduPilot SITL - Joy" -- bash -c \
  "source ~/venv-ardupilot/bin/activate && /home/vaibhav/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I 0 --sysid 1 --model JSON --out=udp:127.0.0.1:14555 -N; exec bash" &
sleep 3

# Drone 2: Marky (Instance 1)
gnome-terminal --title="ArduPilot SITL - Marky" -- bash -c \
  "source ~/venv-ardupilot/bin/activate && /home/vaibhav/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I 1 --sysid 2 --model JSON --out=udp:127.0.0.1:14565 -N; exec bash" &
sleep 3

# Drone 3: DeeDee (Instance 2)
gnome-terminal --title="ArduPilot SITL - DeeDee" -- bash -c \
  "source ~/venv-ardupilot/bin/activate && /home/vaibhav/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I 2 --sysid 3 --model JSON --out=udp:127.0.0.1:14575 -N; exec bash" &

echo "========================================================"
echo " Waiting 15s for SITL MAVLink stack to initialize..."
echo "========================================================"
sleep 15   

echo "========================================================"
echo " Launching MAVROS & Gazebo Bridge..."
echo "========================================================"

# Launch the Multi-MAVROS ROS 2 script in its own terminal
gnome-terminal --title="ROS 2 MAVROS Swarm" -- bash -c \
  "source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash && ros2 launch robofest multi_mavros.launch.py; exec bash" &

# --------------------------------------------------------
# Gazebo -> ROS 2 Bridges (Joy)
# --------------------------------------------------------
ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/Joy/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/iris_runway/model/Joy/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /world/iris_runway/model/Joy/link/lidar_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &

# --------------------------------------------------------
# Gazebo -> ROS 2 Bridges (Marky)
# --------------------------------------------------------
ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/Marky/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/iris_runway/model/Marky/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /world/iris_runway/model/Marky/link/lidar_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &

# --------------------------------------------------------
# Gazebo -> ROS 2 Bridges (DeeDee)
# --------------------------------------------------------
ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/DeeDee/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/iris_runway/model/DeeDee/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /world/iris_runway/model/DeeDee/link/lidar_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &

# Keep the script alive
wait
