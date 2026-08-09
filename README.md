# GPS-Denied Autonomous Navigation

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo_Harmonic-orange.svg)](https://gazebosim.org/)
[![ArduPilot](https://img.shields.io/badge/Flight_Stack-ArduPilot-green.svg)](https://ardupilot.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg)](https://ubuntu.com/)

This repository contains the simulation environment and models required for GPS-denied autonomous navigation. It facilitates the testing of a multi-agent drone fleet utilizing a wireless mesh communication topology in environments without satellite positioning, relying instead on computer vision and localized perception pipelines.

---

## 📂 Repository Structure & File Explanations

*   **`Marky/`, `DeeDee/`, `Joey/`**
    These directories contain the individual drone model files, allowing for multi-agent fleet simulation, swarm testing, and individual payload configurations.
*   **`rpi_cam_v2/`**
    Contains the model files for the Raspberry Pi Camera Module V2. This simulates the downward or forward-facing camera on the drones, providing essential visual data.
*   **`robofest.sdf`**
    The Gazebo world file defining the simulated environment, ground plane, lighting, and obstacles. 
*   **`run_sim.sh`**
    The core execution script that automates the startup sequence for Gazebo, the ArduPilot SITL instances, MAVROS, and the ROS ↔ Gazebo bridges.
*   **`robofest` (ROS 2 Package)**
    The package containing the launch files (like `multi_mavros.launch.py`) required to manage the swarm nodes in ROS 2.

---

## ⚙️ Setup Instructions

### 1. Place the Models and World File
Ensure your drone models, camera model, and the `robofest.sdf` world file are placed in directories where Gazebo can locate them.
*   **Models:** Copy `Marky`, `DeeDee`, `Joey`, and `rpi_cam_v2` to your Gazebo models directory (e.g., `~/gz_ws/src/ardupilot_gazebo/models/`).
*   **World File:** Place the `robofest.sdf` file into the directory from which you plan to run the script. Alternatively, you can copy it to your Gazebo worlds directory and update the `gz sim -r robofest.sdf` command inside `run_sim.sh` to reflect the absolute path to the file.

### 2. Setup the ROS 2 Package
You must build the `robofest` ROS 2 package so the `multi_mavros.launch.py` file can be executed by the startup script.
```bash
# Navigate to your ROS 2 workspace (e.g., ~/ws)
cd ~/ws

# Build the specific package
colcon build --packages-select robofest

# Source the workspace
source install/setup.bash
```

### 3. Configure `run_sim.sh` Paths
The provided `run_sim.sh` script contains hardcoded paths that must be updated to match your local machine's directory structure. Open `run_sim.sh` and carefully modify the following:
*   **ArduPilot Environment:** Change `/home/vaibhav/ardupilot/Tools/autotest/sim_vehicle.py` to the correct absolute path where your ArduPilot repository is cloned.
*   **ROS 2 Workspace:** Change `source ~/ws/install/setup.bash` if your workspace directory is named differently or located elsewhere.


---

## 🚀 Launching the Simulation

Once the models are placed, the package is built, and the script paths are updated, make the script executable and run it:

```bash
chmod +x run_sim.sh
./run_sim.sh
```

**Boot Sequence:**
1.  **Gazebo Harmonic** launches the `robofest.sdf` environment.
2.  **ArduPilot SITL** instances boot up for Marky, DeeDee, and Joey in separate terminal windows.
3.  **MAVROS** swarm nodes initialize via the `robofest` ROS 2 package.
4.  **ROS ↔ Gazebo Bridge** connects the simulated camera and lidar data to ROS 2 topics.
