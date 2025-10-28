# 🤖 Autonomous SLAM Robot ROS2

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8%2B-brightgreen.svg)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)
![Contributions](https://img.shields.io/badge/contributions-welcome-orange.svg)

> Autonomous SLAM robot implementation using ROS2 for real-time mapping and navigation

---

## 📋 Project Idea

This project implements an autonomous mobile robot capable of performing Simultaneous Localization and Mapping (SLAM) in unknown environments. The robot uses ROS2 (Robot Operating System 2) framework to integrate various sensors and actuators for real-time navigation, obstacle avoidance, and path planning.

**Key Features:**
- Real-time SLAM implementation using laser scanners/LiDAR
- Autonomous navigation with dynamic path planning
- Obstacle detection and avoidance
- Map generation and persistence
- Multi-robot coordination support

---

## 🔧 Hardware Components

### Required Hardware:
- **Mobile Robot Platform**: TurtleBot3 Burger/Waffle or custom differential drive robot
- **LiDAR/Laser Scanner**: RPLIDAR A1/A2/A3 or Hokuyo UST-10LX
- **IMU Sensor**: MPU6050 or BNO055 for orientation tracking
- **Wheel Encoders**: Optical encoders for odometry
- **Single Board Computer**: Raspberry Pi 4 (4GB+) or NVIDIA Jetson Nano/Xavier
- **Power Supply**: LiPo battery (11.1V, 2200mAh minimum)
- **Motor Driver**: L298N or Cytron MD10C for motor control

### Optional Hardware:
- RGB-D Camera (Intel RealSense D435i) for 3D mapping
- GPS Module for outdoor navigation
- Ultrasonic sensors for additional obstacle detection

### Hardware Setup Notes:
1. Mount LiDAR sensor at appropriate height (typically robot center, ~25cm from ground)
2. Ensure IMU is mounted rigidly to minimize vibration
3. Connect wheel encoders to GPIO pins or dedicated encoder interface
4. Configure motor driver connections according to your robot's specifications
5. Use proper power distribution to avoid voltage drops

---

## 🏗️ Software Architecture

### System Overview:
```
┌─────────────────────────────────────────────────┐
│              ROS2 Navigation Stack              │
├─────────────────────────────────────────────────┤
│  SLAM Toolbox  │  Nav2  │  AMCL  │  Planner   │
├─────────────────────────────────────────────────┤
│        Sensor Fusion & Odometry Layer          │
├─────────────────────────────────────────────────┤
│  LiDAR Driver  │  IMU Driver  │  Encoders     │
└─────────────────────────────────────────────────┘
```

### Core Components:
- **SLAM Toolbox**: Real-time SLAM implementation
- **Nav2**: Navigation framework for path planning and execution
- **AMCL**: Adaptive Monte Carlo Localization
- **TF2**: Coordinate frame transformations
- **Sensor Drivers**: LiDAR, IMU, and encoder interfaces

---

## 💻 Code Implementation

### Dependencies & Installation

#### System Requirements:
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Python 3.8+
- C++14 compatible compiler

#### Installation Steps:

```bash
# 1. Install ROS2 Humble
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop

# 2. Install dependencies
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rplidar-ros \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# 3. Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AnishVyapari/autonomous_slam_robot_ros2.git

# 4. Install Python dependencies
cd autonomous_slam_robot_ros2
pip install -r requirements.txt

# 5. Build the workspace
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# 6. Source the workspace
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step-by-Step Implementation

#### 1. Launch SLAM Mapping:
```bash
ros2 launch slam_robot slam_mapping.launch.py
```

#### 2. Visualize in RViz2:
```bash
ros2 run rviz2 rviz2 -d config/slam_robot.rviz
```

#### 3. Teleoperate Robot (Optional):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 4. Save Generated Map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

#### 5. Launch Autonomous Navigation:
```bash
ros2 launch slam_robot navigation.launch.py map:=~/maps/my_map.yaml
```

---

## 📁 File Structure

```
autonomous_slam_robot_ros2/
├── config/                    # Configuration files
│   ├── slam_params.yaml      # SLAM parameters
│   ├── nav2_params.yaml      # Navigation parameters
│   ├── robot.yaml            # Robot specifications
│   └── slam_robot.rviz       # RViz configuration
├── launch/                    # Launch files
│   ├── slam_mapping.launch.py
│   ├── navigation.launch.py
│   └── simulation.launch.py
├── src/                       # Source code
│   ├── drivers/              # Hardware drivers
│   │   ├── lidar_driver.py
│   │   ├── imu_driver.py
│   │   └── motor_controller.py
│   ├── navigation/           # Navigation modules
│   │   ├── path_planner.py
│   │   └── obstacle_avoidance.py
│   └── slam/                 # SLAM implementation
│       └── slam_node.py
├── urdf/                      # Robot description
│   ├── robot.urdf.xacro
│   └── sensors.urdf.xacro
├── worlds/                    # Gazebo worlds
│   └── office.world
├── maps/                      # Saved maps
├── tests/                     # Unit tests
├── docs/                      # Documentation
├── requirements.txt           # Python dependencies
├── package.xml               # ROS2 package manifest
├── setup.py                  # Python package setup
├── CMakeLists.txt            # Build configuration
├── LICENSE                   # MIT License
└── README.md                 # This file
```

---

## 🧪 Testing/Simulation Setup

### Gazebo Simulation:

1. **Launch Simulation Environment:**
```bash
ros2 launch slam_robot simulation.launch.py world:=office
```

2. **Run SLAM in Simulation:**
```bash
ros2 launch slam_robot slam_mapping.launch.py use_sim_time:=true
```

3. **Test Navigation:**
```bash
ros2 launch slam_robot navigation.launch.py use_sim_time:=true
```

### Unit Testing:
```bash
# Run all tests
colcon test --packages-select autonomous_slam_robot_ros2

# View test results
colcon test-result --all
```

### Performance Benchmarking:
```bash
# Monitor node performance
ros2 run ros2_performance monitor_node

# Check topic rates
ros2 topic hz /scan
ros2 topic hz /odom
```

---

## 📚 Documentation Tips

### Getting Started:
1. Read through the [ROS2 Documentation](https://docs.ros.org/en/humble/)
2. Familiarize yourself with [Nav2 Concepts](https://navigation.ros.org/)
3. Understand [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

### Troubleshooting:
- **LiDAR not detected**: Check USB permissions (`sudo chmod 666 /dev/ttyUSB0`)
- **Poor mapping quality**: Adjust SLAM parameters in `config/slam_params.yaml`
- **Navigation failures**: Verify costmap configuration and inflation radius
- **TF errors**: Ensure all coordinate frames are properly published

### Useful Resources:
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Navigation2 Configuration Guide](https://navigation.ros.org/configuration/index.html)
- [SLAM Best Practices](https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md)

### Contributing:
Contributions are welcome! Please follow these steps:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🤝 Acknowledgments

- ROS2 Development Team
- Nav2 Contributors
- SLAM Toolbox Maintainers
- Open Source Robotics Foundation

---

## 📞 Contact

**Project Maintainer**: Anish Vyapari  
**GitHub**: [@AnishVyapari](https://github.com/AnishVyapari)

⭐ Star this repository if you find it helpful!
