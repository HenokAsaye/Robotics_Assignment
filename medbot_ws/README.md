# 🏥 Ethiopian Medical Delivery Robot - MedBot

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

An autonomous medical delivery robot simulation designed for Ethiopian urban environments (Addis Ababa style). This project implements a complete ROS 2 navigation stack with SLAM, Nav2, and custom mission control for healthcare logistics.

<p align="center">
  <img src="docs/images/medbot_banner.png" alt="MedBot Banner" width="800"/>
</p>

---

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [Team Members](#-team-members)
- [Features](#-features)
- [Directory Structure](#-directory-structure)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Building the Workspace](#-building-the-workspace)
- [Running the Simulation](#-running-the-simulation)
- [Package Descriptions](#-package-descriptions)
- [Module Ownership Guide](#-module-ownership-guide)
- [ROS 2 Topics and Services](#-ros-2-topics-and-services)
- [Testing Delivery Missions](#-testing-delivery-missions)
- [Mapping New Environments](#-mapping-new-environments)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎯 Project Overview

The MedBot project simulates an autonomous differential-drive robot designed to deliver medical supplies in Ethiopian urban environments. The simulation addresses real-world challenges including:

- **Traffic Navigation**: Minibus taxis, vehicles, and congested streets
- **Pedestrian Avoidance**: Dynamic obstacle detection
- **Infrastructure Challenges**: Potholes, uneven sidewalks, street vendors
- **Healthcare Logistics**: Multi-stop delivery routes between hospitals, clinics, and pharmacies

---

## 👥 Team Members

| Name | Module | Responsibility |
|------|--------|----------------|
| **Abel** | Modeling | Robot URDF/Xacro, sensor integration, Gazebo models |
| **Abenezer** | Localization | SLAM Toolbox, AMCL, EKF sensor fusion |
| **Henok** | Navigation | Nav2 stack, costmaps, planners, controllers |
| **Netsanet** | Obstacle Avoidance | Obstacle detection, emergency stop, mission control |

---

## ✨ Features

### Robot Hardware (Simulated)
- ✅ Differential drive platform
- ✅ 360° 2D LiDAR sensor
- ✅ RGB Camera
- ✅ IMU sensor
- ✅ Medical delivery compartment
- ✅ Ethiopian-themed visual design

### Navigation & Localization
- ✅ SLAM Toolbox for online mapping
- ✅ Nav2 navigation stack
- ✅ AMCL localization
- ✅ DWB local planner
- ✅ A* global planner
- ✅ Behavior tree navigation

### Mission Control
- ✅ Delivery Manager with queue system
- ✅ Multi-waypoint route following
- ✅ Emergency stop functionality
- ✅ Real-time status monitoring
- ✅ Obstacle detection and classification

### Simulation Environment
- ✅ Ethiopian urban world (Addis Ababa style)
- ✅ Hospital, clinic, and pharmacy models
- ✅ Minibus taxi stops
- ✅ Pedestrians and street vendors
- ✅ Road network with intersections

---

## 📁 Directory Structure

```
medbot_ws/
├── src/
│   ├── medbot_description/          # 🤖 Robot Model (Abel)
│   │   ├── urdf/
│   │   │   ├── medbot.urdf.xacro    # Main robot description
│   │   │   ├── robot_core.xacro     # Chassis and wheels
│   │   │   ├── materials.xacro      # Colors and materials
│   │   │   ├── gazebo_control.xacro # Gazebo plugins
│   │   │   └── sensors/
│   │   │       ├── lidar.xacro      # LiDAR sensor
│   │   │       ├── camera.xacro     # Camera sensor
│   │   │       └── imu.xacro        # IMU sensor
│   │   ├── launch/
│   │   │   └── display.launch.py    # RViz visualization
│   │   ├── rviz/
│   │   │   └── display.rviz         # RViz configuration
│   │   ├── meshes/                  # 3D mesh files (if any)
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── medbot_gazebo/               # 🌍 Simulation World
│   │   ├── worlds/
│   │   │   ├── addis_ababa_urban.world  # Main urban environment
│   │   │   └── empty_test.world         # Empty world for testing
│   │   ├── models/                  # Custom Gazebo models
│   │   ├── launch/
│   │   │   └── gazebo_simulation.launch.py
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── medbot_bringup/              # 🚀 System Launch
│   │   ├── launch/
│   │   │   ├── medbot_bringup.launch.py   # Full system launch
│   │   │   ├── simulation_only.launch.py  # Gazebo only
│   │   │   └── teleop.launch.py           # Teleop control
│   │   ├── rviz/
│   │   │   └── navigation.rviz      # Navigation RViz config
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── medbot_localization/         # 📍 Localization (Abenezer)
│   │   ├── config/
│   │   │   ├── slam_toolbox_params.yaml  # SLAM configuration
│   │   │   ├── amcl_params.yaml          # AMCL configuration
│   │   │   └── ekf_params.yaml           # EKF configuration
│   │   ├── launch/
│   │   │   ├── slam.launch.py            # SLAM launch
│   │   │   ├── localization.launch.py    # AMCL launch
│   │   │   └── ekf.launch.py             # EKF launch
│   │   ├── maps/                    # Saved maps
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── medbot_navigation/           # 🧭 Navigation (Henok)
│   │   ├── config/
│   │   │   └── nav2_params.yaml     # Complete Nav2 configuration
│   │   ├── launch/
│   │   │   └── navigation.launch.py # Navigation stack launch
│   │   ├── behavior_trees/          # Custom behavior trees
│   │   ├── maps/                    # Navigation maps
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   └── medbot_mission/              # 🎯 Mission Control (Netsanet)
│       ├── medbot_mission/
│       │   ├── __init__.py
│       │   ├── delivery_manager.py      # Delivery mission manager
│       │   ├── waypoint_publisher.py    # Route management
│       │   ├── status_monitor.py        # System health monitor
│       │   ├── emergency_stop.py        # Emergency stop handler
│       │   └── obstacle_detector.py     # Obstacle detection
│       ├── launch/
│       │   └── mission.launch.py        # Mission nodes launch
│       ├── config/
│       │   └── mission_params.yaml      # Mission parameters
│       ├── resource/
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
├── README.md                        # This file
└── .gitignore
```

---

## 📦 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Classic 11.x
- **RAM**: 8GB minimum (16GB recommended)
- **GPU**: OpenGL 3.3+ compatible

### Required ROS 2 Packages

```bash
# Navigation and SLAM
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox

# Robot description and simulation
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro

# Gazebo integration
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control

# Localization
sudo apt install ros-humble-robot-localization

# Teleop
sudo apt install ros-humble-teleop-twist-keyboard

# Visualization
sudo apt install ros-humble-rviz2
```

---

## 🔧 Installation

### Step 1: Create Workspace Directory

```bash
# Create workspace (if not using the provided structure)
mkdir -p ~/medbot_ws/src
cd ~/medbot_ws/src
```

### Step 2: Clone or Copy the Repository

If you received this as a package:
```bash
# Copy the provided src folder contents to ~/medbot_ws/src
cp -r /path/to/provided/src/* ~/medbot_ws/src/
```

### Step 3: Install Dependencies

```bash
cd ~/medbot_ws

# Install ROS dependencies using rosdep
sudo rosdep init  # Only if not done before
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🔨 Building the Workspace

### Full Build

```bash
cd ~/medbot_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Build Specific Package

```bash
# Build only one package (useful during development)
colcon build --packages-select medbot_description
colcon build --packages-select medbot_navigation
colcon build --packages-select medbot_mission
```

### Clean Build

```bash
# Remove build artifacts and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 🚀 Running the Simulation

### Quick Start - Full System

```bash
# Terminal 1: Source and launch everything
source ~/medbot_ws/install/setup.bash
ros2 launch medbot_bringup medbot_bringup.launch.py
```

This launches:
- Gazebo with Ethiopian urban environment
- Robot with all sensors
- SLAM Toolbox
- Nav2 Navigation stack
- RViz visualization

### Step-by-Step Launch

#### Terminal 1: Gazebo Simulation Only

```bash
source ~/medbot_ws/install/setup.bash
ros2 launch medbot_bringup simulation_only.launch.py
```

#### Terminal 2: Teleop Control (for manual driving)

```bash
source ~/medbot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Terminal 3: SLAM (for mapping)

```bash
source ~/medbot_ws/install/setup.bash
ros2 launch medbot_localization slam.launch.py
```

#### Terminal 4: Navigation

```bash
source ~/medbot_ws/install/setup.bash
ros2 launch medbot_navigation navigation.launch.py
```

#### Terminal 5: Mission Control

```bash
source ~/medbot_ws/install/setup.bash
ros2 launch medbot_mission mission.launch.py
```

### Robot Model Visualization Only

```bash
# View the robot model in RViz without Gazebo
ros2 launch medbot_description display.launch.py
```

---

## 📦 Package Descriptions

### medbot_description
**Owner: Abel (Modeling)**

Contains the robot's URDF/Xacro description files:
- Differential drive base with two drive wheels and one caster
- LiDAR sensor (360°, 12m range)
- RGB camera
- IMU sensor
- Medical delivery compartment with Ethiopian medical cross

Key files to edit:
- `urdf/robot_core.xacro` - Robot dimensions and inertia
- `urdf/sensors/*.xacro` - Sensor configurations

### medbot_gazebo
**Owner: Team**

Gazebo simulation world files:
- `addis_ababa_urban.world` - Complete Ethiopian urban environment
- Features: Hospital, clinic, pharmacy, minibus stops, pedestrians

### medbot_localization
**Owner: Abenezer (Localization)**

Localization and mapping configuration:
- SLAM Toolbox for online SLAM
- AMCL for localization with pre-built maps
- EKF for sensor fusion

Key files to edit:
- `config/slam_toolbox_params.yaml` - SLAM parameters
- `config/amcl_params.yaml` - Localization parameters

### medbot_navigation
**Owner: Henok (Navigation)**

Nav2 navigation stack configuration:
- Costmap configuration (local and global)
- DWB local planner
- NavFn/A* global planner
- Recovery behaviors

Key files to edit:
- `config/nav2_params.yaml` - All navigation parameters

### medbot_mission
**Owner: Netsanet (Obstacle Avoidance)**

Mission control Python nodes:
- `delivery_manager.py` - Handles delivery missions
- `waypoint_publisher.py` - Route management
- `obstacle_detector.py` - Obstacle detection
- `emergency_stop.py` - Safety stop
- `status_monitor.py` - System health

---

## 👨‍💻 Module Ownership Guide

### For Abel (Modeling)
Your primary workspace:
```
src/medbot_description/
├── urdf/           # Edit robot model here
├── meshes/         # Add 3D models here
└── launch/         # Robot visualization
```

### For Abenezer (Localization)
Your primary workspace:
```
src/medbot_localization/
├── config/         # SLAM and AMCL parameters
├── launch/         # Localization launch files
└── maps/           # Save maps here
```

### For Henok (Navigation)
Your primary workspace:
```
src/medbot_navigation/
├── config/         # Nav2 parameters
├── launch/         # Navigation launch
└── behavior_trees/ # Custom BTs
```

### For Netsanet (Obstacle Avoidance)
Your primary workspace:
```
src/medbot_mission/
├── medbot_mission/ # Python nodes
├── config/         # Mission parameters
└── launch/         # Mission launch
```

---

## 📡 ROS 2 Topics and Services

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LiDAR data |
| `/camera/image_raw` | sensor_msgs/Image | Camera image |
| `/imu/data` | sensor_msgs/Imu | IMU data |
| `/odom` | nav_msgs/Odometry | Odometry |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/map` | nav_msgs/OccupancyGrid | SLAM map |
| `/goal_pose` | geometry_msgs/PoseStamped | Navigation goal |
| `/delivery/status` | std_msgs/String | Delivery status |
| `/delivery/request` | std_msgs/String | Delivery requests |

### Key Services

| Service | Type | Description |
|---------|------|-------------|
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | Navigate to goal |
| `/follow_waypoints` | nav2_msgs/action/FollowWaypoints | Follow route |

---

## 🎯 Testing Delivery Missions

### Send a Delivery Request

```bash
# Send a delivery request (JSON format)
ros2 topic pub /delivery/request std_msgs/String "data: '{
  \"id\": \"DEL_001\",
  \"pickup\": \"pharmacy\",
  \"delivery\": \"hospital\",
  \"priority\": 1,
  \"package_type\": \"medicine\",
  \"requester\": \"Tikur Anbessa Hospital\"
}'" --once
```

### Available Locations
- `pharmacy` - Pharmacy location
- `hospital` - Tikur Anbessa Hospital
- `clinic` - Health Clinic
- `home_base` - Robot home position
- `shop_area` - Shop area

### Execute a Predefined Route

```bash
# Execute pharmacy to hospital route
ros2 topic pub /route/request std_msgs/String \
  "data: 'pharmacy_to_hospital'" --once
```

### Monitor Delivery Status

```bash
# Watch delivery status
ros2 topic echo /delivery/status

# Watch system health
ros2 topic echo /robot/health
```

### Set Navigation Goal in RViz

1. Click "2D Goal Pose" button in RViz
2. Click and drag on the map to set goal position and orientation
3. Robot will navigate autonomously

---

## 🗺️ Mapping New Environments

### Step 1: Launch Simulation with SLAM

```bash
ros2 launch medbot_bringup medbot_bringup.launch.py slam:=true nav:=false
```

### Step 2: Drive Robot Around

```bash
# In another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Step 3: Save the Map

```bash
# Save map when mapping is complete
ros2 run nav2_map_server map_saver_cli -f ~/medbot_ws/src/medbot_localization/maps/my_map
```

### Step 4: Use the Map for Navigation

```bash
ros2 launch medbot_bringup medbot_bringup.launch.py \
  slam:=false \
  map:=~/medbot_ws/src/medbot_localization/maps/my_map.yaml
```

---

## 🔍 Troubleshooting

### Gazebo Crashes on Start

```bash
# Kill any zombie Gazebo processes
killall -9 gazebo gzserver gzclient

# Try with verbose output
ros2 launch medbot_gazebo gazebo_simulation.launch.py --debug
```

### Robot Not Moving

1. Check if `/cmd_vel` is being published:
```bash
ros2 topic echo /cmd_vel
```

2. Check if controllers are running:
```bash
ros2 topic list | grep joint
```

### SLAM Not Working

1. Check if `/scan` data is available:
```bash
ros2 topic echo /scan --once
```

2. Check TF tree:
```bash
ros2 run tf2_tools view_frames
```

### Navigation Fails

1. Check if costmaps are being published:
```bash
ros2 topic echo /global_costmap/costmap_raw --once
```

2. Check Nav2 lifecycle:
```bash
ros2 lifecycle list /controller_server
```

### Build Errors

```bash
# Clean and rebuild
cd ~/medbot_ws
rm -rf build/ install/ log/
colcon build --symlink-install 2>&1 | tee build.log
```

---

## 🤝 Contributing

### Git Workflow

1. Create a feature branch:
```bash
git checkout -b feature/your-feature-name
```

2. Make changes and test

3. Commit with clear messages:
```bash
git add .
git commit -m "[MODULE] Description of changes"
# Examples:
# [MODELING] Add camera mount to robot
# [NAV] Tune DWB controller parameters
# [LOCALIZATION] Improve SLAM loop closure
```

4. Push and create pull request

### Code Style
- Python: Follow PEP 8
- XML/YAML: Use 2-space indentation
- Add comments for complex logic

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

---

## 🙏 Acknowledgments

- ROS 2 Community
- Navigation2 Team
- SLAM Toolbox Developers
- Addis Ababa University Robotics Program

---

**Made with ❤️ for Ethiopian Healthcare by:**
- Abel (Modeling)
- Abenezer (Localization)  
- Henok (Navigation)
- Netsanet (Obstacle Avoidance)

---

*For questions or issues, please open an issue on the project repository.*
