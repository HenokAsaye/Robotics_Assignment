# MedBot - Autonomous Medical Delivery Robot Simulation

## ROS-Based Autonomous Delivery Robot Simulation for Medical Supply Transport in Ethiopian Urban Areas

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

An autonomous medical delivery robot simulation designed for Ethiopian urban environments (Addis Ababa style). This project implements a complete ROS 2 navigation stack with SLAM, Nav2, and custom mission control for healthcare logistics.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Team Members](#team-members)
3. [System Requirements](#system-requirements)
4. [Installation Guide](#installation-guide)
5. [Workspace Structure](#workspace-structure)
6. [Building the Workspace](#building-the-workspace)
7. [Running the Simulation](#running-the-simulation)
8. [Package Descriptions](#package-descriptions)
9. [Robot Specifications](#robot-specifications)
10. [ROS2 Topics Reference](#ros2-topics-reference)
11. [Configuration Files](#configuration-files)
12. [Navigation System](#navigation-system)
13. [Mission Control System](#mission-control-system)
14. [Gazebo World Description](#gazebo-world-description)
15. [Usage Examples](#usage-examples)
16. [Saving and Using Maps](#saving-and-using-maps)
17. [Troubleshooting](#troubleshooting)
18. [Individual Contributions](#individual-contributions)

---

## Project Overview

### Problem Statement

In Ethiopia, especially in urban and semi-urban areas, the transportation of essential medical supplies such as medicines, vaccines, and laboratory samples faces significant challenges. Traffic congestion, limited transportation infrastructure, and delays in last-mile delivery often prevent timely access to healthcare services.

Delayed delivery of medical supplies can lead to shortages, treatment interruptions, and serious health risks for patients. Therefore, there is a need for an efficient, low-cost, and autonomous solution that can assist in transporting small medical supplies safely and reliably.

### Proposed Solution

This project implements a simulation of an autonomous mobile delivery robot using ROS 2 and Gazebo to transport medical supplies within a simulated Ethiopian urban environment (modeled after Addis Ababa). The robot autonomously navigates between hospitals, clinics, and pharmacies while avoiding obstacles.

### Key Features

- **Differential Drive Robot** with LiDAR, Camera, and IMU sensors
- **SLAM Mapping** using SLAM Toolbox for environment mapping
- **Nav2 Navigation Stack** for autonomous path planning and obstacle avoidance
- **EKF Sensor Fusion** combining odometry and IMU data
- **Mission Control System** for managing medical deliveries
- **Enhanced Obstacle Detection** with tracking and classification (pedestrians, bajaj, vehicles)
- **Ethiopian Urban Environment** simulation with realistic obstacles
- **Dynamic Pedestrians** walking through the environment

---

## Team Members

| Name | ID | Role | Responsibility |
|------|-----|------|----------------|
| Abel Guta | UGR/0165/15 | Navigation & Integration | Configure Nav2, path planning, system integration |
| Abenezer Elias | UGR/3180/15 | Obstacle Detection | Obstacle detection, avoidance, emergency stop |
| Henok Asaye | UGR/4382/15 | Localization & Mapping | SLAM, AMCL, EKF sensor fusion |
| Netsanet Tibebu | UGR/2340/15 | Robot Modeling | Robot URDF, sensors, Gazebo simulation |

---

## System Requirements

### Software Requirements

| Software | Version | Purpose |
|----------|---------|---------|
| Ubuntu | 22.04 LTS | Operating System |
| ROS 2 | Humble Hawksbill | Robot Operating System |
| Gazebo | Classic 11.x | Physics Simulation |
| Python | 3.10+ | Mission Control Nodes |
| colcon | Latest | Build System |

### Hardware Requirements (Recommended)

- **CPU**: Intel Core i5 or equivalent (4+ cores)
- **RAM**: 8 GB minimum, 16 GB recommended
- **GPU**: Dedicated graphics recommended for Gazebo
- **Storage**: 10 GB free space

### ROS 2 Packages Required

```bash
# Navigation Stack
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# SLAM
sudo apt install ros-humble-slam-toolbox

# Robot Localization (EKF)
sudo apt install ros-humble-robot-localization

# Gazebo Integration
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control

# Robot Description
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher

# Visualization
sudo apt install ros-humble-rviz2

# Teleop
sudo apt install ros-humble-teleop-twist-keyboard

# TF Tools
sudo apt install ros-humble-tf2-tools
```

---

## Installation Guide

### Step 1: Install ROS 2 Humble (if not installed)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install All Dependencies

```bash
# Install all required packages in one command
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-tools \
    python3-colcon-common-extensions

# Install Python dependencies
pip3 install numpy
```

### Step 3: Setup Workspace

```bash
# Navigate to workspace
cd ~/medbot_ws  # or your workspace path

# Install ROS dependencies using rosdep
sudo rosdep init  # Only if not done before
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## Workspace Structure

```
medbot_ws/
├── src/
│   ├── medbot_bringup/              # System launch orchestration
│   │   ├── launch/
│   │   │   ├── medbot_bringup.launch.py    # Full system launch
│   │   │   ├── simulation_only.launch.py   # Gazebo only
│   │   │   └── teleop.launch.py            # Teleoperation
│   │   └── rviz/
│   │       └── navigation.rviz             # RViz config
│   │
│   ├── medbot_description/          # Robot URDF/Xacro files
│   │   ├── urdf/
│   │   │   ├── medbot.urdf.xacro          # Main robot file
│   │   │   ├── robot_core.xacro           # Chassis & wheels
│   │   │   ├── gazebo_control.xacro       # Diff drive plugin
│   │   │   ├── materials.xacro            # Colors
│   │   │   └── sensors/
│   │   │       ├── lidar.xacro            # 2D LiDAR
│   │   │       ├── camera.xacro           # RGB Camera
│   │   │       └── imu.xacro              # IMU sensor
│   │   ├── launch/
│   │   │   └── display.launch.py          # Robot visualization
│   │   └── meshes/                         # 3D mesh files
│   │
│   ├── medbot_gazebo/               # Simulation environments
│   │   ├── worlds/
│   │   │   ├── addis_ababa_urban.world    # Main simulation world
│   │   │   └── empty_test.world           # Testing world
│   │   ├── models/                         # Custom Gazebo models
│   │   └── launch/
│   │       └── gazebo_simulation.launch.py
│   │
│   ├── medbot_localization/         # SLAM & Localization
│   │   ├── config/
│   │   │   ├── slam_toolbox_params.yaml   # SLAM config
│   │   │   ├── amcl_params.yaml           # AMCL config
│   │   │   └── ekf_params.yaml            # EKF config
│   │   ├── launch/
│   │   │   ├── slam.launch.py             # SLAM mapping
│   │   │   ├── localization.launch.py     # AMCL localization
│   │   │   └── ekf.launch.py              # Sensor fusion
│   │   └── maps/                           # Saved maps
│   │
│   ├── medbot_navigation/           # Nav2 Configuration
│   │   ├── config/
│   │   │   └── nav2_params.yaml           # Full Nav2 config
│   │   ├── launch/
│   │   │   └── navigation.launch.py
│   │   ├── behavior_trees/                 # Custom BTs
│   │   └── maps/                           # Navigation maps
│   │
│   └── medbot_mission/              # Mission Control (Python)
│       ├── medbot_mission/
│       │   ├── __init__.py
│       │   ├── delivery_manager.py        # Delivery state machine
│       │   ├── waypoint_publisher.py      # Route management
│       │   ├── obstacle_detector.py       # Enhanced obstacle detection
│       │   ├── emergency_stop.py          # Safety system
│       │   └── status_monitor.py          # Health monitoring
│       ├── config/
│       │   └── mission_params.yaml
│       └── launch/
│           └── mission.launch.py
│
├── build/                           # Build artifacts (generated)
├── install/                         # Installed packages (generated)
├── log/                             # Build logs (generated)
└── README.md                        # This file
```

---

## Building the Workspace

### Full Build

```bash
cd ~/medbot_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for persistence
echo "source ~/medbot_ws/install/setup.bash" >> ~/.bashrc
```

### Build Specific Package

```bash
# Build only one package
colcon build --packages-select medbot_description

# Build package with its dependencies
colcon build --packages-up-to medbot_navigation
```

### Clean Build

```bash
# Remove build artifacts and rebuild
cd ~/medbot_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Simulation

### Quick Start - Launch Everything

```bash
# Source the workspace
source ~/medbot_ws/install/setup.bash

# Launch full system
ros2 launch medbot_bringup medbot_bringup.launch.py
```

This single command launches:
- Gazebo with Ethiopian urban environment
- Robot with all sensors (LiDAR, Camera, IMU)
- SLAM Toolbox for mapping
- EKF sensor fusion
- Nav2 Navigation stack
- Mission Control nodes
- RViz visualization

### Launch with Custom Options

```bash
# Full system with all features (default)
ros2 launch medbot_bringup medbot_bringup.launch.py \
    slam:=true \
    nav:=true \
    mission:=true \
    ekf:=true \
    rviz:=true

# Simulation only (no autonomous navigation)
ros2 launch medbot_bringup medbot_bringup.launch.py \
    slam:=false \
    nav:=false \
    mission:=false

# Without RViz (headless)
ros2 launch medbot_bringup medbot_bringup.launch.py rviz:=false

# Use empty world for testing
ros2 launch medbot_bringup medbot_bringup.launch.py \
    world:=$(ros2 pkg prefix medbot_gazebo)/share/medbot_gazebo/worlds/empty_test.world
```

### Launch Arguments Reference

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo simulation clock |
| `slam` | `true` | Enable SLAM Toolbox mapping |
| `nav` | `true` | Enable Nav2 navigation stack |
| `mission` | `true` | Enable mission control nodes |
| `ekf` | `true` | Enable EKF sensor fusion |
| `rviz` | `true` | Launch RViz visualization |
| `world` | `addis_ababa_urban.world` | Gazebo world file path |

### Step-by-Step Launch (Advanced/Debug)

For debugging or development, launch components separately:

```bash
# Terminal 1: Gazebo Simulation
ros2 launch medbot_gazebo gazebo_simulation.launch.py

# Terminal 2: SLAM Toolbox
ros2 launch medbot_localization slam.launch.py

# Terminal 3: EKF Sensor Fusion
ros2 launch medbot_localization ekf.launch.py

# Terminal 4: Nav2 Navigation Stack
ros2 launch medbot_navigation navigation.launch.py

# Terminal 5: Mission Control
ros2 launch medbot_mission mission.launch.py

# Terminal 6: RViz Visualization
rviz2 -d ~/medbot_ws/src/medbot_bringup/rviz/navigation.rviz
```

### Teleoperation (Manual Control)

```bash
# Launch teleop keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Control Keys:
#        u    i    o
#        j    k    l
#        m    ,    .
#
# i : forward
# , : backward
# j : rotate left
# l : rotate right
# u : forward + left
# o : forward + right
# m : backward + left
# . : backward + right
# k : stop
# q/z : increase/decrease max speeds
```

### View Robot Model Only

```bash
# Visualize robot URDF in RViz without simulation
ros2 launch medbot_description display.launch.py
```

---

## Package Descriptions

### medbot_bringup
**Purpose**: Main entry point for launching the complete system.

| Launch File | Description |
|-------------|-------------|
| `medbot_bringup.launch.py` | Complete system (Gazebo + SLAM + Nav2 + EKF + Mission + RViz) |
| `simulation_only.launch.py` | Gazebo simulation with robot only |
| `teleop.launch.py` | Keyboard teleoperation |

### medbot_description
**Owner**: Netsanet (Robot Modeling)

**Purpose**: Robot model definition using URDF/Xacro.

| File | Description |
|------|-------------|
| `medbot.urdf.xacro` | Main assembly file |
| `robot_core.xacro` | Chassis, wheels, delivery compartment |
| `gazebo_control.xacro` | Differential drive plugin |
| `materials.xacro` | Ethiopian-themed colors |
| `sensors/lidar.xacro` | 2D LiDAR sensor |
| `sensors/camera.xacro` | RGB Camera with optical frame |
| `sensors/imu.xacro` | 6-axis IMU |

### medbot_gazebo
**Purpose**: Simulation world with Ethiopian urban environment.

| World File | Description |
|------------|-------------|
| `addis_ababa_urban.world` | Full urban environment with dynamic pedestrians |
| `empty_test.world` | Minimal world for basic testing |

### medbot_localization
**Owner**: Henok (Localization & Mapping)

**Purpose**: Robot localization and mapping.

| Config File | Purpose |
|-------------|---------|
| `slam_toolbox_params.yaml` | SLAM configuration |
| `amcl_params.yaml` | Particle filter localization |
| `ekf_params.yaml` | Sensor fusion (odom + IMU) |

### medbot_navigation
**Owner**: Abel (Navigation & Path Planning)

**Purpose**: Autonomous navigation using Nav2.

| Component | Configuration |
|-----------|---------------|
| Global Planner | NavFn with A* algorithm |
| Local Planner | DWB (Dynamic Window) |
| Costmaps | 5cm resolution, voxel layer |
| Recovery | Spin, backup, wait behaviors |

### medbot_mission
**Owner**: Abenezer (Obstacle Detection & Mission Control)

**Purpose**: High-level mission control for medical deliveries.

| Node | Function |
|------|----------|
| `delivery_manager.py` | Delivery state machine |
| `waypoint_publisher.py` | Route management |
| `obstacle_detector.py` | LiDAR-based obstacle tracking with categories |
| `emergency_stop.py` | Safety collision avoidance |
| `status_monitor.py` | System health monitoring |

---

## Robot Specifications

### Physical Dimensions

| Component | Dimension | Mass |
|-----------|-----------|------|
| Chassis | 0.50m x 0.35m x 0.15m | 15.0 kg |
| Drive Wheels (x2) | Radius=0.08m, Width=0.04m | 1.0 kg each |
| Wheel Separation | 0.40m | - |
| Caster Wheel | Radius=0.04m | 0.5 kg |
| Delivery Compartment | 0.35m x 0.30m x 0.25m | 2.0 kg |
| **Total Robot** | ~0.50m x 0.40m x 0.40m | **~19.5 kg** |

### Sensor Specifications

| Sensor | Parameter | Value |
|--------|-----------|-------|
| **LiDAR** | | |
| | Range | 0.15m - 12.0m |
| | Samples | 360 per scan |
| | Update Rate | 10 Hz |
| | Resolution | 0.01m |
| | FOV | 360 degrees |
| **RGB Camera** | | |
| | Resolution | 640 x 480 pixels |
| | FOV | 80 degrees horizontal |
| | Frame Rate | 30 Hz |
| | Format | RGB8 |
| **IMU** | | |
| | Update Rate | 100 Hz |
| | Axes | 6 (3 gyro + 3 accelerometer) |

### Motion Capabilities

| Parameter | Value |
|-----------|-------|
| Max Linear Velocity | 0.5 m/s |
| Max Angular Velocity | 1.0 rad/s |
| Max Linear Acceleration | 2.5 m/s² |
| Max Angular Acceleration | 3.2 rad/s² |
| Drive Type | Differential |

---

## ROS2 Topics Reference

### Sensor Topics

| Topic | Type | Hz | Description |
|-------|------|-----|-------------|
| `/scan` | sensor_msgs/LaserScan | 10 | LiDAR scan data |
| `/camera/image_raw` | sensor_msgs/Image | 30 | RGB camera image |
| `/camera/camera_info` | sensor_msgs/CameraInfo | 30 | Camera calibration |
| `/imu/data` | sensor_msgs/Imu | 100 | IMU measurements |

### Odometry Topics

| Topic | Type | Hz | Description |
|-------|------|-----|-------------|
| `/odom` | nav_msgs/Odometry | 50 | Wheel odometry |
| `/odom_filtered` | nav_msgs/Odometry | 30 | EKF-fused odometry |

### Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands to robot |
| `/joint_states` | sensor_msgs/JointState | Wheel joint positions |

### Navigation Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | nav_msgs/OccupancyGrid | SLAM/static map |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Local obstacle map |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Global planning map |
| `/plan` | nav_msgs/Path | Global path |
| `/local_plan` | nav_msgs/Path | Local trajectory |
| `/goal_pose` | geometry_msgs/PoseStamped | Navigation goal |

### Mission Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/delivery/state` | std_msgs/String | Current delivery state |
| `/delivery/status` | std_msgs/String | Detailed status (JSON) |
| `/delivery/request` | std_msgs/String | New delivery request (JSON) |
| `/delivery/cancel` | std_msgs/String | Cancel delivery |
| `/delivery/markers` | visualization_msgs/MarkerArray | Location visualization |

### Obstacle Detection Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/obstacles/detected` | std_msgs/String | Tracked obstacles with velocity (JSON) |
| `/obstacles/markers` | visualization_msgs/MarkerArray | Obstacle visualization |
| `/obstacles/warnings` | std_msgs/String | Close obstacle alerts |
| `/obstacles/danger` | std_msgs/String | Critical proximity alerts |

### Emergency Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/emergency_stop` | std_msgs/Bool | Emergency stop output |
| `/emergency_stop_trigger` | std_msgs/Bool | Manual emergency trigger |

### Transform Frames (TF Tree)

```
map
└── odom
    └── base_footprint
        └── base_link
            ├── lidar_link
            ├── camera_link
            │   └── camera_optical_link
            ├── imu_link
            ├── left_wheel_link
            ├── right_wheel_link
            ├── caster_wheel_link
            └── delivery_compartment_link
                └── medical_cross_link
```

---

## Configuration Files

### SLAM Configuration (`slam_toolbox_params.yaml`)

Key parameters:
```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    resolution: 0.05              # 5cm map resolution
    max_laser_range: 12.0         # Matches LiDAR spec
    do_loop_closing: true         # Enable loop closure
    loop_search_maximum_distance: 3.0
    mode: mapping                 # or 'localization'
```

### Navigation Configuration (`nav2_params.yaml`)

Key parameters:
```yaml
# Controller (Local Planner)
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5              # Max forward speed
      max_vel_theta: 1.0          # Max rotation speed
      BaseObstacle.scale: 0.1     # Obstacle avoidance weight

# Costmaps
local_costmap:
  ros__parameters:
    resolution: 0.05              # 5cm resolution
    width: 3                      # 3m x 3m rolling window
    height: 3
    inflation_radius: 0.65        # Safety buffer around obstacles

# Global Planner
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      use_astar: true             # A* algorithm
      tolerance: 0.5              # Goal tolerance
```

### EKF Configuration (`ekf_params.yaml`)

Key parameters:
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0               # Filter update rate
    two_d_mode: true              # 2D robot motion

    # Wheel odometry input
    odom0: odom
    odom0_config: [true, true, false,    # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   true, false, false,    # vx, vy, vz
                   false, false, true,    # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az

    # IMU input
    imu0: imu/data
    imu0_config: [false, false, false,   # x, y, z
                  true, true, true,       # roll, pitch, yaw
                  false, false, false,    # vx, vy, vz
                  true, true, true,       # vroll, vpitch, vyaw
                  true, true, true]       # ax, ay, az
```

---

## Navigation System

### Nav2 Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    BT Navigator                              │
│  (Behavior Tree - orchestrates navigation behaviors)        │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
┌───────────┐ ┌───────────┐ ┌───────────────┐
│  Planner  │ │Controller │ │   Behavior    │
│  Server   │ │  Server   │ │    Server     │
│ (Global)  │ │ (Local)   │ │  (Recovery)   │
└─────┬─────┘ └─────┬─────┘ └───────┬───────┘
      │             │               │
      ▼             ▼               ▼
┌───────────┐ ┌───────────┐ ┌───────────────┐
│   NavFn   │ │    DWB    │ │ Spin/Backup/  │
│   A*      │ │  Planner  │ │    Wait       │
└───────────┘ └───────────┘ └───────────────┘
```

### Setting Navigation Goals

**Method 1: RViz (Recommended)**
1. Click "2D Goal Pose" button in RViz toolbar
2. Click on the map to set position
3. Drag to set orientation
4. Release to send goal

**Method 2: Command Line**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Method 3: Action Client**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 10.0, y: 5.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

---

## Mission Control System

### Delivery State Machine

```
                    ┌─────────────────┐
                    │      IDLE       │◄──────────────────┐
                    └────────┬────────┘                   │
                             │ New Request                │
                             ▼                            │
                    ┌─────────────────┐                   │
                    │   EN_ROUTE_     │                   │
                    │    PICKUP       │                   │
                    └────────┬────────┘                   │
                             │ Arrived                    │
                             ▼                            │
                    ┌─────────────────┐                   │
                    │   AT_PICKUP     │                   │
                    │   (Loading)     │                   │
                    └────────┬────────┘                   │
                             │ 5 seconds                  │
                             ▼                            │
                    ┌─────────────────┐                   │
                    │   EN_ROUTE_     │                   │
                    │   DELIVERY      │                   │
                    └────────┬────────┘                   │
                             │ Arrived                    │
                             ▼                            │
                    ┌─────────────────┐                   │
                    │  AT_DELIVERY    │                   │
                    │  (Unloading)    │                   │
                    └────────┬────────┘                   │
                             │ 5 seconds                  │
                             ▼                            │
                    ┌─────────────────┐                   │
                    │   COMPLETED     │───────────────────┘
                    └─────────────────┘
```

### Predefined Locations

| Location Name | Coordinates (x, y) | Orientation | Description |
|---------------|-------------------|-------------|-------------|
| `hospital` | (18.0, 8.0) | 90° (North) | Tikur Anbessa Hospital |
| `clinic` | (-16.0, 8.0) | 90° (North) | Health Clinic |
| `pharmacy` | (-8.0, -8.0) | -90° (South) | Pharmacy |
| `home_base` | (0.0, 0.0) | 0° (East) | Robot starting position |
| `shop_area` | (12.0, -8.0) | -90° (South) | Commercial area |

### Sending Delivery Requests

```bash
# Send a delivery request (JSON format)
ros2 topic pub --once /delivery/request std_msgs/String "{data: '{
  \"id\": \"DEL_001\",
  \"pickup\": \"pharmacy\",
  \"delivery\": \"hospital\",
  \"priority\": 1,
  \"package_type\": \"medicine\",
  \"requester\": \"Tikur Anbessa Hospital\",
  \"notes\": \"Urgent insulin delivery\"
}'}"
```

### Delivery Request Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | No | Unique delivery ID (auto-generated if missing) |
| `pickup` | string | Yes | Pickup location name |
| `delivery` | string | Yes | Delivery location name |
| `priority` | int | No | 1 (highest) to 5 (lowest), default: 3 |
| `package_type` | string | No | `medicine`, `blood`, `vaccine`, `equipment` |
| `requester` | string | No | Requesting facility name |
| `notes` | string | No | Additional notes |

### Monitor Delivery Status

```bash
# Watch delivery state changes
ros2 topic echo /delivery/state

# Watch detailed status (JSON)
ros2 topic echo /delivery/status
```

### Obstacle Categories

The enhanced obstacle detector classifies obstacles into Ethiopian urban categories:

| Category | Size Range | Examples | Marker Color |
|----------|------------|----------|--------------|
| `small_object` | < 0.3m | Debris, stones, cones | Grey |
| `pedestrian` | 0.3m - 0.8m | Walking people | Blue |
| `bajaj` | 0.8m - 1.5m | Three-wheeler taxis | Green |
| `vehicle` | 1.5m - 3.0m | Cars, minibuses | Yellow |
| `structure` | > 3.0m | Buildings, walls | Red |

---

## Gazebo World Description

### Environment: Addis Ababa Urban

**Dimensions**: 60m x 40m navigable area

### Healthcare Facilities

| Model | Location | Size | Description |
|-------|----------|------|-------------|
| Tikur Anbessa Hospital | (20, 12) | 15m x 10m x 8m | Main hospital with red cross |
| Health Clinic | (-18, 12) | 8m x 6m x 4m | Clinic with green cross |
| Pharmacy | (-8, -12) | 6m x 5m x 3.5m | Green pharmacy building |

### Dynamic Actors

| Actor | Path | Behavior |
|-------|------|----------|
| `walking_pedestrian_1` | North sidewalk | East-West patrol (32s cycle) |
| `walking_pedestrian_2` | South sidewalk | West-East patrol (42s cycle) |
| `walking_pedestrian_3` | Cross street | North-South crossing (26s cycle) |

### Static Obstacles

| Type | Count | Description |
|------|-------|-------------|
| Bajaj (3-wheeler) | 2 | Green and blue variants |
| Minibus Taxi | 1 | Blue/white, parked at stop |
| Vendor Carts | 2 | With colored umbrellas |
| Traffic Cones | 3 | Orange construction markers |
| Potholes | 2 | Road surface defects |
| Debris Pile | 1 | Construction materials |
| Trees | 3 | Ethiopian vegetation |
| Traffic Light | 1 | At intersection |
| Garbage Bin | 1 | Street furniture |
| Pedestrians (static) | 2 | Waiting/standing |

---

## Usage Examples

### Example 1: Basic Navigation to Hospital

```bash
# Launch full system
ros2 launch medbot_bringup medbot_bringup.launch.py

# Wait for system to initialize (~30 seconds)

# Send robot to hospital
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 18.0, y: 8.0, z: 0.0},
    orientation: {z: 0.707, w: 0.707}
  }
}"
```

### Example 2: Complete Delivery Mission

```bash
# Launch system
ros2 launch medbot_bringup medbot_bringup.launch.py

# In another terminal, monitor status
ros2 topic echo /delivery/status

# Send delivery request (pharmacy to hospital)
ros2 topic pub --once /delivery/request std_msgs/String "{data: '{
  \"pickup\": \"pharmacy\",
  \"delivery\": \"hospital\",
  \"package_type\": \"vaccine\",
  \"priority\": 1
}'}"

# Robot will:
# 1. Navigate to pharmacy
# 2. Wait 5 seconds (loading)
# 3. Navigate to hospital
# 4. Wait 5 seconds (unloading)
# 5. Return to home base
```

### Example 3: Monitor Obstacles

```bash
# Launch system
ros2 launch medbot_bringup medbot_bringup.launch.py

# Monitor detected obstacles
ros2 topic echo /obstacles/detected

# Monitor proximity warnings
ros2 topic echo /obstacles/warnings

# Monitor danger alerts (critical proximity)
ros2 topic echo /obstacles/danger
```

### Example 4: Emergency Stop

```bash
# Trigger emergency stop
ros2 topic pub --once /emergency_stop_trigger std_msgs/Bool "{data: true}"

# Robot will stop immediately

# Release emergency stop
ros2 topic pub --once /emergency_stop_trigger std_msgs/Bool "{data: false}"

# Robot resumes normal operation
```

### Example 5: Manual Teleoperation

```bash
# Launch simulation only
ros2 launch medbot_bringup medbot_bringup.launch.py nav:=false mission:=false

# In another terminal, start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Use keyboard to drive robot manually
```

---

## Saving and Using Maps

### Step 1: Create a Map with SLAM

```bash
# Launch system with SLAM enabled
ros2 launch medbot_bringup medbot_bringup.launch.py slam:=true

# Drive robot around using teleop or navigation goals
# Ensure complete coverage of the environment

# In RViz, you can see the map building in real-time
```

### Step 2: Save the Map

```bash
# When mapping is complete, save the map
ros2 run nav2_map_server map_saver_cli -f ~/medbot_ws/src/medbot_localization/maps/addis_ababa_map

# This creates two files:
# - addis_ababa_map.pgm (image)
# - addis_ababa_map.yaml (metadata)
```

### Step 3: Use Saved Map for Navigation

```bash
# Launch with localization mode (using saved map)
ros2 launch medbot_bringup medbot_bringup.launch.py \
    slam:=false \
    map:=~/medbot_ws/src/medbot_localization/maps/addis_ababa_map.yaml
```

---

## Troubleshooting

### Issue 1: Gazebo Crashes on Start

```bash
# Kill existing Gazebo processes
killall -9 gazebo gzserver gzclient

# Clear Gazebo cache
rm -rf ~/.gazebo/models/*

# Restart with verbose output
ros2 launch medbot_bringup medbot_bringup.launch.py --debug
```

### Issue 2: Robot Not Moving

```bash
# Check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Check if Gazebo is receiving commands
ros2 topic info /cmd_vel

# Verify differential drive plugin is loaded
ros2 topic list | grep joint

# Check for TF errors
ros2 run tf2_tools view_frames
```

### Issue 3: SLAM Not Building Map

```bash
# Check if LiDAR data is available
ros2 topic echo /scan --once

# Check SLAM node status
ros2 node list | grep slam

# Verify TF tree is complete
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

### Issue 4: Navigation Goal Rejected

```bash
# Check if robot is localized
ros2 topic echo /amcl_pose

# Check costmap for obstacles
ros2 topic echo /local_costmap/costmap --once

# Check Nav2 lifecycle state
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server

# If nodes are inactive, activate them
ros2 lifecycle set /controller_server activate
```

### Issue 5: Build Errors

```bash
# Clean and rebuild
cd ~/medbot_ws
rm -rf build/ install/ log/
colcon build --symlink-install 2>&1 | tee build.log

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Issue 6: TF Transform Errors

```bash
# View complete TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_footprint

# Monitor TF errors
ros2 run tf2_ros tf2_monitor
```

### Useful Debug Commands

```bash
# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Check topic frequency
ros2 topic hz /scan

# View node info
ros2 node info /slam_toolbox

# Check parameter values
ros2 param list /controller_server
ros2 param get /controller_server FollowPath.max_vel_x
```

---

## Individual Contributions

### Netsanet Tibebu - Robot Modeling & Simulation
- Designed robot URDF/Xacro model with proper kinematics and inertia
- Configured LiDAR, Camera (with optical frame), and IMU sensor plugins
- Created Ethiopian urban Gazebo world with realistic obstacles
- Implemented differential drive control plugin
- Added dynamic pedestrian actors

### Henok Asaye - Localization & Mapping
- Configured SLAM Toolbox for online environment mapping
- Set up AMCL particle filter for localization in known maps
- Implemented EKF sensor fusion combining odometry and IMU data
- Tuned localization parameters for Ethiopian urban terrain

### Abel Guta - Navigation & Path Planning
- Configured complete Nav2 navigation stack
- Tuned DWB local planner for safe urban navigation
- Set up costmap layers with appropriate inflation for pedestrians
- Integrated all system components for full autonomous operation
- System testing and performance evaluation

### Abenezer Elias - Obstacle Detection & Avoidance
- Implemented enhanced obstacle detector with spatial indexing
- Added Ethiopian obstacle categories (bajaj, pedestrian, vehicle)
- Created proximity warning and danger zone alert system
- Implemented emergency stop safety mechanism
- Developed delivery mission control state machine

---

## License

This project is developed for educational purposes as part of the Robotics course at Addis Ababa University.

---

## Acknowledgments

- ROS 2 Community
- Navigation2 Team
- SLAM Toolbox Developers
- Gazebo Simulation Team
- Addis Ababa University Robotics Program

---

**Project**: ROS-Based Autonomous Delivery Robot Simulation

**Course**: Robotics

**Institution**: Addis Ababa University

**Date**: January 2026

**ROS 2 Distribution**: Humble Hawksbill

**Gazebo Version**: Classic 11.x
