# MedBot - Autonomous Medical Delivery Robot

ROS 2 Humble autonomous delivery robot simulation with Gazebo Harmonic, Nav2, and SLAM Toolbox.

## Quick Start

### Prerequisites
- ROS 2 Humble
- Gazebo Harmonic
- Python 3.10+
- colcon build system

### Build

```bash
cd ~/Robotics_Assignment/medbot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/install/medbot_gazebo/share/medbot_gazebo/models:$GZ_SIM_RESOURCE_PATH
ros2 launch medbot_bringup medbot_bringup.launch.py slam:=true nav:=true mission:=true
```

### Publish Transform (Terminal 2)

```bash
source install/setup.bash
python3 /home/henok/Robotics_Assignment/fix_map_transform.py
```

### Send Delivery Request (Terminal 3)

```bash
source install/setup.bash
python3 /home/henok/Robotics_Assignment/move_robot.py
```

## Package Structure

- `medbot_bringup` - System launch configuration
- `medbot_gazebo` - Simulation and robot model
- `medbot_description` - URDF/SDF definitions
- `medbot_navigation` - Nav2 configuration
- `medbot_localization` - EKF and transforms
- `medbot_mission` - Delivery manager and state machine

## Robot Specifications

- Type: Differential drive
- Sensors: Lidar (10Hz), Camera, IMU
- Control: DiffDrive plugin
- Localization: EKF Robot Localization
- Navigation: Nav2 (NavFn + DWB)

## Delivery Locations

- Hospital: (18.0, 8.0)
- Clinic: (-16.0, 8.0)
- Pharmacy: (-8.0, -8.0)
- Shop Area: (12.0, -8.0)
- Home Base: (0.0, 0.0)

## Topics

Delivery Request: `/delivery/request` (JSON)
Delivery State: `/delivery/state` (String)
Robot Odometry: `/odom` (Odometry)
Lidar Scan: `/scan` (LaserScan)

## License

Apache 2.0
