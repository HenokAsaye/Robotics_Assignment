#!/bin/bash
# Test script to run MedBot simulation and capture all logs

set -e

WORKSPACE="/home/henok/Robotics_Assignment/medbot_ws"
LOG_DIR="$WORKSPACE/test_logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/medbot_test_$TIMESTAMP.log"

# Create log directory
mkdir -p "$LOG_DIR"

echo "=========================================="
echo "MedBot Gazebo Harmonic Test"
echo "=========================================="
echo "Workspace: $WORKSPACE"
echo "Log file: $LOG_FILE"
echo "Time: $(date)"
echo ""

# Set up environment
cd "$WORKSPACE"
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$WORKSPACE/install/medbot_gazebo/share/medbot_gazebo/models:$GZ_SIM_RESOURCE_PATH

echo "Environment:"
echo "  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "  ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
echo ""

# Verify model file exists
echo "Checking model files..."
if [ -f "$WORKSPACE/install/medbot_gazebo/share/medbot_gazebo/models/medbot/model.sdf" ]; then
    echo "✓ Model SDF found"
else
    echo "✗ Model SDF NOT found!"
    exit 1
fi

if [ -f "$WORKSPACE/install/medbot_gazebo/share/medbot_gazebo/models/medbot/model.config" ]; then
    echo "✓ Model config found"
else
    echo "✗ Model config NOT found!"
fi

echo ""
echo "Starting ROS 2 launch (timeout 45 seconds)..."
echo "Command: ros2 launch medbot_bringup medbot_bringup.launch.py slam:=false nav:=false mission:=false rviz:=false"
echo ""

# Run with timeout and capture all output
timeout 45 ros2 launch medbot_bringup medbot_bringup.launch.py slam:=false nav:=false mission:=false rviz:=false 2>&1 | tee "$LOG_FILE" || true

echo ""
echo "=========================================="
echo "Test completed. Full log saved to:"
echo "  $LOG_FILE"
echo ""
echo "Key log entries (spawn-related):"
grep -E "spawn_medbot|gazebo_harmonic|ERROR|data:|create" "$LOG_FILE" | head -30 || true
echo ""
echo "Complete log available with: cat $LOG_FILE"
echo "=========================================="
