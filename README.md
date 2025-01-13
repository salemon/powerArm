# powerArm

A ROS2-based robotic arm control and simulation project utilizing MoveIt2 for motion planning and control.

## Environment Prerequisites

- **Operating System:** Ubuntu 22.04
- **ROS Distribution:** ROS2 Humble
- **Motion Planning:** MoveIt2 for ROS2 Humble
- **Simulation:** Gazebo

## Project Structure

The project consists of three primary workspaces:

1. **Main Workspace (powerarm_ws)**
   - Core robot functionality and configurations
   - Robot description and primary nodes

2. **Gazebo Simulation Workspace (powerarm_ws_gazebo)**
   - Simulation configurations
   - Testing environments
   - Visual tools and demonstrations

3. **Control Workspace (powerArm_control_ws)**
   - Hardware interface implementations
   - Real robot control nodes

## Installation

### 1. System Dependencies

First, install the required system dependencies:

```bash
# Update package lists
sudo apt update

# Install ROS2 and core dependencies
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-moveit

# Install additional required packages
sudo apt install ros-humble-moveit-common
sudo apt install ros-humble-graph-msgs
sudo apt install ros-humble-visualization-msgs ros-humble-rviz-visual-tools
```

### 2. Building Workspaces

Build each workspace in sequence:

```bash
# Build main workspace
cd ~/powerArm/powerarm_ws
colcon build --symlink-install
source install/setup.bash

# Build control workspace
cd ../powerArm_control_ws
colcon build --symlink-install
source install/setup.bash

# Build Gazebo simulation workspace
cd ../powerarm_ws_gazebo
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Setting Up the Environment

Before using the robot, ensure proper environment setup:

```bash
# Source ROS2 base installation
source /opt/ros/humble/setup.bash

# For simulation:
source ~/powerArm/powerarm_ws/install/setup.bash
source ~/powerArm/powerarm_ws_gazebo/install/setup.bash

# For real robot control:
source ~/powerArm/powerarm_ws/install/setup.bash
source ~/powerArm/powerArm_control_ws/install/setup.bash
```

## Troubleshooting

### Common Issues

#### Build Errors

If you encounter build errors:

```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Rebuild with clean cache
colcon build --symlink-install --cmake-clean-cache
```

#### Missing Dependencies

Install missing packages:

```bash
sudo apt update
sudo apt install ros-humble-moveit-common
sudo apt install ros-humble-graph-msgs
```

#### General Tips

- Ensure all workspaces are sourced in the correct order
- Verify package dependencies are properly installed
- Check for CMake configuration errors in the build logs
```