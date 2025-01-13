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

The powerArm system provides simulation and control capabilities for robotic arm manipulation. Below are the key procedures for operating the system.

### Environment Setup

Before operating the robot, configure your environment:

```bash
# Source ROS2 base installation
source /opt/ros/humble/setup.bash

# For simulation environment:
source ~/powerArm/powerarm_ws/install/setup.bash
source ~/powerArm/powerarm_ws_gazebo/install/setup.bash

# For real robot control:
source ~/powerArm/powerarm_ws/install/setup.bash
source ~/powerArm/powerArm_control_ws/install/setup.bash
```

### Simulation Mode

Launch the robot in simulation mode with visualization:

```bash
# Launch MoveIt with RViz visualization
ros2 launch powerarm_moveit_config demo.launch.py

# Launch Gazebo simulation (in a new terminal)
ros2 launch powerarm_moveit_config gazebo.launch.py
```

### Robot Control

Control the robot using ROS2 command-line interface:

```bash
# Check robot state
ros2 topic echo /joint_states

# Monitor robot status
ros2 topic echo /powerarm/controller_state

# Execute a predefined motion
ros2 action send_goal /move_group moveit_msgs/action/MoveGroup "{...}"
```

### Motion Planning

Plan and execute movements using MoveIt2:

```bash
# Launch MoveIt2 motion planning server
ros2 launch powerarm_moveit_config move_group.launch.py

# Execute planned trajectory
ros2 service call /execute_trajectory moveit_msgs/srv/ExecuteTrajectory
```

### System Verification

Verify system functionality with these commands:

```bash
# Check active controllers
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# Verify robot model loaded correctly
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity

# Test communication with robot
ros2 topic hz /joint_states
```

### Real Robot Operation

For operating the physical robot:

```bash
# Start robot control nodes
ros2 launch powerarm_control robot_control.launch.py

# Enable robot motors
ros2 service call /powerarm/enable std_srvs/srv/SetBool "data: true"

# Home the robot
ros2 service call /powerarm/home std_srvs/srv/Trigger
```

### Common Operations

Control standard robot operations:

```bash
# Move to home position
ros2 service call /powerarm/go_home std_srvs/srv/Trigger

# Emergency stop
ros2 service call /powerarm/estop std_srvs/srv/Trigger

# Reset robot state
ros2 service call /powerarm/reset std_srvs/srv/Trigger
```

### Safety Considerations

The powerArm system includes several safety features:

```bash
# Check safety status
ros2 topic echo /powerarm/safety_status

# Enable safety monitoring
ros2 service call /powerarm/enable_monitoring std_srvs/srv/SetBool "data: true"

# Reset safety locks
ros2 service call /powerarm/reset_safety std_srvs/srv/Trigger
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