# powerArm

ROS and motion planning

## Environment

ROS Distro: ROS2 humble  
OS Version: Ubuntu 22.04  
Moveit version: Moveit2 for ROS humble

## Project Structure

```
POWERARM/
├── powerArm_control_ws/     # Control workspace
│   └── .vscode/            # VS Code configuration
│
├── powerarm_ws/            # Main workspace
│   ├── .vscode/           # VS Code configuration
│   ├── build/             # Build directory
│   ├── install/           # Install directory
│   ├── log/              # Log files
│   ├── src/              # Source code
│   │   ├── bernoulli_points.txt
│   │   ├── cmd.txt
│   │   └── points.txt
│   └── README.md
│
└── powerarm_ws_gazebo/    # Gazebo simulation workspace
    ├── .vscode/          # VS Code configuration
    ├── build/           # Build directory
    ├── install/        # Install directory
    ├── src/           # Source code
    │   ├── bernoulli_points.txt
    │   ├── cmd.txt
    │   ├── my_send_goal.sh
    │   ├── note.txt
    │   └── points.txt
    └── .gitignore
```

## Installation

1. Create the workspaces:
```bash
mkdir -p powerArm_control_ws/src
mkdir -p powerarm_ws/src
mkdir -p powerarm_ws_gazebo/src
```

2. Clone this repository:
```bash
git clone <repository-url> .
```

3. Install ROS2 dependencies:
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-moveit
```

4. Build the workspaces:
```bash
# Build main workspace
cd powerarm_ws
colcon build

# Build control workspace
cd ../powerArm_control_ws
colcon build

# Build Gazebo workspace
cd ../powerarm_ws_gazebo
colcon build
```

## Usage

1. Source ROS2 and workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/powerarm_ws/install/setup.bash
```

2. For Gazebo simulation:
```bash
source ~/powerarm_ws_gazebo/install/setup.bash
# Launch commands will be added here
```

3. For real robot control:
```bash
source ~/powerArm_control_ws/install/setup.bash
# Control commands will be added here
```



