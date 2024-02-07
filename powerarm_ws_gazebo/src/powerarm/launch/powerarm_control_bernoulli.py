from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("powerarm_urdf", package_name="powerarm_moveit_config").to_moveit_configs()

    powerarm_control = Node(
        name="powerarm_control_bernoulli",
        package="powerarm",
        executable="powerarm_control_bernoulli",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([powerarm_control])
