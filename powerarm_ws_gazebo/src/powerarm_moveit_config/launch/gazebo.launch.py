# import os
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, RegisterEventHandler
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# from launch.event_handlers import OnProcessExit

# def generate_launch_description():
#     robot_name_in_model = 'powerarm'
#     package_name = 'powerarm_urdf'
#     urdf_name = "powerarm_urdf.urdf"

#     pkg_share = FindPackageShare(package=package_name).find(package_name) 
#     urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

#     # Start Gazebo server
#     start_gazebo_cmd =  ExecuteProcess(
#         cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
#         output='screen')

#     # Launch the robot
#     spawn_entity_cmd = Node(
#         package='gazebo_ros', 
#         executable='spawn_entity.py',
#         arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')
    
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         arguments=[urdf_model_path],
#         parameters=[{'use_sim_time': True}],
#         output='screen'
#     )

#     # 关节状态发布器
#     load_joint_state_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'joint_state_broadcaster'],
#         output='screen'
#     )

#     # 路径执行控制器
#     load_joint_trajectory_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'powerarm_controller'],
#         output='screen'
#     )

#     close_evt1 =  RegisterEventHandler( 
#             event_handler=OnProcessExit(
#                 target_action=spawn_entity_cmd,
#                 on_exit=[load_joint_state_controller],
#             )
#     )
#     close_evt2 = RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_joint_state_controller,
#                 on_exit=[load_joint_trajectory_controller],
#             )
#     )
    
#     ld = LaunchDescription()
#     ld.add_entity(close_evt1)
#     ld.add_entity(close_evt2)

#     ld.add_action(start_gazebo_cmd)
#     ld.add_action(node_robot_state_publisher)
#     ld.add_action(spawn_entity_cmd)


#     return ld
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    robot_name_in_model = 'powerarm'
    package_name = 'powerarm_urdf'
    urdf_name = "powerarm_urdf.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', urdf_model_path],
        output='screen')
    
    # Add Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_model_path},
                   os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')],
        output='screen'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'powerarm_controller'],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_entity_cmd)

    # Add event handlers
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_joint_state_controller],
        )
    ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller],
        )
    ))

    return ld