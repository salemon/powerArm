ros2 action send_goal /powerarm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_joint_1, shoulder_joint_2, shoulder_joint_3, elbow_joint],
    points: [
      { positions: [0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 0, nanosec: 500 } },
      { positions: [0.2, 0.5, 0.2 ,0.2], time_from_start: { sec: 5, nanosec: 500 } },
      { positions: [0.3, 0.3, 0.7, 0.3], time_from_start: { sec: 7, nanosec: 500 } },
      { positions: [0.4, 0.4, 0.9, 0.4], time_from_start: { sec: 8, nanosec: 500 } }
    ]
  }
}"
