#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "powerarm_state",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("powerarm");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "powerarm");

  // Set a target Pose

  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -4.456e-05;
    msg.orientation.y = -2.6453e-06; 
    msg.orientation.z = -0.70255;
    msg.orientation.w = 0.71164;
    msg.position.x = 1.3066;
    msg.position.y = -0.11213;
    msg.position.z = 1.0248;
    return msg;
  }();

  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute 
  if(success) {
    move_group_interface.execute(plan);
    // move_group_interface.move();
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
/*
  left:
  msg.orientation.x = -4.456e-05;
  msg.orientation.y = -2.6453e-06; 
  msg.orientation.z = -0.70255;
  msg.orientation.w = 0.71164;
  msg.position.x = 1.3066;
  msg.position.y = -0.11213;
  msg.position.z = 1.0248;

  right:
  msg.orientation.x = -0.0018889;
  msg.orientation.y = -0.0021159; 
  msg.orientation.z = -0.61066;
  msg.orientation.w = 0.79189;
  msg.position.x = 1.3043;
  msg.position.y = 0.16549;
  msg.position.z = 1.0248;
*/