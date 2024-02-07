#include <vector>
#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char *argv[])
{
	// initial ros2 node
	rclcpp::init(argc, argv);
	auto const node = std::make_shared<rclcpp::Node>(
		"powerarm_state",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
	// auto const logger = rclcpp::get_logger("powerarm");
	const auto& LOGGER = node->get_logger();

	static const std::string PLANNING_GROUP = "powerarm";

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	std::thread([&executor]() { executor.spin(); }).detach();
	moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

	// load model
	robot_model_loader::RobotModelLoader robot_model_loader(node);
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

	// initial robot state
	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
	moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);

	robot_state->setToDefaultValues();
	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

	std::vector<double> joint_values;
	robot_state->copyJointGroupPositions(joint_model_group, joint_values);
	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	// Calculate jacobian
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	Eigen::MatrixXd jacobian;
	robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
							reference_point_position, jacobian);
	RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

	joint_values[0] = 10 / 180.0 * 3.1415926;
	joint_values[1] = 45 / 180.0 * 3.1415926;
	joint_values[2] = 0;
	joint_values[3] = 45 / 180.0 * 3.1415926;

	bool within_bounds = move_group.setJointValueTarget(joint_values);
	if (!within_bounds)
	{
		RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
	}

	// create plan list
	std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan_list;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (success){
		size_t trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
		std::string trajectory_size_str = std::to_string(trajectory_size);
		RCLCPP_INFO(LOGGER, "The total number of points on the trajactory %s", trajectory_size_str.c_str());
	}
	plan_list.push_back(my_plan);

	// Check each points on the trajectory
	// printf("%ld %ld %ld\n",  my_plan.start_state_.joint_state.name.size(),  my_plan.start_state_.joint_state.position.size(),  my_plan.start_state_.joint_state.velocity.size());

	// for(std::size_t i = 0; i < my_plan.start_state_.joint_state.name.size(); i++) {
	// 	printf("%s %.5f\n", my_plan.start_state_.joint_state.name[i].c_str(), my_plan.start_state_.joint_state.position[i]);
	// }
	// for(std::size_t i = 0; i <  my_plan.trajectory_.joint_trajectory.points.size(); i++) {
	// 	for(std::size_t j = 0; j <  my_plan.trajectory_.joint_trajectory.points[i].positions.size(); j++) {
	// 		printf("%.5f\t",  my_plan.trajectory_.joint_trajectory.points[i].positions[j]);
	// 		}
	// 		printf("\n");
	// 		printf("time_from_start %u %u\n", my_plan.trajectory_.joint_trajectory.points[i].time_from_start.sec, my_plan.trajectory_.joint_trajectory.points[i].time_from_start.nanosec);
	// }
	// RCLCPP_INFO_STREAM(LOGGER, "trajactory: \n" << my_plan.start_state_.joint_trajectory. << "\n");
	
	// Second pose
	moveit::core::RobotStatePtr start_state = move_group.getCurrentState();
	start_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
	move_group.setStartState(*start_state);
	start_state->copyJointGroupPositions(joint_model_group, joint_values);

	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		RCLCPP_INFO(LOGGER, "Start State Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	joint_values[0] = 20 / 180.0 * 3.1415926;
	joint_values[1] = 0 / 180.0 * 3.1415926;
	joint_values[2] = 0;
	joint_values[3] = 60 / 180.0 * 3.1415926;
	
	within_bounds = move_group.setJointValueTarget(joint_values);
	if (!within_bounds)
	{
		RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
	}

	success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (success){
		size_t trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
		std::string trajectory_size_str = std::to_string(trajectory_size);
		RCLCPP_INFO(LOGGER, "The total number of points on the trajactory %s", trajectory_size_str.c_str());
	}
	plan_list.push_back(my_plan);

	// Execute plans
	// for (const auto& plan : plan_list) {
	//  move_group.execute(plan);
	// }

	moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
	moveit_msgs::msg::RobotTrajectory trajectory;

	for (const auto& plan : plan_list) {
		trajectory.joint_trajectory.points.insert(
			trajectory.joint_trajectory.points.end(),
			plan.trajectory_.joint_trajectory.points.begin(),
			plan.trajectory_.joint_trajectory.points.end()
		);
	}
	
	robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);

	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

	trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
	time_param.computeTimeStamps(rt, 1.0, 1.0);

	moveit_msgs::msg::RobotTrajectory rt_msg;
	rt.getRobotTrajectoryMsg(rt_msg);
	combined_plan = plan_list[0];
	combined_plan.trajectory_.joint_trajectory.points = rt_msg.joint_trajectory.points;

	move_group.execute(combined_plan);


	// robot_state->setJointGroupPositions(joint_model_group, joint_values);
	const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("ee_link");

	RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
	RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

	// moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
	// current_state->copyJointGroupPositions(joint_model_group, joint_values);

	// robot_state need to be updated manually every time you want to get the current state
	robot_state->update();
	robot_state->setJointGroupPositions(joint_model_group, joint_values);
	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
							reference_point_position, jacobian);
	RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

	rclcpp::shutdown();
	return 0;
}
