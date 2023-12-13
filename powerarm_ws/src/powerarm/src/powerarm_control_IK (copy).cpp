#include <vector>
#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

// #include <yaml-cpp/yaml.h>
// #include <fstream>

// #include <moveit_visual_tools/moveit_visual_tools.h>

constexpr double pi = 3.14159265358979323846;

int main(int argc, char *argv[])
{
	// initial ros2 node
	rclcpp::init(argc, argv);
	auto const node = std::make_shared<rclcpp::Node>(
		"powerarm_control_IK",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
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
	
	// planning scene
	// planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
	// planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
	// planning_scene->getCurrentStateNonConst().setToDefaultValues();

	robot_state->setToDefaultValues();
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// start_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);

	std::vector<double> joint_values;
	move_group.setStartState(*current_state);
	current_state->copyJointGroupPositions(joint_model_group, joint_values);
	
	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

	for (std::size_t i = 0; i < joint_names.size(); i++)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	// initial jacobian
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	Eigen::MatrixXd jacobian;
	current_state->getJacobian(joint_model_group, current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
							reference_point_position, jacobian);
	RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

	Eigen::MatrixXd jacobian_reduced;
	Eigen::MatrixXd jacobian_inv;
	std::vector<int> idx = {0, 1, 2};
	jacobian_reduced = jacobian.block(idx[0], 0, idx.size(), jacobian.cols());
	jacobian_inv = jacobian_reduced.completeOrthogonalDecomposition().pseudoInverse();
	Eigen::MatrixXd prod;
	prod = jacobian_reduced * jacobian_inv;
	RCLCPP_INFO_STREAM(LOGGER, "Reduced matrix of Jacobian: \n" << jacobian_reduced << "\n");
	RCLCPP_INFO_STREAM(LOGGER, "Inverse matrix of reduced jacobian: \n" << jacobian_inv << "\n");
	RCLCPP_INFO_STREAM(LOGGER, "Product of jacobian: \n" << prod << "\n"); 
	// moveit::core::RobotState goal_state(kinematic_model);

	move_group.setPlanningTime(10.0);

	// Set a target Pose
	std::vector<geometry_msgs::msg::Pose> target_poses;
	geometry_msgs::msg::Pose sample_pose;

	// left:
	// sample_pose.orientation.x = -4.456e-05;
	// sample_pose.orientation.y = -2.6453e-06;
	// sample_pose.orientation.z = -0.70255;
	// sample_pose.orientation.w = 0.71164;
	sample_pose.position.x = 1.3066;
	sample_pose.position.y = -0.11213;
	sample_pose.position.z = 1.0248;
	target_poses.push_back(sample_pose);

	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (success){
		move_group.setPositionTarget(sample_pose.position.x , sample_pose.position.y, sample_pose.position.z, "ee_link");
		size_t trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
		std::string trajectory_size_str = std::to_string(trajectory_size);
		RCLCPP_INFO(LOGGER, "The total number of points on the trajactory %s", trajectory_size_str.c_str());

		current_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
		move_group.setStartState(*current_state);
		move_group.execute(my_plan);
	}

	// right:
	// sample_pose.orientation.x = -0.0018889;
	// sample_pose.orientation.y = -0.0021159;
	// sample_pose.orientation.z = -0.61066;
	// sample_pose.orientation.w = 0.79189;
	// sample_pose.position.x = 1.3043;
	// sample_pose.position.y = 0.16549;
	// sample_pose.position.z = 1.0248;

	// sample_pose.orientation.x = -0.10161; 
	// sample_pose.orientation.y = 0.14272; 
	// sample_pose.orientation.z = -0.28253; 
	// sample_pose.orientation.w = 0.94312;
	// sample_pose.position.x = 1.1077; 
	// sample_pose.position.y = 0.39747; 
	// sample_pose.position.z = 0.83359;
	// target_poses.push_back(sample_pose);

	// move_group.setPoseTarget(sample_pose, "ee_link");

	// circular path segment
	std::vector<std::vector<double>> target_pos;
	const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("ee_link");
	Eigen::Vector3d cur_pos = end_effector_state.translation();
	Eigen::Vector3d pos_dot;
	Eigen::Vector4d joint_dot;

	moveit_msgs::msg::RobotTrajectory trajectory;

	current_state = move_group.getCurrentState();
	current_state->copyJointGroupPositions(joint_model_group, joint_values);
	trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
	trajectory_point.positions = joint_values;
	trajectory.joint_trajectory.points.push_back(trajectory_point);

	for (std::size_t i = 0; i < joint_names.size(); i++)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), trajectory.joint_trajectory.points.back().positions[i]);
	}

	RCLCPP_INFO(LOGGER, "The type of trajectory_point.positions is %s",  typeid(trajectory_point.positions).name());

	float r = 0.13;
	// float dt = 0.1;
	std::vector<std::vector<double>> joint_limits {{-pi/2, pi/4}, {-pi/6, pi/2}, {0.0, 0.0}, {0.0, pi/2}};

	for(int theta = 0; theta <= 360; theta ++){
		// y = y_s + r(1 - std::cos(theta/180.0 * 3.1415926);
		// z = x_s + r * std::sin(theta/180.0 * 3.1415926);

		pos_dot.x() = 0;
		pos_dot.y() = r * std::sin(theta/180.0 * 3.1415926) * 1.0 / 180.0 * 3.1415926;
		pos_dot.z() = r * std::cos(theta/180.0 * 3.1415926) * 1.0 / 180.0 * 3.1415926;

		// if (theta % 90 == 0){
		// 	RCLCPP_INFO_STREAM(LOGGER, "pos derivation: \n" << pos_dot << "\n");
		// 	RCLCPP_INFO_STREAM(LOGGER, "pos: \n" << cur_pos << "\n");
		// }

		cur_pos += pos_dot;

		// current_state->getJacobian(joint_model_group, current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
		// 				reference_point_position, jacobian);

		// jacobian_reduced = jacobian.block(idx[0], 0, idx.size(), jacobian.cols());
		// jacobian_inv = jacobian_reduced.completeOrthogonalDecomposition().pseudoInverse();

		// joint_dot = jacobian_inv * pos_dot;

		// joints limitation
		for (std::size_t i = 0; i < joint_names.size(); i++)
		{
			joint_values[i] += joint_dot[i];

			// if (theta % 45 == 0){
			// 	RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			// }

			while (joint_values[i] >= pi){
				joint_values[i] -= 2.0 * pi;
			}

			while (joint_values[i] <= -pi){
				joint_values[i] += 2.0 * pi;
			}

			// if (theta % 45 == 0){
			// 	RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			// }

			joint_values[i] = std::min(std::max(joint_values[i], joint_limits[i][0]), joint_limits[i][1]);
			
			// if (temp != joint_values[i]){
			// 	std::cout << theta << ',' << i << std::endl;
			// }
		}

		// for (std::size_t i = 0; i < joint_names.size(); ++i)
		// {
		// 	joint_values[i] += joint_dot[i];
		// }

		current_state->setJointGroupPositions(joint_model_group, joint_values);
		current_state->copyJointGroupPositions(joint_model_group, joint_values);
		
		// if (theta % 45 == 0){
		// 	for (std::size_t i = 0; i < joint_names.size(); ++i)
		// 	{
		// 		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		// 	}

		// 	RCLCPP_INFO_STREAM(LOGGER, "joint derivation: \n" << joint_dot << "\n");
		// }

		trajectory_point.positions = joint_values;
		trajectory.joint_trajectory.points.push_back(trajectory_point);
	}
	
	
	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	moveit::core::RobotStatePtr end_state = move_group.getCurrentState();
	end_state->copyJointGroupPositions(joint_model_group, joint_values);

	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	// std::cout << trajectory.joint_trajectory.points.size() << std::endl;
	// for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); i++){
	// 	for (std::size_t j = 0; j < joint_names.size(); j++){
	// 		printf("%.5f\t",  trajectory.joint_trajectory.points[i].positions[j]);
	// 	}
	// 	printf("\n");
	// }

	robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), PLANNING_GROUP);
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

	trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
	time_param.computeTimeStamps(rt, 1.0, 1.0);

	moveit_msgs::msg::RobotTrajectory rt_msg;
	rt.getRobotTrajectoryMsg(rt_msg);

	moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
	combined_plan.trajectory_.joint_trajectory = rt_msg.joint_trajectory;

	combined_plan.trajectory_.joint_trajectory = trajectory.joint_trajectory;

	for(std::size_t i = 0; i <  my_plan.trajectory_.joint_trajectory.points.size(); i++) {
		for(std::size_t j = 0; j <  my_plan.trajectory_.joint_trajectory.points[i].positions.size(); j++) {
			printf("%.5f\t",  my_plan.trajectory_.joint_trajectory.points[i].positions[j]);
		}
		printf("\n");
		printf("time_from_start %u %u\n", my_plan.trajectory_.joint_trajectory.points[i].time_from_start.sec, my_plan.trajectory_.joint_trajectory.points[i].time_from_start.nanosec);
		printf("\n");
	}


	move_group.execute(combined_plan);

	current_state = move_group.getCurrentState();
	// const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("ee_link");

	RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
	RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

	// moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
	// current_state->copyJointGroupPositions(joint_model_group, joint_values);

	// robot_state need to be updated manually every time you want to get the current state
	current_state->copyJointGroupPositions(joint_model_group, joint_values);
	for (std::size_t i = 0; i < joint_names.size(); i++)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	// Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	// Eigen::MatrixXd jacobian;
	current_state->getJacobian(joint_model_group, current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
							reference_point_position, jacobian);
	RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

	rclcpp::shutdown();
	return 0;
}
/*
	left:  
	msg.position.x = 1.3066;
	msg.position.y = -0.11213;
	msg.position.z = 1.0248;
	msg.orientation.x = -4.456e-05;
	msg.orientation.y = -2.6453e-06;
	msg.orientation.z = -0.70255;
	msg.orientation.w = 0.71164;

	right:  
	msg.position.x = 1.3043;
	msg.position.y = 0.16549;
	msg.position.z = 1.0248;
	msg.orientation.x = -0.0018889;
	msg.orientation.y = -0.0021159;
	msg.orientation.z = -0.61066;
	msg.orientation.w = 0.79189;

	home:
	msg.position.x = 1.3197;
	msg.position.y = 0.095671;
	msg.position.z = 1.0248;
	msg.orientation.x = -6.6545e-05;
	msg.orientation.y = 1.2237e-06;
	msg.orientation.z = -0.63299; 
	msg.orientation.w = 0.77416;
*/