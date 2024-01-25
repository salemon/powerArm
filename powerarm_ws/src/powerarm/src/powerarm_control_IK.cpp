#include <vector>
#include <fstream>
#include <iostream>
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
	moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
	
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
	// Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	// Eigen::MatrixXd jacobian;
	// current_state->getJacobian(joint_model_group, current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
	// 						reference_point_position, jacobian);
	// RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

	move_group.setPlanningTime(10.0);

	// Set a target Pose
	// std::vector<geometry_msgs::msg::Pose> target_poses;
	geometry_msgs::msg::Pose sample_left;

	// left:
	// sample_left.orientation.x = 0;
	// sample_left.orientation.y = 0;
	// sample_left.orientation.z = -0.7047552466392517;
	// sample_left.orientation.w = 0.7094505429267883;
	// sample_left.position.x = 1.3014137744903564;
	// sample_left.position.y = -0.126162;
	// sample_left.position.z = 1.024908;
	// // target_poses.push_back(sample_pose);

	sample_left.position.x = 1.2841;
	sample_left.position.y = -0.15968;
	sample_left.position.z = 1.0249;
	sample_left.orientation.x = 0;
	sample_left.orientation.y = 0;
	sample_left.orientation.z = -0.70607;
	sample_left.orientation.w = 0.70814;


	// right:
	geometry_msgs::msg::Pose sample_right;
	// sample_pose.orientation.x = -0.0018889;
	// sample_pose.orientation.y = -0.0021159;
	// sample_pose.orientation.z = -0.61066;
	// sample_pose.orientation.w = 0.79189;
	// sample_right.position.x = 1.304;
	// sample_right.position.y = 0.16549;
	// sample_right.position.z = 1.0248;

	sample_right.position.x = 1.2835;
	sample_right.position.y = 0.16549;
	sample_right.position.z = 1.0249;

	// up:
	geometry_msgs::msg::Pose sample_up;
	// sample_pose.orientation.x = -0.0018889;
	// sample_pose.orientation.y = -0.0021159;
	// sample_pose.orientation.z = -0.61066;
	// sample_pose.orientation.w = 0.79189;
	sample_up.position.x = 1.304;
	sample_up.position.y = 0.02668;
	sample_up.position.z = 1.16361;

	// down:
	geometry_msgs::msg::Pose sample_down;
	// sample_pose.orientation.x = -0.0018889;
	// sample_pose.orientation.y = -0.0021159;
	// sample_pose.orientation.z = -0.61066;
	// sample_pose.orientation.w = 0.79189;
	sample_down.position.x = 1.304;
	sample_down.position.y = 0.02668;
	sample_down.position.z = 0.88599;



	// move_group.setPositionTarget(sample_left.position.x , sample_left.position.y, sample_left.position.z, "ee_link");
	move_group.setPoseTarget(sample_left);

	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if(success){
		// for(std::size_t i = 0; i <  my_plan.trajectory_.joint_trajectory.points.size(); i++) {
		// 	my_plan.trajectory_.joint_trajectory.points[i].positions[2] = 0.0;
		// }
		
		current_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
		move_group.setStartState(*current_state);
		move_group.execute(my_plan);
	}
	
	RCLCPP_INFO(LOGGER, "Homing completed!");
	// std::cout << my_plan.trajectory_.joint_trajectory.points[0].velocities.size() << std::endl;
	
	// for(std::size_t i = 0; i <  my_plan.trajectory_.joint_trajectory.points.size(); i++) {
	// 	for(std::size_t j = 0; j <  my_plan.trajectory_.joint_trajectory.points[i].velocities.size(); j++) {
	// 		printf("%.5f\t",  my_plan.trajectory_.joint_trajectory.points[i].velocities[j]);
	// 		}
	// 	printf("\n");
	// 	printf("time_from_start %u %u\n", my_plan.trajectory_.joint_trajectory.points[i].time_from_start.sec, my_plan.trajectory_.joint_trajectory.points[i].time_from_start.nanosec);
	// }



	// circular path segment
	// const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("ee_link");
	// Eigen::Vector3d cur_pos = end_effector_state.translation();

	// Eigen::Vector3d pos_dot;
	// Eigen::Vector4d joint_dot;

	current_state = move_group.getCurrentState();
	current_state->copyJointGroupPositions(joint_model_group, joint_values);

	for (std::size_t i = 0; i < joint_names.size(); i++)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	float r = (sample_right.position.y - sample_left.position.y) / 2.0; //-0.05
	// float dy;
	// float dz;
	// float dt = 0.1;

	int num_samples = 64;
	// double dy = (sample_right.position.y - sample_left.position.y) / num_samples;
	// RCLCPP_INFO(LOGGER, "step size %f", dy);

	geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose("ee_link").pose;
	RCLCPP_INFO_STREAM(LOGGER, "Current Pose before plan: \n" << geometry_msgs::msg::to_yaml(current_pose));

	std::vector<geometry_msgs::msg::Pose> waypoints;
	waypoints.push_back(current_pose);

	moveit::core::RobotState start_state = *current_state;

	float y_init = current_pose.position.y;
	float z_init = current_pose.position.z;

	for (int i = 0; i < num_samples; i++){
		// dy = r * std::sin(360.0 / num_samples * (i + 1.0) / 180.0 * pi) * 1.0 / 180.0 * pi;
		// dz = r * std::cos(360.0 / num_samples * (i + 1.0) / 180.0 * pi) * 1.0 / 180.0 * pi;

		current_pose.position.y = y_init + r * (1 - std::cos(360.0 / num_samples * (i + 1.0) / 180.0 * pi));
		current_pose.position.z = z_init + r * std::sin(360.0 / num_samples * (i + 1.0) / 180.0 * pi);

		move_group.setPositionTarget(current_pose.position.x, current_pose.position.y, current_pose.position.z, "ee_link");

		success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (!success){
			RCLCPP_ERROR(LOGGER, "Position %d is %s", i, success ? "" : "FAILED");
		}

		// my_plan.trajectory_.joint_trajectory.points.back().positions[2] = 0;
		current_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
		const Eigen::Isometry3d& current_tf_pose = current_state->getGlobalLinkTransform("ee_link");

		current_pose.position.x = current_tf_pose.translation().x();
		current_pose.position.y = current_tf_pose.translation().y();
		current_pose.position.z = current_tf_pose.translation().z();

		Eigen::Quaterniond q = (Eigen::Quaterniond)current_tf_pose.linear();
		current_pose.orientation.x = q.x();
		current_pose.orientation.y = q.y();
		current_pose.orientation.z = q.z();
		current_pose.orientation.w = q.w();

		if(current_pose.orientation.w < 0){
			current_pose.orientation.x *= -1;
			current_pose.orientation.y *= -1;
			current_pose.orientation.z *= -1;
			current_pose.orientation.w *= -1;
		}
		
		// RCLCPP_INFO_STREAM(LOGGER, "Current Pose: \n" << geometry_msgs::msg::to_yaml(current_pose));
		// RCLCPP_INFO(LOGGER, "Caculated dy: %.5f, dz: %.5f \n", dy, dz);

		move_group.setStartState(*current_state);
		waypoints.push_back(current_pose);
	}

	move_group.setStartState(start_state);
	// current_pose = move_group.getCurrentPose("ee_link").pose;
	// RCLCPP_INFO_STREAM(LOGGER, "Current Pose after plan: \n" << geometry_msgs::msg::to_yaml(current_pose));

	moveit_msgs::msg::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	RCLCPP_INFO(LOGGER, "Visualizing circle plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
	
	moveit::core::robotStateToRobotStateMsg(start_state, my_plan.start_state_);
	my_plan.trajectory_ = trajectory;

	// current_state->copyJointGroupPositions(joint_model_group, joint_values);

	// for (std::size_t i = 0; i < joint_names.size(); i++)
	// {
	// 	RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	// }

	start_state.copyJointGroupPositions(joint_model_group, joint_values);

	for (std::size_t i = 0; i < joint_names.size(); i++)
	{
		RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}
	
	robot_trajectory::RobotTrajectory r_trajectory(move_group.getRobotModel(), PLANNING_GROUP);
	r_trajectory.setRobotTrajectoryMsg(start_state, trajectory);

	trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.01, 0.1, 0.001);
	totg.computeTimeStamps(r_trajectory, 1, 1);
	
	moveit_msgs::msg::RobotTrajectory trajectory_velocity_changed;
	r_trajectory.getRobotTrajectoryMsg(trajectory_velocity_changed);

	move_group.execute(trajectory_velocity_changed);

	// for(std::size_t i = 0; i <  trajectory_velocity_changed.joint_trajectory.points.size(); i++) {
	// 	for(std::size_t j = 0; j <  trajectory_velocity_changed.joint_trajectory.points[i].velocities.size(); j++) {
	// 		printf("%.5f\t", trajectory_velocity_changed.joint_trajectory.points[i].velocities[j]);
	// 	}
	// 	printf("\n");
	// 	printf("time_from_start %u %u\n", trajectory_velocity_changed.joint_trajectory.points[i].time_from_start.sec, trajectory_velocity_changed.joint_trajectory.points[i].time_from_start.nanosec);
	// }
	
	// move_group.execute(trajectory);
	// move_group.setStartState(start_state);
	// move_group.setPositionTarget(sample_right.position.x, sample_right.position.y, sample_right.position.z, "ee_link");
	// success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	// if (!success){
	// 	RCLCPP_ERROR(LOGGER, "Position is %s", success ? "" : "FAILED");
	// }

	// move_group.execute(my_plan);

	std::stringstream ss;

    for (unsigned i=0; i< trajectory_velocity_changed.joint_trajectory.points.size(); i++)
    {
		
		ss
			<< trajectory_velocity_changed.joint_trajectory.points[i].positions[0] << "," 
			<< trajectory_velocity_changed.joint_trajectory.points[i].positions[1] << "," 
			<< trajectory_velocity_changed.joint_trajectory.points[i].positions[2] << "," 
			<< trajectory_velocity_changed.joint_trajectory.points[i].positions[3] << "," 
			<< trajectory_velocity_changed.joint_trajectory.points[i].velocities[0] << "," 
			<< trajectory_velocity_changed.joint_trajectory.points[i].velocities[1] << "," 
			<< trajectory_velocity_changed.joint_trajectory.points[i].velocities[2] << "," 
			<< trajectory_velocity_changed.joint_trajectory.points[i].velocities[3] << "," 

		<< std::endl;

		if(i == (trajectory_velocity_changed.joint_trajectory.points.size()-1))
		{
			std::ofstream outfile ("/home/wenda/powerarm_ws/points.txt",std::ios::out);
			if(!outfile.is_open())
			{
				RCLCPP_INFO(LOGGER, "open failed");
			}
			else
			{
				outfile << "There are a total of " << i + 1 << " points in the trajectory" <<std::endl; //<<count<<std::endl;
				//outfile<<finger_state[i][0]<<","<<finger_state[i][1]<<","<<finger_state[i][2]<<std::endl;
				//multi_dof_joint_trajectory.joint_names[0]<<std::endl;
				outfile << ss.str() <<std::endl;
				outfile.close();
				
				RCLCPP_INFO(LOGGER, "File created");
			}
		}
    }

	RCLCPP_INFO(LOGGER, "Completed");

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

	sample_left.orientation.x = 0;
	sample_left.orientation.y = 0;
	sample_left.orientation.z = -0.7047552466392517;
	sample_left.orientation.w = 0.7094505429267883;
	sample_left.position.x = 1.3014137744903564;
	sample_left.position.y = -0.126162;
	sample_left.position.z = 1.024908;

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