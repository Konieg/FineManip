#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "Communication.h"

#include <memory>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick and place demo");
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

bool try_plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group,
						  const geometry_msgs::msg::Pose &target_pose,
						  int max_attempts = 20)
{
	bool success = false;  // 记录是否成功
	int attempt_count = 0; // 记录尝试次数

	// 设置目标位姿
	move_group.setPoseTarget(target_pose);
	move_group.setPlanningTime(10.0);

	// 尝试最多max_attempts次规划和执行
	while (attempt_count < max_attempts && !success)
	{
		// 进行规划
		bool plan_success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // 判断规划是否成功

		if (plan_success)
		{
			RCLCPP_INFO(LOGGER, "Plan successful, attempting to execute...");

			// 尝试执行计划
			moveit::core::MoveItErrorCode execute_result = move_group.execute(my_plan);

			if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
			{
				RCLCPP_INFO(LOGGER, "Plan executed successfully.");
				success = true; // 规划和执行都成功
			}
			else
			{
				// 执行失败，尝试重新规划和执行
				attempt_count++;
				RCLCPP_WARN(LOGGER, "Plan execute fail in the %d time attempt", attempt_count);
			}
		}
		else
		{
			// 规划失败，增加尝试次数
			attempt_count++;
			RCLCPP_WARN(LOGGER, "Plan fail in the %d time attempt", attempt_count);
		}
	}

	// 如果尝试了max_attempts次后仍然失败，记录错误日志
	if (!success)
	{
		RCLCPP_ERROR(LOGGER, "Plan and execute fail, teke %d attempts", max_attempts);
	}

	return success; // 返回最终是否成功
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("move_group_interface_manipulator", node_options);
	auto communication_node = std::make_shared<Communication>(); // 创建通信节点


	// 创建一个单线程执行器获得机器人状态
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	executor.add_node(communication_node); 
	std::thread([&executor]()
				{ executor.spin(); })
		.detach();

	// 实例化move_group
	static const std::string PLANNING_GROUP = "arm";
	static const std::string GRIPPER_GROUP = "hand";
	moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
	moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// move_group.setPlanningPipelineId("chomp");
	// move_group.setPlannerId("FMTkConfigDefault");  // 设置为FMT算法
	move_group.setMaxVelocityScalingFactor(1);	   // 增加速度到 0.5 倍

	gripper_group.setNamedTarget("open_max");
	gripper_group.move();

	// 获取关于arm规划组的信息
	const moveit::core::JointModelGroup *joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// 可视化工具初始化
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
														move_group.getRobotModel());


	std::vector<std::string> touch_links;
    touch_links.push_back("link_right");
    touch_links.push_back("link_left");


	geometry_msgs::msg::Pose pick_pose, place_pose;

	/*************************************************
	*                   抓取/放置规划
	*************************************************/
	RCLCPP_INFO(LOGGER, "Start to pick and place...");
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
	std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	joint_group_positions[0] =  0.0;  // radians
	joint_group_positions[1] = -1.2516;
	joint_group_positions[2] = -0.1024;
	joint_group_positions[3] =  0.0;
	joint_group_positions[4] = -2.846;
	joint_group_positions[5] =  0.0;
    move_group.setJointValueTarget(joint_group_positions);
	move_group.plan(my_plan);

	// while (rclcpp::ok()){
	// 调用类中的阻塞方法等待消息
	// RCLCPP_INFO(LOGGER, "Waiting for pose publishing...");
	// communication_node->waitForMessage();
	// 获取接收到的目标位置信息并赋值
	// communication_node->getPosition(pick_pose);
	// RCLCPP_INFO(LOGGER, "Get target pick pose!");

	// move_group.setNamedTarget("horizon_pick");
	move_group.move(); 

	// 执行抓取规划
	// if (!try_plan_and_execute(move_group, pick_pose)) {
	// 	rclcpp::shutdown();  // 如果规划失败，退出程序
	// 	return 1;
	// }

	// 在RViz中可视化路径
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.0;
	RCLCPP_INFO(LOGGER, "Visualizing pick plan as trajectory line.");
	visual_tools.publishAxisLabeled(pick_pose, "pick pose");
	visual_tools.publishText(text_pose, "Pose_Goal(Pick)", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();

	// 执行手爪关闭动作
	rclcpp::sleep_for(std::chrono::seconds(1));
	RCLCPP_INFO(LOGGER, "Attach the object to the robot");
	gripper_group.setNamedTarget("close");
	gripper_group.move();

	visual_tools.trigger();

	rclcpp::sleep_for(std::chrono::seconds(1));

	// 将起始状态设置为关节空间当前状态
	move_group.setStartStateToCurrentState();
	

	// 放置物体的目标位姿
	Eigen::Vector3d axis(0.0, 1.0, 0.0);     // 指定沿Y轴的旋转
	// 通过Eigen的AngleAxis来生成四元数, M_PI/2为旋转角度
	Eigen::Quaterniond quaternion(Eigen::AngleAxisd(M_PI, axis));
	place_pose.orientation.x = quaternion.x();
	place_pose.orientation.y = quaternion.y();
	place_pose.orientation.z = quaternion.z();
	place_pose.orientation.w = quaternion.w();
	place_pose.position.x = 0.20;
	place_pose.position.y = -0.10;
	place_pose.position.z = 0.45;

	// 尝试执行规划
	if (!try_plan_and_execute(move_group, place_pose)) {
		rclcpp::shutdown();  // 如果规划失败，退出程序
		return 1;
	}

	// 执行手爪打开操作，放开物体
	gripper_group.setNamedTarget("open_max");
	gripper_group.move();  // 执行打开手爪的动作

	rclcpp::sleep_for(std::chrono::seconds(1));
	communication_node->sendMoveStatus(true);  // 发布运动完成状态

	// 将起始状态设置为关节空间当前状态
	move_group.setStartStateToCurrentState();
	// }

	move_group.setNamedTarget("initial");
    move_group.move();  // 执行回到初始位置的动作

	rclcpp::shutdown();
	return 0;
}
