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

#include <memory>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_arm_move_group_demo");
const double CYLINDER_HEIGHT = 0.075;
const double CYLINDER_RADIUS = 0.025;

bool trajectory_plan(moveit::planning_interface::MoveGroupInterface &move_group,
						  moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
						  const geometry_msgs::msg::Pose &target_pose,
						  int max_attempts = 20)
{
	bool success = false;  // 记录是否成功
	int attempt_count = 0; // 记录尝试次数

	// 设置目标位姿
	move_group.setPoseTarget(target_pose);
	move_group.setPlanningTime(10.0);

	// 尝试最多max_attempts次规划和执行
	while (attempt_count < max_attempts && !success){
		// 进行规划
		success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // 判断规划是否成功

		if (!success){
			// 规划失败，增加尝试次数
			attempt_count++;
			RCLCPP_WARN(LOGGER, "Plan fail in the %d time attempt", attempt_count);
		}
	}

	// 如果尝试了max_attempts次后仍然失败，记录错误日志
	if (!success){
		RCLCPP_ERROR(LOGGER, "Plan and execute fail, teke %d attempts", max_attempts);
	}

	return success; // 返回最终是否成功
}

int main(int argc, char **argv)
{
	RCLCPP_WARN(LOGGER, "Start new demo");
	// 初始化ROS2节点系统，并创建move_group_interface节点
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("move_group_interface_manipulator", node_options);

	// 创建一个单线程执行器获得机器人状态
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread executor_thread([&executor]() { executor.spin(); });
	// std::thread([&executor]()
	// 			{ executor.spin(); })
	// 	.detach();

	// 实例化move_group
	static const std::string PLANNING_GROUP = "arm";
	static const std::string GRIPPER_GROUP = "hand";
	moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
	moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);

	moveit::planning_interface::MoveGroupInterface::Plan traj_plan;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	
	move_group.setPlannerId("FMTkConfigDefault");
	// move_group.setPlannerId("RRTConnectkConfigDefault");
	move_group.setMaxVelocityScalingFactor(1);	   // 增加速度到 1 倍

	gripper_group.setNamedTarget("open");
	gripper_group.move();

	// 获取关于arm规划组的信息
	const moveit::core::JointModelGroup *joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// 可视化工具初始化
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
														move_group.getRobotModel());
  
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();

	// 创建圆柱体(collision_objects)
	std::vector<std::string> object_ids = {"cylinder_1", "cylinder_2", "cylinder_3"};
	std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
	std::vector<moveit_msgs::msg::ObjectColor> object_colors;

	for (int i = 0; i < 3; ++i) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";  // 使用世界坐标系作为物体初始位置的参考

        collision_object.id = object_ids[i];

        shape_msgs::msg::SolidPrimitive cylinder_primitive;
        cylinder_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        cylinder_primitive.dimensions.resize(2);
        cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = CYLINDER_HEIGHT;
        cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = CYLINDER_RADIUS;


        geometry_msgs::msg::Pose cylinder_pose;
        cylinder_pose.orientation.w = 1.0;
        cylinder_pose.position.x = 0.3;  
        cylinder_pose.position.y = 0.15 - i * 0.15;
        cylinder_pose.position.z = 0.075 / 2;

		moveit_msgs::msg::ObjectColor cylinder_color;
    	cylinder_color.id = object_ids[i];

        collision_object.primitives.push_back(cylinder_primitive);
        collision_object.primitive_poses.push_back(cylinder_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);

		cylinder_color.color.a = 1.0;  // 不透明度（1为完全不透明）
    	object_colors.push_back(cylinder_color);
    }

	object_colors[0].color.r = 1.0;  // 红色
	object_colors[0].color.g = 0.0;
	object_colors[0].color.b = 0.0;

	object_colors[1].color.r = 0.0;  // 绿色
	object_colors[1].color.g = 1.0;
	object_colors[1].color.b = 0.0;

	object_colors[2].color.r = 0.0;  // 蓝色
	object_colors[2].color.g = 0.0;
	object_colors[2].color.b = 1.0;

	// 放置桌面
	moveit_msgs::msg::CollisionObject box_object;
    box_object.header.frame_id = "world";
    box_object.id = "table_box";          

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.12; 
    box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.30; 
    box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.008; 

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0; // 默认姿态 (无旋转)
    box_pose.position.x = -0.18;   
    box_pose.position.y = 0.0;   
    box_pose.position.z = 0.17;   

    box_object.primitives.push_back(box_primitive);
    box_object.primitive_poses.push_back(box_pose);
    box_object.operation = box_object.ADD;

    moveit_msgs::msg::ObjectColor box_color;
    box_color.id = "table_box"; // ID 必须匹配
    box_color.color.a = 1.0;    // 不透明
    box_color.color.r = 0;    // 灰色
    box_color.color.g = 0.5;
    box_color.color.b = 0;

    collision_objects.push_back(box_object);
    object_colors.push_back(box_color);

	// 添加物体到场景
	planning_scene_interface.addCollisionObjects(collision_objects, object_colors);

	std::vector<std::string> touch_links;
    touch_links.push_back("link_right");
    touch_links.push_back("link_left");

	const double GRASP_Z_OFFSET = 0.22; // 抓取时TCP的Z值 = 物体Z中心 + 此偏移

	// 抓取/放置的通用朝向 (Top-down Grasp)
    Eigen::Vector3d axis_vec(0.0, 1.0, 0.0);
    Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI, axis_vec));
    geometry_msgs::msg::Quaternion grasp_orientation;
    grasp_orientation.x = quat.x();
    grasp_orientation.y = quat.y();
    grasp_orientation.z = quat.z();
    grasp_orientation.w = quat.w();


	/*************************************************
	*                   抓取/放置规划
	*************************************************/

	for (int i = 0; i < 3; ++i){
		RCLCPP_INFO(LOGGER, "Start to pick and place %s", object_ids[i].c_str());

		// Pick位姿
		geometry_msgs::msg::Pose pick_pose;
		pick_pose.orientation = grasp_orientation;

		pick_pose.position.x = 0.3; 
		pick_pose.position.y = 0.15 - i * 0.15;
		pick_pose.position.z = 0.075 / 2 + GRASP_Z_OFFSET;

		// Pick规划
		bool pick_success = false;
		pick_success = trajectory_plan(move_group, traj_plan, pick_pose);
		if (!pick_success) {
			RCLCPP_ERROR(LOGGER, "Pick plan failed for %s. Skipping.", object_ids[i].c_str());
            continue; // 跳过这个物体
		}

		RCLCPP_INFO(LOGGER, "Pick plan successful. Visualizing...");
        visual_tools.publishAxisLabeled(pick_pose, "pick pose");
        visual_tools.publishTrajectoryLine(traj_plan.trajectory_, joint_model_group);
        visual_tools.trigger();

		if (move_group.execute(traj_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Pick execution failed for %s. Skipping.", object_ids[i].c_str());
            continue;
        }
									

		// 执行手爪关闭动作
		rclcpp::sleep_for(std::chrono::seconds(1));
		RCLCPP_INFO(LOGGER, "Attach the object to the robot");
		gripper_group.setNamedTarget("close");
		gripper_group.move();

		gripper_group.attachObject(collision_objects[i].id, "link_hand", touch_links);
		visual_tools.trigger();
		visual_tools.deleteAllMarkers();

		// 将起始状态设置为关节空间当前状态
		move_group.setStartStateToCurrentState();

		// Place位姿
		geometry_msgs::msg::Pose place_pose;
		place_pose.orientation = grasp_orientation;

		switch (i){
			case 0:{
				place_pose.position.x = -0.22;   
				place_pose.position.y = 0.10;
				place_pose.position.z = 0.44;
				break;
			}
			case 1:{
				place_pose.position.x = -0.15; 
				place_pose.position.y = 0.12;
				place_pose.position.z = 0.44;
				break;
			}
			case 2:{
				place_pose.position.x = -0.22;  
				place_pose.position.y = -0.12;
				place_pose.position.z = 0.44;
				break;
			}
			default:
				break;
		}

		bool place_success = false;
		place_success = trajectory_plan(move_group, traj_plan, place_pose);

		// Place规划
		if (!place_success)
        {
            RCLCPP_ERROR(LOGGER, "Pick plan failed for %s. Skipping.", object_ids[i].c_str());
            continue; // 跳过这个物体
        }
		

		RCLCPP_INFO(LOGGER, "Place plan successful. Visualizing...");
        visual_tools.publishAxisLabeled(place_pose, "place pose");
        visual_tools.publishTrajectoryLine(traj_plan.trajectory_, joint_model_group);
        visual_tools.trigger();

		if (move_group.execute(traj_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Pick execution failed for %s. Skipping.", object_ids[i].c_str());
            continue;
        }


		// 执行手爪打开操作，放开物体
		gripper_group.setNamedTarget("open");
		gripper_group.move();  // 执行打开手爪的动作
		gripper_group.detachObject(collision_objects[i].id);
		visual_tools.trigger();

        rclcpp::sleep_for(std::chrono::seconds(1));

		visual_tools.deleteAllMarkers();

		// 将起始状态设置为关节空间当前状态
		move_group.setStartStateToCurrentState();
	}

	/*************************************************
	*                     结束
	*************************************************/
	visual_tools.prompt("Finish all the pick and place tasks.");
	
	// 最后移除所有物体
    rclcpp::sleep_for(std::chrono::seconds(1));
    planning_scene_interface.removeCollisionObjects(object_ids);

	visual_tools.deleteAllMarkers();
	visual_tools.trigger();

	rclcpp::shutdown();
	if (executor_thread.joinable()){
        executor_thread.join();
    }

	return 0;
}
