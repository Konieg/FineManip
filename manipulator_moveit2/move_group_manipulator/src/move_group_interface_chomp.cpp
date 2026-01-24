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

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_arm_move_group_chomp_demo");
const double CYLINDER_HEIGHT = 0.075;
const double CYLINDER_RADIUS = 0.025;

bool traj_plan(moveit::planning_interface::MoveGroupInterface &move_group,
                        const std::vector<double> &joint_target, 
                        moveit::planning_interface::MoveGroupInterface::Plan &plan_output, 
                        int max_attempts = 20)
{
    bool success = false;
    int attempt_count = 0;
    
    move_group.setPlanningTime(5.0);
    move_group.setJointValueTarget(joint_target);

    // 1. 获取 RobotModel，用于后续计算
    auto robot_model_ptr = move_group.getRobotModel();
    // 获取当前规划组的名字（例如 "arm"）
    std::string group_name = move_group.getName();

    while (attempt_count < max_attempts && !success)
    {
        // 尝试规划
        success = (move_group.plan(plan_output) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            // ================== 手动时间参数化核心代码 ==================
            RCLCPP_INFO(LOGGER, "Planning success. Computing timestamps manually...");

            // 1. 将 Plan 转换为 RobotTrajectory 对象，方便处理
            robot_trajectory::RobotTrajectory rt(robot_model_ptr, group_name);
            
            // 注意：这里需要传入当前机器人的状态作为参考
            rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), plan_output.trajectory_);

            // 2. 创建时间优化求解器 (Time Optimal Trajectory Generation - TOTG)
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;

            // 3. 计算时间戳
            // 参数2: 速度缩放因子 (0.2 表示 20% 速度)
            // 参数3: 加速度缩放因子 (0.2 表示 20% 加速度)
            // 这里的数值必须 > 0 且 <= 1.0
            bool time_param_success = totg.computeTimeStamps(rt, 0.8, 0.8);

            if (time_param_success) {
                // 4. 将计算好速度的轨迹写回 plan_output
                rt.getRobotTrajectoryMsg(plan_output.trajectory_);
                RCLCPP_INFO(LOGGER, "Time parameterization FINISHED. Velocity added.");
            } else {
                RCLCPP_ERROR(LOGGER, "Time parameterization FAILED.");
                success = false; // 如果算不出速度，就视为规划失败
            }
            // ==========================================================
        } 
        else 
        {
            attempt_count++;
            RCLCPP_WARN(LOGGER, "Planning failed, attempt %d/%d", attempt_count, max_attempts);
        }
    }
    return success;
}

/*
bool traj_plan(moveit::planning_interface::MoveGroupInterface &move_group,
							const std::vector<double> &joint_target, moveit::planning_interface::MoveGroupInterface::Plan &plan, 
							int max_attempts = 20)
{
	bool success = false;  // 记录是否成功
	int attempt_count = 0; // 记录尝试次数
    move_group.setPlanningTime(15.0);

	// 设置关节目标
	move_group.setJointValueTarget(joint_target);

	while (attempt_count < max_attempts && !success)
	{
		// 进行规划
		success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 判断规划是否成功

		if (!success)
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
*/

bool plan_with_ik_retry(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const moveit::core::JointModelGroup *joint_model_group,
    const geometry_msgs::msg::Pose &target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan, 
    int max_ik_attempts = 10) // 尝试10次不同的IK解
{
    RCLCPP_INFO(LOGGER, "Starting IK/Plan retry loop for pose target.");

    auto base_state = move_group.getCurrentState();
    if (!base_state)
    {
        RCLCPP_ERROR(LOGGER, "Failed to get current state");
        return false;
    }

    for (int i = 0; i < max_ik_attempts; ++i)
    {
        moveit::core::RobotState ik_seed_state(*base_state);
        
        // 第一次尝试(i=0)使用当前状态作为种子，后续尝试使用随机种子来寻找不同的IK解
        if (i > 0)
        {
            RCLCPP_INFO(LOGGER, "IK/Plan Attempt %d/%d: Using random seed.", i + 1, max_ik_attempts);
            ik_seed_state.setToRandomPositions(joint_model_group);
        }
        else
        {
            RCLCPP_INFO(LOGGER, "IK/Plan Attempt %d/%d: Using current state as seed.", i + 1, max_ik_attempts);
        }

      
        bool found_ik = ik_seed_state.setFromIK(joint_model_group, target_pose, 0.5); // 0.5秒IK超时
        if (!found_ik)
        {
            RCLCPP_WARN(LOGGER, "IK/Plan Attempt %d: IK failed (no solution found).", i + 1);
            continue; // 尝试下一次循环
        }

        RCLCPP_INFO(LOGGER, "IK/Plan Attempt %d: IK solution found. Checking if plan is feasible for CHOMP.", i + 1);

        // 从已求解的状态中提取关节值
        std::vector<double> joint_target;
        ik_seed_state.copyJointGroupPositions(joint_model_group, joint_target);

        // 尝试规划到关节空间目标
        if (traj_plan(move_group, joint_target, plan, 5)) 
        {
            RCLCPP_INFO(LOGGER, "IK/Plan Attempt %d: Plan and execute successful.", i + 1);
            return true;
        }
        else
        {
            // 这个IK解虽然有效，但无法规划。
            RCLCPP_WARN(LOGGER, "IK/Plan Attempt %d: Planning failed for this valid IK solution. Trying next IK.", i + 1);
        }
    }

    RCLCPP_ERROR(LOGGER, "Failed to find a plannable IK solution after %d attempts.", max_ik_attempts);
    return false;
}


// 辅助函数：打印轨迹详细信息
void print_trajectory_debug(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    const auto &traj = plan.trajectory_.joint_trajectory;
    size_t point_count = traj.points.size();
    
    RCLCPP_INFO(LOGGER, "================ TRAJECTORY DEBUG INFO ================");
    RCLCPP_INFO(LOGGER, "Total Points: %zu", point_count);
    RCLCPP_INFO(LOGGER, "Joint Names: %s, %s, ...", 
                traj.joint_names.size() > 0 ? traj.joint_names[0].c_str() : "None",
                traj.joint_names.size() > 1 ? traj.joint_names[1].c_str() : "None");

    // 遍历每一个轨迹点
    for (size_t i = 0; i < point_count; ++i)
    {
        const auto &p = traj.points[i];
        
        // 计算当前点的时间（秒）
        double time_sec = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;

        // 使用 stringstream 格式化输出，避免日志太乱
        std::stringstream ss_pos, ss_vel;
        
        // 打印前2个关节的值作为示例（你可以根据需要把 j < p.positions.size() 全打印出来）
        for (size_t j = 0; j < std::min((size_t)3, p.positions.size()); ++j) {
            ss_pos << p.positions[j] << " ";
        }

        // 检查速度是否存在
        if (!p.velocities.empty()) {
            for (size_t j = 0; j < std::min((size_t)3, p.velocities.size()); ++j) {
                ss_vel << p.velocities[j] << " ";
            }
        } else {
            ss_vel << "EMPTY";
        }

        // 打印日志： 索引 | 时间 | 位置片段 | 速度片段
        RCLCPP_INFO(LOGGER, "Pt[%zu] T:%.4f | Pos:[%s...] | Vel:[%s...]", 
                    i, time_sec, ss_pos.str().c_str(), ss_vel.str().c_str());
    }
    RCLCPP_INFO(LOGGER, "=======================================================");
}


int main(int argc, char **argv)
{
	// 初始化ROS2节点系统，并创建move_group_interface节点
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("move_group_interface_fines", node_options);

	// 创建一个单线程执行器获得机器人状态
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread executor_thread([&executor]() { executor.spin(); });

	// 实例化move_group
	static const std::string PLANNING_GROUP = "arm";
	static const std::string GRIPPER_GROUP = "hand";
	moveit::planning_interface::MoveGroupInterface arm_group(move_group_node, PLANNING_GROUP);
	moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan traj_plan;
	
	// move_group.setPlanningPipelineId("chomp");
    // arm_group.setPlanningPipelineId("ompl_chomp");
    arm_group.setPlannerId("RRTConnectkConfigDefault");
    arm_group.setPlanningPipelineId("ompl_chomp");
    // move_group.setPlannerId("FMTkConfigDefault");

	arm_group.setMaxVelocityScalingFactor(0.8);	   
	arm_group.setMaxAccelerationScalingFactor(0.8);

	gripper_group.setNamedTarget("open");
	gripper_group.move();

	// 获取关于arm规划组的信息
	const moveit::core::JointModelGroup *joint_model_group =
		arm_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// 可视化工具初始化
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
														arm_group.getRobotModel());
  
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();

	// 创建圆柱体(collision_objects)
	std::vector<std::string> object_ids = {"cylinder_1", "cylinder_2", "cylinder_3"};
	std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
	std::vector<moveit_msgs::msg::ObjectColor> object_colors;

	shape_msgs::msg::SolidPrimitive cylinder_primitive;
	cylinder_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	cylinder_primitive.dimensions.resize(2);
	cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = CYLINDER_HEIGHT;
	cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = CYLINDER_RADIUS;

	for (int i = 0; i < 3; ++i) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";          // 使用世界坐标系作为物体初始位置的参考

        collision_object.id = object_ids[i];


        geometry_msgs::msg::Pose object_pose;
        object_pose.orientation.w = 1.0;
        object_pose.position.x = 0.3;  
        object_pose.position.y = 0.1 - i * 0.15;
        object_pose.position.z = 0.075 / 2;

		moveit_msgs::msg::ObjectColor object_color;
    	object_color.id = object_ids[i];

        collision_object.primitives.push_back(cylinder_primitive);
        collision_object.primitive_poses.push_back(object_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);

		object_color.color.a = 1.0;  // 不透明度（1为完全不透明）
    	object_colors.push_back(object_color);
    }

	// 添加物体到场景
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
    box_color.id = "table_box";
    box_color.color.a = 1.0;    // 不透明
    box_color.color.r = 0;    // 灰色
    box_color.color.g = 0.5;
    box_color.color.b = 0;

    collision_objects.push_back(box_object);
    object_colors.push_back(box_color);

	planning_scene_interface.addCollisionObjects(collision_objects, object_colors);

	std::vector<std::string> touch_links;
    touch_links.push_back("link_right");
    touch_links.push_back("link_left");


	const double GRASP_Z_OFFSET = 0.28;

    Eigen::Vector3d axis_vec(0.0, 1.0, 0.0);
    Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI, axis_vec));
    geometry_msgs::msg::Quaternion grasp_orientation;
    grasp_orientation.x = quat.x();
    grasp_orientation.y = quat.y();
    grasp_orientation.z = quat.z();
    grasp_orientation.w = quat.w();

    /*************************************************
    *                    抓取/放置规划
    *************************************************/
    for (int i = 0; i < 3; ++i){   
        RCLCPP_INFO(LOGGER, "Start to pick and place %s", object_ids[i].c_str());

        // Pick位姿
		geometry_msgs::msg::Pose pick_pose;
		pick_pose.orientation = grasp_orientation;

		pick_pose.position.x = 0.3; 
		pick_pose.position.y = 0.1 - i * 0.15;
		pick_pose.position.z = 0.075 / 2 + GRASP_Z_OFFSET;

        // Pick规划
        RCLCPP_INFO(LOGGER, "Planning to PICK pose");
        if (!plan_with_ik_retry(arm_group, joint_model_group, pick_pose, traj_plan))
        {
            RCLCPP_ERROR(LOGGER, "Pick plan failed for %s. Skipping.", object_ids[i].c_str());
            continue; // 跳过这个物体
        }

        RCLCPP_INFO(LOGGER, "Pick plan successful. analyzing trajectory...");
        print_trajectory_debug(traj_plan);

        RCLCPP_INFO(LOGGER, "Pick plan successful. Visualizing...");
        visual_tools.publishAxisLabeled(pick_pose, "pick pose");
        visual_tools.publishTrajectoryLine(traj_plan.trajectory_, joint_model_group);
        visual_tools.trigger();

        if (arm_group.execute(traj_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Pick execution failed for %s. Skipping.", object_ids[i].c_str());
            continue;
        }

        // 规划一个线性的笛卡尔路径（向下）
        RCLCPP_INFO(LOGGER, "Reached pre-grasp pose. Moving down to grasp...");
		geometry_msgs::msg::Pose grasp_pose = pick_pose;
		double approach_distance = 0.06;
		grasp_pose.position.z -= approach_distance;
		
		// 规划一个线性的笛卡尔路径（向下）
		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.push_back(grasp_pose);

		moveit_msgs::msg::RobotTrajectory trajectory_down;
		double fraction = arm_group.computeCartesianPath(waypoints,
                                                	0.01,  // eef_step：每1cm一个点
                                                	0.0,   // jump_threshold：不允许跳跃
                                                	trajectory_down);

		if (fraction < 1.0)
		{
			RCLCPP_WARN(LOGGER, "Failed to plan linear approach (only %f achieved)", fraction);
			// 可以在这里选择跳过此物体
			continue; 
		}
        arm_group.execute(trajectory_down);
        visual_tools.trigger();

        // 执行手爪关闭动作
        // rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(LOGGER, "Attach cylinder %d to the robot", i);
        gripper_group.setNamedTarget("close");
        gripper_group.move();

        // rclcpp::sleep_for(std::chrono::seconds(1));
        gripper_group.attachObject(collision_objects[i].id, "link_hand", touch_links);
        visual_tools.trigger();

        // 将起始状态设置为关节空间当前状态
		arm_group.setStartStateToCurrentState();

        // Place位姿
        geometry_msgs::msg::Pose place_pose;
		place_pose.orientation = grasp_orientation;

        switch (i){
            case 0:{
				place_pose.position.x = -0.24;
                place_pose.position.y = 0.10;
                break;
            }
            case 1:{
                place_pose.position.x = -0.17;
                place_pose.position.y = 0.12;
                break;
            }
            case 2:{
                place_pose.position.x = -0.22;
                place_pose.position.y = -0.12;
                break;
            }
            default:
                break;
        }
        place_pose.position.z = 0.44;


        visual_tools.deleteAllMarkers();
        // 规划到“放置”位姿
        RCLCPP_INFO(LOGGER, "Planning to PLACE pose");
        if (!plan_with_ik_retry(arm_group, joint_model_group, place_pose, traj_plan))
        {
            RCLCPP_ERROR(LOGGER, "Pick plan failed for %s. Skipping.", object_ids[i].c_str());
            continue; // 跳过这个物体
        }

        RCLCPP_INFO(LOGGER, "Place plan successful. Visualizing...");
        visual_tools.publishAxisLabeled(place_pose, "place pose");
        visual_tools.publishTrajectoryLine(traj_plan.trajectory_, joint_model_group);
        visual_tools.trigger();


        if (arm_group.execute(traj_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Place execution failed for %s. Skipping.", object_ids[i].c_str());
            continue;
        }

        gripper_group.setNamedTarget("open");
        gripper_group.move();
        gripper_group.detachObject(collision_objects[i].id);


        visual_tools.trigger();
        visual_tools.deleteAllMarkers();

        // 将起始状态设置为关节空间当前状态
		arm_group.setStartStateToCurrentState();
    }

	/*************************************************
	*                     结束
	*************************************************/
	visual_tools.prompt("Finish all the pick and place tasks.");
	// 最后移除所有物体
    rclcpp::sleep_for(std::chrono::seconds(1));
    planning_scene_interface.removeCollisionObjects(object_ids);

	// 结束可视化
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();

	rclcpp::shutdown();
	return 0;
}
