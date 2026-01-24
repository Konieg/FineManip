#include "Communication.h"
#include <chrono>
#include <thread>
//后续添加可删除
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>

// 构造函数，初始化订阅者和发布者
Communication::Communication() : Node("communication_node"), received_valid_pose(false) {
    // 初始化订阅者，接收/arm_pose话题
    pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
        "arm_pose", 10,
        std::bind(&Communication::poseCallback, this, std::placeholders::_1)
    );

    // 初始化发布者，发送完成信号
    status_publisher = this->create_publisher<std_msgs::msg::Bool>("move_status", 10);

    // 初始化反馈发布者，发送消息接收成功信号
    feedback_publisher = this->create_publisher<std_msgs::msg::Bool>("feedback_topic", 10);
}

// 获取目标位姿
void Communication::getPosition(geometry_msgs::msg::Pose& target) const {
    target = target_pose_;
}

// 检查Pose是否有效的辅助函数
bool Communication::is_valid_pose(const geometry_msgs::msg::Pose& msg) {
    // 检查 x, y, z 是否为有效的浮点值
    if (std::isnan(msg.position.x) || std::isnan(msg.position.y) || std::isnan(msg.position.z)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pose: NaN value detected in x, y, or z.");
        return false;
    }
    return true;
}

// 回调函数，用于接收并更新目标位置
void Communication::poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Recieve pose.");
    if (received_valid_pose)
        return; 

    // 检查每个pose是否有效
    if (!is_valid_pose(*msg)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pose detected.");
        return; // 如果任何一个pose无效，忽略这个消息
    }

    // 更新目标位姿
    target_pose_ = *msg;

    RCLCPP_INFO(this->get_logger(), "Received target positions: x=%f, y=%f, z=%f, =%f, =%f, =%f",
                target_pose_.position.x, target_pose_.position.y, target_pose_.position.z,
                target_pose_.orientation.x, target_pose_.orientation.y, target_pose_.orientation.z);

    // 标记为接收到有效位置信息
    if(is_processing_message)
        received_valid_pose = true;

    // 发布反馈消息，通知发送端已成功接收
    auto feedback_msg = std_msgs::msg::Bool();
    feedback_msg.data = true; // true 表示接收成功
    feedback_publisher->publish(feedback_msg);

    RCLCPP_INFO(this->get_logger(), "Feedback sent: PoseArray received successfully.");
}

// 等待接收消息的阻塞方法
void Communication::waitForMessage() {
    is_processing_message=true;
    // 每两秒检查一次是否接收到成功的消息
    while (rclcpp::ok()) {
        // RCLCPP_INFO(this->get_logger(), "Checking received_valid_pose: %s", received_valid_pose ? "true" : "false");
        if (received_valid_pose) {
            RCLCPP_INFO(this->get_logger(), "Message received successfully. Exiting loop.");
            received_valid_pose = false;  // 重置标志以接受下一个消息
            is_processing_message=false;
            break;  // 退出循环
        } 
        // else {
        //     RCLCPP_INFO(this->get_logger(), "Waiting for message...");
        //     std::this_thread::sleep_for(std::chrono::seconds(2));  // 每两秒等待
        // }
    }
}


// 发送运动完成状态
void Communication::sendMoveStatus(bool status) {
    auto msg = std_msgs::msg::Bool();
    msg.data = status;
    status_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Movement completed. Status sent: %s", status ? "true" : "false");
}
