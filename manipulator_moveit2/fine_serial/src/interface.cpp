#include <iostream>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include "serial_station.hpp"
#include "msg_types.hpp"
#include "utils/crc.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <example_interfaces/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>


// 带有帧标识符、长度和 CRC 的编码函数
void encode(std::vector<uint8_t>& data, uint8_t frame_identifier) {
    const uint16_t FRAME_HEADER = 0xAA;
    const uint16_t FRAME_TRAILER = 0xBB;
    // 计算 CRC 校验码并插入到帧尾前
    uint8_t crc = calculate_crc(data);
    // 动态计算数据长度，并将其作为长度字段（1 字节）
    uint8_t data_length = static_cast<uint8_t>(data.size());

    // 在 data 的开头插入帧头、帧标识符和标识符长度
    data.insert(data.begin(), data_length);  // 插入标识符长度
    data.insert(data.begin(), frame_identifier);   // 插入帧标识符
    data.insert(data.begin(), FRAME_HEADER);

    data.push_back(crc);
    // 在 data 的末尾添加帧尾
    data.push_back(FRAME_TRAILER);

}

// 解码函数
void decode(std::vector<uint8_t> data, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher)
{
    std::string data_str;
    for (const auto& byte : data)
    {
        data_str += fmt::format("{:02X}", byte);
    }

    // static int YYY = 0;
    // std::cout << "Index: " << YYY << " Received: " << data_str.c_str() << std::endl;
    // YYY++;

    // 如果接收到数据为"AA05010000BB"，则创建话题并发布1.0的float32
    if (data_str == "AA01BB") {
        std::cout << "话题: finish" << std::endl;

        // 创建消息并发布
        std_msgs::msg::Float32 msg;
        msg.data = 1.0f;
        publisher->publish(msg);
    }
}
// 处理接收到机械臂的轨迹点
void trajectory_manipulator_callback(
        const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg,
        std::shared_ptr<SerialStation> serial_station) {

    // 先打印收到的轨迹点信息
    std::string positions_str;
    for (const auto& pos : msg->positions) {
        positions_str += fmt::format("{:.6f} ", pos);  // 格式化打印浮点数，6位小数
    }
    RCLCPP_INFO(rclcpp::get_logger("serial_station"), "Received trajectory point positions: [%s]", positions_str.c_str());

    // 将轨迹点的数据转换为字节数组
    // 将轨迹点的数据转换为字节数组
    std::vector<uint8_t> data;
    for (size_t i = 0; i < msg->positions.size(); ++i) {
        // 获取当前位置值
        double pos = msg->positions[i];

        // 如果是第二个位置（索引1），将其取反
        if (i == 1) {
            pos = -pos;
        }

        if(i==2){
            pos =-pos;
        }
        
        if(i==5){
        pos=-pos;
        }

        // 将 double 转换为 float
        float float_pos = static_cast<float>(pos);

        // 将 float 转换为 uint32_t 的二进制表示
        uint32_t as_int = *reinterpret_cast<uint32_t*>(&float_pos);

        // 将 uint32_t 的每个字节插入到 data 数组中
        data.push_back(static_cast<uint8_t>(as_int & 0xFF));         // 低字节
        data.push_back(static_cast<uint8_t>((as_int >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>((as_int >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((as_int >> 24) & 0xFF)); // 高字节
    }

    // 打印将要发送的数据（原始数据）到日志
    std::string data_str;
    for (const auto& byte : data) {
        data_str += fmt::format("{:02X} ", byte);  // 格式化为十六进制
    }

    // 使用 ROS 日志系统记录信息
//    RCLCPP_INFO(rclcpp::get_logger("serial_station"), "Transmitting data: %s", data_str.c_str());
    // 通过串口发送编码后的数据
    serial_station->transmit(data, MOVE_MANIPULATOR);
}


// 处理接收到的相机纠偏数据
void camera_correction_callback(
        const example_interfaces::msg::Float32MultiArray::SharedPtr msg,
        std::shared_ptr<SerialStation> serial_station) {

    // 先打印收到的纠偏数据
    std::string corrections_str;
    for (const auto& value : msg->data) {
        corrections_str += fmt::format("{:.6f} ", value);  // 格式化打印浮点数，6位小数
    }
    RCLCPP_INFO(rclcpp::get_logger("serial_station"), "Received camera correction data: [%s]", corrections_str.c_str());

    // 将纠偏数据转换为字节数组
    std::vector<uint8_t> data;
    for (const auto& value : msg->data) {
        float float_value = static_cast<float>(value);
        uint32_t as_int = *reinterpret_cast<uint32_t*>(&float_value);

        data.push_back(static_cast<uint8_t>(as_int & 0xFF));         // 低字节
        data.push_back(static_cast<uint8_t>((as_int >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>((as_int >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((as_int >> 24) & 0xFF)); // 高字节
    }

    // 打印将要发送的纠偏数据（原始数据）到日志
    std::string data_str;
    for (const auto& byte : data) {
        data_str += fmt::format("{:02X} ", byte);  // 格式化为十六进制
    }

    RCLCPP_INFO(rclcpp::get_logger("serial_station"), "Transmitting camera correction data: %s", data_str.c_str());
    // 通过串口发送编码后的纠偏数据
    serial_station->transmit(data, ODOMETRY_OFFSET);
}

// 自锁控制回调函数
void set_path_joint(
        const std_msgs::msg::Float32::SharedPtr msg,
        std::shared_ptr<SerialStation> serial_station) {
    // 获取传入的浮动数
    float float_value = msg->data;

    // 将浮动数转换为字节数组
    uint32_t as_int = *reinterpret_cast<uint32_t*>(&float_value);

    // 构造要发送的数据
    std::vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>(as_int & 0xFF));         // 低字节
    data.push_back(static_cast<uint8_t>((as_int >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>((as_int >> 16) & 0xFF));
    data.push_back(static_cast<uint8_t>((as_int >> 24) & 0xFF)); // 高字节

    // 打印日志
    RCLCPP_INFO(rclcpp::get_logger("set_path_joint"), "Received float value: %.6f", float_value);

    // 通过串口发送数据
    serial_station->transmit(data, SET_PATH_POINT);
}

// 夹爪状态控制回调函数
void gripper_state_callback(
        const std_msgs::msg::Float32::SharedPtr msg,
        std::shared_ptr<SerialStation> serial_station) {
    // 帧标识符
    const uint8_t GRIPPER_CLOSE_FRAME = 0x00;
    const uint8_t GRIPPER_OPEN_FRAME = 0x01;

    // 检查夹爪状态，是否需要打开或关闭夹爪
    if (msg->data == 0.0) {
        RCLCPP_INFO(rclcpp::get_logger("gripper_control"), "Received gripper close signal, closing gripper.");

        // 构造要发送的数据
        std::vector<uint8_t> data = {GRIPPER_CLOSE_FRAME};

        // 将数据存入队列并编码
        serial_station->transmit(data, GRIPPER_STATE);
    } else if (msg->data == 1.0) {
        RCLCPP_INFO(rclcpp::get_logger("gripper_control"), "Received gripper open signal, opening gripper.");

        // 构造要发送的数据
        std::vector<uint8_t> data = {GRIPPER_OPEN_FRAME};

        // 将数据存入队列并编码
        serial_station->transmit(data, GRIPPER_STATE);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("gripper_control"), "Received invalid value: %f", msg->data);
    }
}

//void base_state_callback(
//    const geometry_msgs::msg::Pose2D::SharedPtr msg,
//    std::shared_ptr<SerialStation> serial_station) {
//
//    std::vector<float> base_data = {msg->x, msg->y, msg->theta};
//    std::vector<uint8_t> data;
//
//    for (float value : base_data) {
//        uint32_t as_int = *reinterpret_cast<uint32_t*>(&value);
//        data.push_back(static_cast<uint8_t>(as_int & 0xFF));
//        data.push_back(static_cast<uint8_t>((as_int >> 8) & 0xFF));
//        data.push_back(static_cast<uint8_t>((as_int >> 16) & 0xFF));
//        data.push_back(static_cast<uint8_t>((as_int >> 24) & 0xFF));
//    }
//
//    RCLCPP_INFO(rclcpp::get_logger("base_control"),
//                "Received base pose: x=%.3f y=%.3f theta=%.3f",
//                msg->x, msg->y, msg->theta);
//
//    serial_station->transmit(data, BASE_MOVE);  // 使用你定义的帧类型
//}
void base_state_callback(
    const geometry_msgs::msg::Pose2D::SharedPtr msg,
    std::shared_ptr<SerialStation> serial_station) {

    static bool is_first = true;
    static geometry_msgs::msg::Pose2D last_pose;

    if (is_first) {
        last_pose = *msg;
        is_first = false;
        RCLCPP_INFO(rclcpp::get_logger("base_control"), "First base pose received, skipping velocity computation.");
        return;  // 第一次不发送，避免突变
    }

    // 计算差值时间间隔（固定为 0.05 秒）
    constexpr float dt = 0.03f;

    // 计算速度
    float vx = (msg->x - last_pose.x) / dt;
    float vy = (msg->y - last_pose.y) / dt;
    float vtheta = (msg->theta - last_pose.theta) / dt;

    // 更新上一帧位置
    last_pose = *msg;

    // 打包为 float 数组
    std::vector<float> velocity_data = {vx, vy, vtheta};
    std::vector<uint8_t> data;

    // float -> uint32_t -> 4 字节发送
    for (float value : velocity_data) {
        uint32_t as_int = *reinterpret_cast<uint32_t*>(&value);
        data.push_back(static_cast<uint8_t>(as_int & 0xFF));
        data.push_back(static_cast<uint8_t>((as_int >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>((as_int >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((as_int >> 24) & 0xFF));
    }

    RCLCPP_INFO(rclcpp::get_logger("base_control"),
                "Sending base velocity: vx=%.3f, vy=%.3f, vtheta=%.3f",
                vx, vy, vtheta);

    // 发送至串口，帧标识符使用 BASE_MOVE（已在 msg_types.hpp 定义为 0x0A）
    serial_station->transmit(data, BASE_MOVE);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "Hello, World!" << std::endl;

    // 创建ROS 2节点
    auto decode_node = rclcpp::Node::make_shared("decode_node");
    // 创建发布者，发布到"finish"话题
    auto publisher = decode_node->create_publisher<std_msgs::msg::Float32>("finish", 10);
    //这里是创建一个话题来回传

    SerialConfig_t config = {
        "/dev/ttyUSB0", // serial_port
        921600, // baudrate
        serial::Timeout(  // 如果只有最长超时，每次read都会超时
                0, // inter_byte_timeout_
                0, // read_timeout_constant_
                0, // read_timeout_multiplier_
                0, // write_timeout_constant_
                0 // write_timeout_multiplier_
        ),
        serial::eightbits, // byte_size
        serial::parity_none, // parity
        serial::stopbits_one, // stopbits
        serial::flowcontrol_none, // flowcontrol
        1, // tx_handle_period, unit: ms
        1, // rx_handle_period, unit: ms
    };
    auto serial_station = std::make_shared<SerialStation>(config);

    serial_station->bindEncodeFunc(encode);
    // serial_station->bindDecodeFunc(decode);
    // 直接传递带有publisher的lambda，确保签名匹配
    serial_station->bindDecodeFunc([publisher](std::vector<uint8_t> data, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub) {
        // 这里的 decode 函数发布消息
        decode(data, publisher);  // 调用decode函数并传入publisher
    });
    // 订阅 buffer_display_trajectory_point 话题
    auto sub = serial_station->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/buffered_display_trajectory_point", 10,
            [serial_station](const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
                trajectory_manipulator_callback(msg, serial_station);
            }
    );

    // 订阅 camera_correction_data 话题
    auto sub_camera_correction = serial_station->create_subscription<example_interfaces::msg::Float32MultiArray>(
            "/camera_correction_data", 10,
            [serial_station](const example_interfaces::msg::Float32MultiArray::SharedPtr msg) {
                camera_correction_callback(msg, serial_station);
            }
    );

    // auto sub_set_path_point = serial_station->create_subscription<example_interfaces::msg::Float32MultiArray>(
    //         "/set_path_joint", 10,
    //         [serial_station](const example_interfaces::msg::Float32MultiArray::SharedPtr msg) {
    //             set_path_joint(msg, serial_station);
    //         }
    // );

    // 底盘移动
    auto sub_set_path_point = serial_station->create_subscription<std_msgs::msg::Float32>(
            "/set_path_joint", 10,
            [serial_station](const std_msgs::msg::Float32::SharedPtr msg) {
                set_path_joint(msg, serial_station);
            }
    );

    // 订阅底盘自锁控制话题
    auto sub_gripper = serial_station->create_subscription<std_msgs::msg::Float32>(
            "/buffered_gripper_state", 10,
            [serial_station](const std_msgs::msg::Float32::SharedPtr msg) {
                gripper_state_callback(msg, serial_station);
            }
    );

    auto sub_base_move = serial_station->create_subscription<geometry_msgs::msg::Pose2D>(
        "/buffered_base_state", 10,
        [serial_station](const geometry_msgs::msg::Pose2D::SharedPtr msg) {
            base_state_callback(msg, serial_station);
        }
    );


    // auto sub_gripper = serial_station->create_subscription<std_msgs::msg::UInt8>(
    //         "/buffered_gripper_state", 10,
    //         [serial_station](const std_msgs::msg::UInt8::SharedPtr msg) {
    //             gripper_state_callback(msg, serial_station);
    //         }
    // );
    rclcpp::spin(serial_station);

    rclcpp::shutdown();

    return 0;
}
