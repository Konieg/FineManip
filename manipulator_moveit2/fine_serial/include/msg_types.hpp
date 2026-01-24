#ifndef FINES_NODE_MSG_TYPES_HPP
#define FINES_NODE_MSG_TYPES_HPP

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

typedef enum {
    NONE =0x00,
    MOVE_CHASSIS  = 0X01,
    MOVE_MANIPULATOR = 0X0B,
    ODOMETRY_OFFSET = 0X03,
    CHASSIS_STOP = 0X04,
    SET_PATH_POINT =0X02,
    GRIPPER_STATE =0X05,
    BASE_MOVE=0X0A
} MessageType_e;


#endif //FINES_NODE_MSG_TYPES_HPP
