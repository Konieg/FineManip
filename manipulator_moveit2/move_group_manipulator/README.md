## 编译
```bash
colcon build --mixin release --packages-select move_group_manipulator
source install/setup.bash
```

## 启动规划组
```bash
ros2 launch move_group_manipulator move_group.launch.py
ros2 launch move_group_manipulator demo.launch.py
```

## 基本节点
### 缓存轨迹点分批次发送fines_serial，发送下位机执行
```bash
ros2 run move_group_manipulator trajectory_buffer_node
```
### 接收目标位姿进行规划
```bash
ros2 launch move_group_manipulator arm_control.launch.py
```

### 基于OMPL的抓取规划
```bash
ros2 launch move_group_manipulator pick_cylinder.launch.py
```
sa
### 基于CHOMP的抓取规划
```bash
ros2 launch move_group_manipulator pick_cylinder_chomp.launch.py
```

### 测试时发布pose
```
ros2 topic pub /arm_pose geometry_msgs/msg/Pose "{position: {x: 0.25, y: 0.25, z: 0.30}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}" --once
```

### 串口开启
```
ros2 run fines_serial fines_serial
```