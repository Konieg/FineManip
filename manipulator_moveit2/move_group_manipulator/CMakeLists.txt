cmake_minimum_required(VERSION 3.22)
project(move_group_manipulator)    


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)


add_executable(move_group_interface_manipulator
    src/move_group_interface_manipulator.cpp)

add_executable(pick_cylinder_demo
    src/move_group_interface.cpp)

add_executable(pick_cylinder_chomp_demo
    src/move_group_interface_chomp.cpp)

add_executable(pick_cylinder_ompl_chomp_demo
    src/move_group_interface_ompl_chomp.cpp)

add_executable(trajectory_buffer_node
    src/trajectory_buffer.cpp)

add_executable(pick_and_place_node
    src/arm_control.cpp
    src/Communication.cpp)


target_include_directories(move_group_interface_manipulator
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    
target_include_directories(pick_cylinder_demo
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(pick_cylinder_ompl_chomp_demo
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)   
    
target_include_directories(pick_cylinder_chomp_demo
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(pick_and_place_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 本地头文件
    $<INSTALL_INTERFACE:include>)


target_compile_features(move_group_interface_manipulator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(pick_cylinder_demo PUBLIC c_std_99 cxx_std_17)
target_compile_features(pick_cylinder_chomp_demo PUBLIC c_std_99 cxx_std_17)    
target_compile_features(pick_cylinder_ompl_chomp_demo PUBLIC c_std_99 cxx_std_17)    
target_compile_features(pick_and_place_node PUBLIC c_std_99 cxx_std_17)  

ament_target_dependencies(move_group_interface_manipulator
    rclcpp
    moveit_ros_planning_interface
    graph_msgs
    moveit_visual_tools
    rviz_visual_tools
)

ament_target_dependencies(pick_cylinder_demo
    rclcpp
    moveit_ros_planning_interface
    graph_msgs
    moveit_visual_tools
    rviz_visual_tools
)

ament_target_dependencies(pick_cylinder_chomp_demo
    rclcpp
    moveit_ros_planning_interface
    graph_msgs
    moveit_visual_tools
    rviz_visual_tools
)

ament_target_dependencies(pick_cylinder_ompl_chomp_demo
    rclcpp
    moveit_ros_planning_interface
    graph_msgs
    moveit_visual_tools
    rviz_visual_tools
)

ament_target_dependencies(trajectory_buffer_node
    rclcpp
    moveit_msgs
    moveit_ros_planning_interface
    trajectory_msgs
)

ament_target_dependencies(pick_and_place_node
    rclcpp
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    geometry_msgs
    Eigen3
    tf2
    tf2_eigen
    tf2_ros
    moveit_visual_tools
)


install(TARGETS move_group_interface_manipulator pick_cylinder_demo pick_cylinder_chomp_demo 
    pick_cylinder_ompl_chomp_demo
    trajectory_buffer_node pick_and_place_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()