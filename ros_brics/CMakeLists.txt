cmake_minimum_required(VERSION 3.8)
project(ros_brics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Eigen3
#   interactive_markers
#   moveit_core
#   moveit_msgs
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   moveit_servo
#   moveit_task_constructor_core
#   moveit_visual_tools
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(nlohmann_json REQUIRED)

#C++ dependencies
include_directories(include)

#Create cpp exe
add_executable(move_program src/move_program.cpp)
ament_target_dependencies(move_program rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)

#Install cpp exe
install(TARGETS move_program DESTINATION lib/${PROJECT_NAME})

#install python modules
ament_python_install_package(${PROJECT_NAME})

# install py execs
install(PROGRAMS
  scripts/joint_transform_printer.py
  scripts/point_recording.py
  scripts/tf_listener.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
