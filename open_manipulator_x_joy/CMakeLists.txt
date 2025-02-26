cmake_minimum_required(VERSION 3.11)
project(open_manipulator_x_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast -g)
endif()

find_package(backward_ros REQUIRED)
set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(joy_servo SHARED 
  src/joy_servo_node.cpp
  src/joy_control.cpp
  src/manipulation_controller.cpp)
ament_target_dependencies(joy_servo ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(joy_servo
  PLUGIN "open_manipulator_x_joy::JoyServoNode"
  EXECUTABLE joy_servo_node
)

add_executable(demo_twist src/demo_twist.cpp)
ament_target_dependencies(demo_twist ${PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)
install(TARGETS demo_twist joy_servo DESTINATION lib/${PROJECT_NAME})

ament_package()