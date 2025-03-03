cmake_minimum_required(VERSION 3.22)
project(open_manipulator_x_moveit)

set(PACKAGE_INCLUDE_DEPENDS
    ament_cmake moveit_core moveit_msgs moveit_ros_planning
    moveit_ros_planning_interface rclcpp)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(dock src/dock.cpp)
ament_target_dependencies(dock ${PACKAGE_INCLUDE_DEPENDS})
add_executable(ready src/ready.cpp)
ament_target_dependencies(ready ${PACKAGE_INCLUDE_DEPENDS})

install(TARGETS dock ready DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
