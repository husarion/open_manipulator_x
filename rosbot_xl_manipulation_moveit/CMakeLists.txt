cmake_minimum_required(VERSION 3.10.2)
project(rosbot_xl_manipulation_moveit)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/start_moveit_servo_node.py
  ${PROJECT_NAME}/joy_servo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()