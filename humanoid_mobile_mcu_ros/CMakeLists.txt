cmake_minimum_required(VERSION 3.8)
project(humanoid_mobile_mcu_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate custom ROS 2 message interfaces
set(msg_files
  "msg/RobotTelemetry.msg"
  "msg/BmsTelemetry.msg"
  "msg/PowerCmd.msg"
  "msg/LedCmd.msg"
  "msg/SignalCmd.msg"
  "msg/BmsCmd.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
)

# Install Python package module (alongside generated msg module)
install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install executable Python scripts
install(PROGRAMS
  scripts/mcu_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_protocol test/test_protocol.py)
  ament_add_pytest_test(test_mcu_bridge_node test/test_mcu_bridge_node.py TIMEOUT 60)
endif()

ament_package()
