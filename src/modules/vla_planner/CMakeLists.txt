cmake_minimum_required(VERSION 3.8)
project(vla_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(isaac_utils REQUIRED)
find_package(custom_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  vla_planner/vla_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vla_controller_node
)

install(PROGRAMS
  vla_planner/action_executor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME action_executor_node
)

install(PROGRAMS
  vla_planner/planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME planner_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
