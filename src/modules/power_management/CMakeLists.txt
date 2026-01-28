cmake_minimum_required(VERSION 3.8)
project(power_management)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(isaac_utils REQUIRED)
find_package(custom_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  power_management/power_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME power_manager_node
)

install(PROGRAMS
  power_management/gpio_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gpio_controller_node
)

install(PROGRAMS
  power_management/battery_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME battery_monitor_node
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
