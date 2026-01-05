cmake_minimum_required(VERSION 3.8)
project(isaac_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Create entry point scripts for Python executables
install(CODE "
  file(MAKE_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}\")
  file(WRITE \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/isaac_graph_manager\"
    \"#!/usr/bin/env python3\\n\"
    \"from isaac_robot.graph_manager import main\\n\"
    \"if __name__ == '__main__':\\n\"
    \"    main()\\n\"
  )
  file(CHMOD \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/isaac_graph_manager\"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
  )
")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
