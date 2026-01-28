cmake_minimum_required(VERSION 3.8)
project(medbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  medbot_localization/odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch files
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
