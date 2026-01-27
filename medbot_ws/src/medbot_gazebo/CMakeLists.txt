cmake_minimum_required(VERSION 3.8)
project(medbot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install directories
install(DIRECTORY
  worlds
  models
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Set Gazebo model path
set(GAZEBO_MODEL_PATH ${GAZEBO_MODEL_PATH}:${CMAKE_CURRENT_SOURCE_DIR}/models)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
