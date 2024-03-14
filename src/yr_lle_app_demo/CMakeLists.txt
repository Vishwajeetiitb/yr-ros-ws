cmake_minimum_required(VERSION 3.8)
project(yr_lle_app_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yr_lle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include C++ directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${yr_lle_msgs_INCLUDE_DIRS}
)

add_compile_options(-Wno-reorder -Wno-unused-variable
  -Wno-vla -Wno-pedantic -Wno-missing-field-initializers
  -Wno-unused-but-set-variable -Wno-unused-parameter
)

# Declare a C++ executable
add_executable(app_demo_node src/app_demo_node.cpp include/yr_lle_app_demo/message_statistics.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(app_demo_node rclcpp yr_lle_msgs sensor_msgs)

# Install the node executable
install(TARGETS app_demo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
