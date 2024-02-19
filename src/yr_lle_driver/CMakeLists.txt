cmake_minimum_required(VERSION 3.8)
project(yr_lle_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yr_lle_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(Protobuf REQUIRED)

if(PROTOBUF_FOUND)
  message("Protobuf version : ${Protobuf_VERSION}")
  include_directories(${PROTOBUF_INCLUDE_DIR})
else()
  message(FATAL_ERROR "Protobuf not found")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

set(YR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../yr_sdk/src")
add_custom_target(run_proto_compile COMMAND ${CMAKE_COMMAND} -E chdir ${YR_SDK_DIR}/protobuf ./compile_protobuf.sh)

# add_compile_options(-Wno-reorder -Wno-unused-variable -Wno-vla -Wno-designated-init)
add_compile_options(-Wno-reorder -Wno-unused-variable
  -Wno-vla -Wno-pedantic -Wno-missing-field-initializers
  -Wno-unused-but-set-variable -Wno-unused-parameter
)

include_directories(
  include
  ${YR_SDK_DIR}
  ${YR_SDK_DIR}/protobuf
)

file(GLOB YR_SDK_SRC_FILES
  "${YR_SDK_DIR}/*.cpp"
  "${YR_SDK_DIR}/protobuf/*.cc"
)

# list(REMOVE_ITEM YR_SDK_SRC_FILES "${YR_SDK_DIR}/spi_test.cpp")
list(FILTER YR_SDK_SRC_FILES EXCLUDE REGEX ".*test\\.cpp$")
list(REMOVE_ITEM YR_SDK_SRC_FILES "${YR_SDK_DIR}/pb2ros_converter.cpp")

message(STATUS "YR_SDK_SRC_FILES: ${YR_SDK_SRC_FILES}")

set(NODES
  mcu_bridge_ros_node
  # periodic_tasks_node
  # spi_driver_ros_node
)

foreach(NODE ${NODES})
  add_executable(${NODE}
    src/${NODE}.cpp
    ${YR_SDK_SRC_FILES}
    ${YR_SDK_DIR}/protobuf/exo_data.pb.cc
  )
  ament_target_dependencies(${NODE} rclcpp geometry_msgs yr_lle_msgs sensor_msgs)
  add_dependencies(${NODE} run_proto_compile)
  target_link_libraries(${NODE} ${PROTOBUF_LIBRARY} "${cpp_typesupport_target}")
  install(TARGETS ${NODE} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
