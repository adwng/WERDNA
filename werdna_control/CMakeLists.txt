cmake_minimum_required(VERSION 3.8)
project(werdna_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(werdna_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(werdna_control_node src/werdna_control.cpp)
add_executable(werdna_balance_node src/werdna_balance.cpp)

ament_target_dependencies(werdna_control_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  werdna_msgs
  trajectory_msgs
)

ament_target_dependencies(werdna_balance_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
)

install(TARGETS
  werdna_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  werdna_balance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
