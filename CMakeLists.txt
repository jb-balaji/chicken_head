cmake_minimum_required(VERSION 3.8)
project(chicken_head)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Library
add_library(chicken_head src/chicken_head.cpp)
ament_target_dependencies(chicken_head
  rclcpp
  champ_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
  tf2
)

# Executable
add_executable(chicken_head_node src/chicken_head_node.cpp)
ament_target_dependencies(chicken_head_node
  rclcpp
  champ_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
  tf2
)
target_link_libraries(chicken_head_node chicken_head)

# Install targets
install(TARGETS 
    chicken_head
    chicken_head_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

