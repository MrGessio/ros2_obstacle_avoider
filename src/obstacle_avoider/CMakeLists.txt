cmake_minimum_required(VERSION 3.8)
project(obstacle_avoider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# add executable
add_executable(obstacle_avoider_node src/obstacle_avoider.cpp)
ament_target_dependencies(obstacle_avoider_node rclcpp sensor_msgs geometry_msgs)

# install executable
install(TARGETS
  obstacle_avoider_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

