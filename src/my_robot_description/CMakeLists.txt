cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(
  urdf/robot.urdf.xacro
  OUTPUT urdf/my_robot.urdf
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

