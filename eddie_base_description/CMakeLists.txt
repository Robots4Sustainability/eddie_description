cmake_minimum_required(VERSION 3.12)
project(eddie_base_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(xacro REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(rviz2 REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(urdf REQUIRED)

# Install the xacro and dae files
install(DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

# Add ament package macros
ament_package()