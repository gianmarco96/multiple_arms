################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_multi_robot)


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)




################################################################################
# Install
################################################################################
install(DIRECTORY launch models rviz urdf worlds params nodes
  DESTINATION share/${PROJECT_NAME}/
)

# Install node
set(NODES_DIR nodes)
install(PROGRAMS
    ${NODES_DIR}/initial_pose_publisher.py
    DESTINATION lib/${PROJECT_NAME}
) 

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

ament_package()
