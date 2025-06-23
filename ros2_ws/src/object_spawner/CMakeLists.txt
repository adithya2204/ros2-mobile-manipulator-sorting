cmake_minimum_required(VERSION 3.5)
project(object_spawner)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)


# Include directories for Gazebo models
include_directories(include)

# Install directories (launch, urdf, meshes, config, worlds)
install(DIRECTORY launch urdf meshes config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install other files if necessary (example: other resources or scripts)
# install(DIRECTORY scripts
#   DESTINATION share/${PROJECT_NAME}/scripts
#)

ament_package()
