# 4-Wheeled 2-DoF Manipulator Robot – ROS 2 Workspace

## Overview

This repository contains the ROS 2 workspace (`ros2_ws`) and mechanical design files for a 4-wheeled differential drive robot with a 2-DoF manipulator arm. The robot is designed for industrial automation tasks such as pick-and-place and sorting, and is fully simulated in Gazebo with teleoperation and manipulation capabilities. The ROS 2 packages enable differential drive control, arm and gripper actuation, object attachment/detachment, and keyboard-based teleoperation.

---

## ROS 2 Node and Topic Architecture

The following diagram illustrates the main ROS 2 nodes and topics used in the simulation and control of the robot:

![ROS 2 Node Graph](https://ppl-ai-code-interpreter-files.s3.amazonaws.com/web/direct-files/15bd12ea6598bb5278faf53644113a30/b8b20a4d-7339-4074-94ac-bee14d6ca665/b6505559.png)

---

## Key Components

- **Differential Drive Base:** Controlled via `/diff_drive_controller` and `/diff_drive_controller/cmd_vel_unstamped`.
- **Manipulator Arm:**  
  - Shoulder joint: `/shoulder_joint_position_controller`
  - Elbow joint: `/elbow_joint_position_controller`
- **Grippers:**  
  - Right gripper: `/right_gripper_position_controller`
  - Left gripper: `/left_gripper_position_controller`
- **Teleoperation:**  
  - Base: `/teleop_twist_keyboard` node
  - Arm and grippers: `/teleop_robot` node
- **Attachment Plugin:**  
  - `/robot/link_attacher_node/boeing_gazebo_model_attachment_plugin` for attaching/detaching objects in Gazebo.
- **State Publishing:**  
  - `/joint_state_broadcaster` and `/robot_state_publisher` for joint and robot states.

---

## Repository Structure

- **manbot/**  
  CAD models and assembly files for the robot (mechanical design)
- **ros2_ws/**  
  ROS 2 workspace with source code, launch files, and simulation packages

---

## Quick Start

### 1. Build the Workspace
cd ros2_ws
colcon build

### 2. Source the Environmentsource /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash



### 3. Launch the Robot in Gazebo


ros2 launch object_spawner robot.launch.py


### 4. Teleoperate the Robot

#### Wheel Teleoperation

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped


#### Arm and Gripper Teleoperation

ros2 run custom_teleop teleop_arm_node



---

## Contributors

- Adithya Pothula (robot design, simulation, and ROS2 integration)
- Medhini V
- Hamna Hakkim
- Dr. Rajesh Kannan Megalingam (faculty advisor)

---

## License

Apache-2.0

---

*This workspace enables full simulation and teleoperation of the 4-wheeled 2-DoF manipulator robot in ROS 2 and Gazebo, supporting further research and development in industrial robotics and automation.*




