# 4-Wheeled Robot with 3-DOF Manipulator Controlled by Teleoperation

## Description
A 4-wheeled robot with a 3-DOF manipulator controlled by teleoperation.

---

## Necessary Commands

### Build the Workspace
```bash
colcon build
```

---

### Source the Environment

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/.bashrc
```

---

## Launch Files

### Launch the Robot
```bash
ros2 launch object_spawner robot.launch.py
```

---

### Launch Wheel Teleoperation
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

---

### Launch Arm Teleoperation
```bash
ros2 run custom_teleop teleop_arm_node
```

### to test attach function
```bash

ros2 service call /gazebo/attach boeing_gazebo_model_attachment_plugin_msgs/srv/Attach "{joint_name: 'right_gripper_cylinder_joint', model_name_1: 'robot', link_name_1: 'right_gripper', model_name_2: 'cylinder', link_name_2: 'cylinder'}"

```
### to test detach function
```bash

ros2 service call /gazebo/detach boeing_gazebo_model_attachment_plugin_msgs/srv/Detach "{joint_name: 'right_gripper_cylinder_joint', model_name_1: 'robot', model_name_2: 'cylinder'}"


```
