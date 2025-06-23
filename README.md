# Differential Drive Manipulator Robot – huT Labs Internship Project

## Overview

This repository contains the design and assembly files for a differential drive manipulator robot developed during an internship at huT Labs, Amrita Vishwa Vidyapeetham. The robot is engineered for industrial automation tasks such as pick-and-place and item sorting, and is designed to be simulated and controlled using ROS 2 and Gazebo. The project emphasizes modularity, teleoperation, and simulation-driven validation, enabling safe and efficient testing of robotic kinematics and manipulation strategies in a virtual factory environment.

## Repository Structure

- **manbot/**  
  Contains the complete robot design and assembly files, including CAD models and URDF descriptions for simulation.

- **ros2_ws/**  
  *(To be added later)* ROS 2 workspace with simulation, control, and teleoperation packages for the robot.

---

## Project Highlights

- **Robot Design:**  
  Four-wheeled differential drive base supporting a 2-DOF manipulator arm with synchronized grippers. All components modeled in SolidWorks and exported for simulation.

- **Simulation Environment:**  
  Robot and a realistic factory floor workspace modeled in Gazebo, including ramps with varying friction coefficients to test mobility and manipulation.

- **Control System:**  
  ROS 2-based teleoperation interface for driving the base and controlling the manipulator arm. Uses diff-drive and joint position controllers, with real-time feedback and visualization.

- **Validation:**  
  Experiments conducted in simulation to assess teleoperation responsiveness, pick-and-place accuracy, and performance across different terrains.

---

## Applications

- Industrial automation: pick-and-place, sorting, and material handling
- Robotics research and education
- Simulation-driven design and validation

---

## Contributors

- Adithya Pothula (robot design, simulation, and ROS2 integration)
- Medhini V
- Hamna Hakkim
- Dr. Rajesh Kannan Megalingam (faculty advisor)

---

## License

Specify your preferred license here (e.g., MIT, GPL, etc.).

---

*Note: The ROS2 workspace (`ros2_ws`) for simulation and control will be added in a future update.*
