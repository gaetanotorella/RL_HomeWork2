# Trajectory Generation and Control in Operational Space

This project concerns the planning and execution of end-effector trajectories for a 7-DOF robotic manipulator within a ROS and Gazebo simulation environment. The control framework is based on the Kinematics and Dynamics Library (KDL), which provides forward/inverse kinematics, Jacobians, and dynamic model computation. The work extends an existing ROS control package designed for manipulators.

---

## Overview

The objective is to guide the end-effector of the robot along predefined spatial paths while regulating motion dynamics. Two types of trajectories were considered:

- **Linear paths**, defined between two distinct Cartesian poses.
- **Circular paths**, defined in the vertical plane passing through the initial end-effector position.

For both trajectory types, the evolution along the path is parameterized using a **curvilinear progress variable** ranging between start and end states.

---

## Motion Profiles

Two different time laws were implemented to govern how the progress variable evolves:

1. **Trapezoidal velocity profile**, ensuring bounded acceleration and velocity during motion.
2. **Cubic polynomial profile**, enforcing zero initial and final velocity and acceleration.

These profiles influence smoothness, dynamic demand, and control effort during movement.

---

## Control Framework

Trajectory execution is handled through an **inverse dynamics controller**. The controller uses:

- The robot’s **inertia matrix**, **Coriolis contributions**, and **gravity terms**.
- The **geometric Jacobian** to map quantities between joint and task spaces.

Two control strategies were applied:

| Approach | Control Domain | Error Feedback | Use Case |
|---------|----------------|---------------|----------|
| Joint-space inverse dynamics | Joint coordinates | Difference in joint states | Tracking predefined trajectories in configuration space |
| Operational-space inverse dynamics | Cartesian coordinates | Difference in end-effector pose and twist | Direct control of the end-effector in task space |

In the operational-space formulation, the Cartesian stiffness and damping behavior is regulated directly, while internal joint dynamics are handled through projectors that preserve redundancy.

---

## Simulation and Evaluation

- All tests were performed in **Gazebo**, using ROS controllers interfaced through the standard control manager.
- End-effector motion was visualized to validate geometric correctness of linear and circular paths.
- Joint torques were recorded by streaming effort commands and logging them into a bag file.
- Logged data were analyzed in **MATLAB** to assess smoothness and transient response.
- Controller gains were adjusted based on these plots to obtain stable and non-oscillatory behavior.

---

## Software Dependencies

- **ROS** (with real-time controller manager)
- **Gazebo**
- **KDL** (Kinematics and Dynamics Library)
- Standard ROS packages for the **KUKA iiwa** manipulator
- **MATLAB** (optional, for post-processing of recorded torque data)

---

## Repository Structure

```
src/
├─ Trajectory planning module (path and profile generation)
├─ Control module (inverse dynamics in joint and operational space)
└─ Test node (execution of trajectories in Gazebo)
```

---

## Demo Video

### Circular Trajectory - Cubic Polynomial velocity profile
https://github.com/user-attachments/assets/59dfebfd-0aaa-4f82-851d-0da1914fb5a5


### Linear Trajectory - Trapezoidal velocity profile 
https://github.com/user-attachments/assets/357e4869-1052-4ffb-94cd-adadaf1f48d7


