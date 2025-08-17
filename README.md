# Formula Student System Architecture

> **Note:** This repository is designed to share high‑level information about the project and does not include any source code. The code is kept private for competitive reasons.

## Background

The **Formula Student BGR** team is developing an autonomous race car as part of the Formula Student Driverless (FSD) competition. Launched in 2017, this competition challenges teams from around the world to build and program electric autonomous vehicles capable of navigating complex tracks at high speeds. The goal of this project is to develop a system architecture that can handle this dynamic environment and serve as a foundation for future developments.

## Project Objectives

- **Comprehensive architecture proposal:** Develop a software architecture that integrates perception, localization, mapping, path planning and control so the vehicle can respond to its environment in real time.
- **Algorithm research and selection:** Evaluate and choose advanced algorithms for sensor processing, object detection, mapping and path planning (e.g., YOLOv8 for cone detection, LiDAR clustering for mapping, Kalman filtering for localization, Delaunay triangulation for path planning, and Pure Pursuit and PID control for navigation).
- **Incorporate existing research:** Adapt and integrate proven techniques from literature and open‑source projects (such as Berlin Team’s path planning) to strengthen the proposed architecture.
- **Foundation for future work:** Build a modular software framework that can be expanded and improved in future projects.

## Key Components

| Component | Role |
|---|---|
| **Processing Unit** | Central computer (e.g., NVIDIA Jetson) running the ROS 2 stack and real‑time algorithms. |
| **Sensor Suite** | Combination of a 3D LiDAR, stereo cameras, precision GPS and IMU for perception, positioning and motion sensing. |
| **Actuation System** | Electric drivetrain, steering and braking systems controlled by Pure Pursuit/PID/MPC controllers to ensure safe, fast navigation. |
| **Perception Module** | Processes camera data with YOLOv8 to detect cones and obstacles and uses LiDAR clustering for environmental mapping. |
| **Localization & Mapping Module** | Fuses GPS, IMU and LiDAR data with a Kalman filter; uses kinematic models (e.g., Kinematic Bicycle Model) to estimate the car’s pose. |
| **Path Planning Module** | Creates safe trajectories using Delaunay triangulation, draws on open‑source projects and builds custom algorithms based on research. |
| **Control Module** | Implements Pure Pursuit and PID controllers (and explores MPC) for steering and speed control. |
| **Safety System** | Emergency shutdown mechanism that safely halts the vehicle in case of critical failure. |

## Overall Architecture

The project uses the **ROS 2** framework to connect all components as nodes communicating via topics and services. Machine‑learning‑based perception (YOLOv8), point‑cloud mapping algorithms, Kalman filtering for localization, Delaunay triangulation for path planning and Pure Pursuit/PID control are executed on a central processing unit. A **Docker** environment ensures reproducible development and deployment, and simulation tools allow testing algorithms before live runs.

**Autonomous system pipeline**

- **Sensors:** LiDAR, cameras, IMU and GPS collect raw data  
  ↓
- **Perception & mapping:** YOLOv8 detects cones and obstacles, while LiDAR clustering builds an environmental map  
  ↓
- **Localization:** A Kalman filter together with the Kinematic Bicycle Model estimate the vehicle’s position and orientation  
  ↓
- **Path planning:** Delaunay triangulation and custom path planners generate safe, optimal trajectories  
  ↓
- **Control:** Pure Pursuit and PID/MPC controllers translate the planned trajectory into steering and speed commands  
  ↓
- **Actuators:** Steering, throttle and braking systems execute the commands on the vehicle



## Usage Guidelines

Because the code is private, this repository serves as documentation only. Here you can find:

- System requirement documents and specifications.
- Architecture diagrams and data structures.
- Setup and run instructions for the ROS 2 and Docker environments.
- Reading materials on autonomous system design and FSD competitions.

## Contributions

Source code and development work are handled privately by the team. We welcome comments, questions and collaboration requests, but any access to the code requires approval from the project team.
