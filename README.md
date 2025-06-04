# UOW Mechatronics Engineering Projects

This repository showcases a comprehensive collection of engineering projects developed as part of the Mechatronics Engineering curriculum at the University of Wollongong. The projects span from MATLAB/Simulink-based control systems to advanced ROS-based autonomous robotics, reflecting practical applications of theoretical knowledge in automation, vibration analysis, control system design, and intelligent autonomous systems.

---

## Projects Overview

### 1. **TurtleBot3 Urban Search and Rescue System with Computer Vision**
**Duration:** Apr 2025 – May 2025  
**Course:** ECTE477/ECT8477 Programming Autonomous Systems  
**Objective:** Develop an autonomous TurtleBot3 system for urban search and rescue operations using SLAM, computer vision, and beacon detection.

This advanced robotics project implements a complete autonomous navigation and search system using ROS2. The TurtleBot3 robot autonomously explores an unknown maze environment while simultaneously building a map (SLAM), detecting colored beacons representing "victims," and publishing real-time location data for rescue coordination.

**Key Features:**
- **Autonomous Maze Exploration:** Unknown environment navigation using laser scanner
- **Simultaneous Localization and Mapping (SLAM):** Real-time map building during exploration
- **Computer Vision Beacon Detection:** RGB-D camera processing to identify colored beacon pairs
- **Custom ROS2 Message Types:** Structured data communication for beacon information
- **Multi-Topic Publishing:** Real-time data streams for map, path, and beacon locations
- **Marker Visualization:** RViz integration for visual feedback and monitoring

**Technical Implementation:**
- **SLAM Navigation:** Autonomous exploration with obstacle avoidance
- **Image Processing:** HSV color space filtering for beacon detection (4 unique color combinations)
- **Coordinate Transformation:** Camera frame to map frame coordinate conversion
- **Custom Message Protocol:** Header timestamps, sequence numbering, and position data
- **Launch File Architecture:** Automated system startup and parameter loading

**ROS2 Topics Published:**
- `/ecte477/map` - Real-time SLAM-generated map
- `/ecte477/path` - Robot trajectory tracking
- `/ecte477/beacons` - Custom beacon detection messages
- `/ecte477/beacon_markers` - RViz marker visualization

**Skills Demonstrated:** ROS2, Python, Computer Vision (OpenCV), SLAM, Autonomous Navigation, Custom Message Types, Launch Files, Parameter Loading, Coordinate Transformations

**View Code:** [RosProjects/TurtleBot Slam,Nav and Cv/](./RosProjects/TurtleBot%20Slam,Nav%20and%20Cv/)

---

### 2. **Design and Simulation of a Robotic Manipulator for Automated Assembly**
**Duration:** Aug 2023 – Oct 2023  
**Objective:** Automate nut-and-bolt operations using a 6-DOF manipulator.

This project focuses on the complete design, simulation, and control of a robotic manipulator for an automated assembly task. The system uses Denavit–Hartenberg (D-H) parameters to define joint transformations and compute both forward and inverse kinematics. The robot is programmed to pick and place M3 screws, demonstrating precision control and path planning.

**Key Features:**
- 6-DOF robot model with revolute and prismatic joints
- Forward and inverse kinematics
- Jacobian analysis for velocity control
- Path planning and sequence animation using `drawrobot3d()`
- Real-time trajectory interpolation for screw handling

**Skills Demonstrated:** MATLAB, Robotics Toolbox, Kinematics, Jacobians, Path Planning, Animation

**View Code:**
- [`Animation.m`](./Robotic%20Manipulator/Animation.m)
- [`Parameters.m`](./Robotic%20Manipulator/Parameters.m)
- [`Questions.m`](./Robotic%20Manipulator/Questions.m)

---

### 3. **Design and Optimization of a Tuned Mass Damper (TMD) System for Vehicle Suspension**
**Duration:** Sep 2023 – Oct 2023  
**Objective:** Reduce vibrations in vehicle suspension systems using passive damping.

This project simulates a two-mass spring-damper system to study and suppress unwanted oscillations in a vehicle-like structure. The primary and secondary masses are dynamically coupled, and numerical simulation via MATLAB's `ode45` reveals vibration behavior and effectiveness of the tuned mass damper.

**Key Features:**
- 2-DOF system modeled with spring-damper coupling
- External force excitation applied to the primary mass
- Numerical integration using Runge-Kutta (ODE45)
- Plots for displacement and velocity of both masses
- Parameter tuning for optimal damping response

**Skills Demonstrated:** System Modeling, Vibration Analysis, Damping Optimization, MATLAB Simulation

**View Code:** [`TunedMassDamper/TunedMassDamper.m`](./TunedMassDamper/TunedMassDamper.m)

---

### 4. **Vibration Control and Optimization of a Flatbed Trolley Using PID Control**
**Duration:** Apr 2024 – May 2024  
**Objective:** Design a digital PID controller for a flatbed trolley to minimize vibration.

This project implements an advanced control system using tuned PID gains to reduce vibration in a flatbed trolley. Starting with a mass-spring-damper model, the system is represented by a transfer function. A continuous PID controller is tuned, analyzed using Bode plots and root locus, and then discretized using Zero-Order Hold (ZOH) for digital control. Performance metrics like overshoot, settling time, and stability are evaluated.

**Key Features:**
- Transfer function representation of physical system
- Tuned PID controller (Kp, Ki, Kd)
- Conversion to discrete-time control system
- Sampling rate derived from system bandwidth
- Stability check (pole analysis inside unit circle)
- Step response and performance metric plots

**Skills Demonstrated:** Control Systems, PID Tuning, System Discretization, MATLAB & Signal Processing

**View Code:** [`Flatbed Trolley Spring Damper Feedback Control System/machinery.m`](./Flatbed%20Trolley%20Spring%20Damper%20Feedback%20Control%20System/machinery.m)

---

## Tools & Technologies Used

### Robotics & Autonomous Systems
- **ROS2 (Robot Operating System)** - Foxy/Humble distributions
- **Python 3.8+** - Core programming language for ROS nodes
- **OpenCV** - Computer vision and image processing
- **NumPy** - Numerical computations and array operations
- **TurtleBot3** - Physical robot platform
- **Gazebo** - 3D robot simulation environment
- **RViz2** - Visualization and debugging tool
- **SLAM Toolbox** - Simultaneous localization and mapping
- **Nav2** - Navigation framework for autonomous robots

### Control Systems & Simulation
- **MATLAB R2020a+**
- Simulink
- Symbolic Math Toolbox
- Control System Toolbox
- Robotics Toolbox (RTB)
- Numerical ODE Solvers
- Frequency Analysis (Bode, Pole-Zero Maps)

---

## Quick Start Guide

### ROS TurtleBot Project
```bash
# Prerequisites: ROS2 Humble, TurtleBot3 packages, OpenCV

# 1. Clone the repository
git clone https://github.com/manavsikkas/Projects.git
cd Projects/RosProjects

# 2. Build the workspace
colcon build
source install/setup.bash

# 3. Launch the simulation environment
ros2 launch practice_mazes maze_24_beacon_world.launch.xml

# 4. Start the autonomous search and rescue system
ros2 launch demo2 start_all.launch.xml

# 5. Monitor beacon detection (in separate terminal)
ros2 topic echo /ecte477/beacons
```

### MATLAB Projects
```bash
# 1. Clone the repository
git clone https://github.com/manavsikkas/Projects.git
cd Projects

# 2. Open MATLAB and navigate to desired project folder
# 3. Run the main script (e.g., Animation.m, TunedMassDamper.m, machinery.m)
```

---

## Project Achievements

- 🤖 **Autonomous Navigation:** Successfully implemented SLAM-based exploration in unknown environments
- 🎯 **Computer Vision:** Achieved reliable beacon detection using color-based image processing
- 📡 **ROS Integration:** Developed custom message types and multi-topic communication system
- 🎮 **Control Systems:** Designed and optimized PID controllers for vibration suppression
- 🦾 **Robotics:** Forward/inverse kinematics implementation for 6-DOF manipulator
- 📊 **Simulation:** Advanced MATLAB modeling for dynamic systems analysis

---

## Academic Performance

- **ECTE477 Programming Autonomous Systems:** Demonstration 2 - **90% (9.0/10.0)**
  - Requirement 1 (SLAM & Navigation): 2.5/3.0
  - Requirement 2 (Computer Vision): 6.0/6.0
  - Requirement 3 (Marker Visualization): 2.5/3.0
  - Requirement 4 (Custom Messages): 2.5/3.0

---

## Author

**Manav Sikka**  
Mechatronics Engineering Student  
University of Wollongong  
📧 [Contact](mailto:ms309@uowmail.edu.au)  
🔗 [GitHub](https://github.com/manavsikkas)  
🌐 [LinkedIn](https://linkedin.com/in/manav-sikka)

---

## Acknowledgments

- **University of Wollongong** - Faculty of Engineering and Information Sciences
- **ROS Community** - Open-source robotics framework and extensive documentation
- **TurtleBot3** - Robotis for the excellent robot platform
- **OpenCV Community** - Computer vision library and algorithms
