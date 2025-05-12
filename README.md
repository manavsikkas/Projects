# UOW Mechatronics Engineering Projects

This repository showcases a series of MATLAB and Simulink-based engineering projects developed as part of the Mechatronics Engineering curriculum at the University of Wollongong. Each project reflects the practical application of theoretical knowledge in automation, vibration analysis, and control system design.

---

## Projects Overview

### 1. **Design and Simulation of a Robotic Manipulator for Automated Assembly**
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

### 2. **Design and Optimization of a Tuned Mass Damper (TMD) System for Vehicle Suspension**
**Duration:** Sep 2023 – Oct 2023  
**Objective:** Reduce vibrations in vehicle suspension systems using passive damping.

This project simulates a two-mass spring-damper system to study and suppress unwanted oscillations in a vehicle-like structure. The primary and secondary masses are dynamically coupled, and numerical simulation via MATLAB’s `ode45` reveals vibration behavior and effectiveness of the tuned mass damper.

**Key Features:**
- 2-DOF system modeled with spring-damper coupling
- External force excitation applied to the primary mass
- Numerical integration using Runge-Kutta (ODE45)
- Plots for displacement and velocity of both masses
- Parameter tuning for optimal damping response

**Skills Demonstrated:** System Modeling, Vibration Analysis, Damping Optimization, MATLAB Simulation

**View Code:** [`TunedMassDamper.m`](./TunedMassDamper.m)

---

### 3. **Vibration Control and Optimization of a Flatbed Trolley Using PID Control**
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

**View Code:** [`machinery.m`](./machinery.m)

---

## Tools & Technologies Used

- **MATLAB R2020a+**
- Simulink
- Symbolic Math Toolbox
- Control System Toolbox
- Robotics Toolbox (RTB)
- Numerical ODE Solvers
- Frequency Analysis (Bode, Pole-Zero Maps)

---

## How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/manavsikkas/UOW_Projects_Mechatronics-Engineering.git
   cd UOW_Projects_Mechatronics-Engineering
