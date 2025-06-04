# Project 2: Intelligent Robot Control System

This project implements an intelligent robot control system using fuzzy logic and ANFIS (Adaptive Network-based Fuzzy Inference System) for autonomous navigation and obstacle avoidance.

## Overview

The project focuses on developing a robust control system for a mobile robot that can navigate through environments with obstacles. The system employs fuzzy logic controllers and ANFIS models to make intelligent decisions based on sensor inputs.

## Files Description

### Main Control Files
- `robo_control.fis` - Main fuzzy logic controller for robot navigation
- `robo_sugeno.fis` - Sugeno-type fuzzy controller
- `Robot_control.fis` - Enhanced robot control system
- `robo_control_improved.fis` - Improved version of the control system
- `robo_control_test.fis` - Test version of the controller

### ANFIS Models
- `anfis_l.fis` - ANFIS model for left direction control
- `anfis_r.fis` - ANFIS model for right direction control
- `anfis_l_train.fis` - Trained ANFIS model for left control
- `anfis_r_train.fis` - Trained ANFIS model for right control
- `robo_sugeno_l.fis` - Left Sugeno controller
- `robo_sugeno_r.fis` - Right Sugeno controller

### Simulink Models
- `robo.slx` - Main robot simulation model
- `RoboBlock.slx` - Robot block simulation
- `RoboBlock.slx.r2016b` - Backward compatible version

### Data Files
- `obstacle_data.p` - Obstacle detection data
- `obstacle_data5.p` - Additional obstacle data
- `final_IO.mat` - Input/output training data
- `final_IO_l.mat` - Left controller I/O data
- `final_IO_r.mat` - Right controller I/O data
- `aggregated_anfis_training_data_final.mat` - ANFIS training dataset
- `sim_run_*.mat` - Various simulation run data files

### Documentation
- `Project-II.pdf` - Project documentation and requirements
- `anfis_l_2000.pdf` - ANFIS left model documentation
- `anfis_r_2000.pdf` - ANFIS right model documentation
- `Screenshot *.png` - System screenshots

### Support Files
- `initRobot.p` - Robot initialization
- `roboanimate_org.p` - Robot animation functions

## Features

- **Fuzzy Logic Control**: Implements Mamdani and Sugeno fuzzy controllers
- **ANFIS Integration**: Uses adaptive neural fuzzy systems for learning
- **Obstacle Avoidance**: Intelligent obstacle detection and avoidance
- **Multiple Controllers**: Various controller implementations for comparison
- **Simulation Support**: Complete Simulink simulation environment

## Requirements

- MATLAB with Fuzzy Logic Toolbox
- Simulink
- Neural Network Toolbox (for ANFIS)

## Usage

1. Open MATLAB and navigate to the project directory
2. Run the main Simulink model `robo.slx`
3. Configure the desired fuzzy controller
4. Run simulation to observe robot behavior

## Author

Manav Sikka

## License

This project is for educational purposes. 