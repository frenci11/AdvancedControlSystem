# Advanced Control System

These MATLAB scripts compute the following:

- Forward kinematics
- Inverse kinematics
- Dynamic Lagrangian
- Dynamic Euler
- Equation of motion matrices for Simulink
- Environment visualization

## Execution Instructions

1. Open `main.m` to execute the script.

## Robot Parameters

- Open `robot_properties.m` to adjust:
  - Joint position, velocity, and acceleration.
  - Other robot parameters.

## Results

- All results are computed both for the `.URDF` robot using the Robotic Toolbox functions and for the manually built robot.
- Results are used for comparison. Parameters like joint values, masses, and gravity are kept the same, except for friction (not supported by the Robotic Toolbox).

