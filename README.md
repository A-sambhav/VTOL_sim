# JETRAY VTOL Simulation Framework

## Overview

JETRAY is a ROS 2 Humble based VTOL (Vertical Take-Off and Landing) UAV simulation framework designed to emulate a complete autonomous flight stack.

The project models the entire chain from mission planning to state estimation:

```text
Mission Planner
       ↓
Flight Controller
       ↓
Aircraft Dynamics
       ↓
Sensor Simulation
       ↓
State Estimator
       ↓
Visualization
```

The architecture is intentionally designed so that simulated sensors can later be replaced by real hardware sensors without changing the higher-level software.

### Target Hardware

* STM32F401CCU6 Flight Controller
* Raspberry Pi Zero 2 W Companion Computer
* MPU6500 IMU
* MS5611 Barometer
* QMC5883L Magnetometer
* NEO-6M / NEO-7M GPS

---

# Repository Structure

```text
src/

├── sensor_simulator
├── state_estimator
├── vtol_controller
├── vtol_description
├── vtol_dynamics
├── vtol_planner
├── vtol_plant
├── vtol_testbench
```

---

# System Architecture

```text
Planner
   ↓
Controller
   ↓
VTOL Plant
   ↓
Sensor Simulator
   ↓
State Estimator
   ↓
Estimated State
```

Visualization Layer:

```text
Trajectory Visualizer
State Visualizer
Estimated State Visualizer
RViz
```

---

# Aircraft State Vector

The aircraft state vector contains twelve states:

```math
X = [u,v,w,p,q,r,x,y,z,\phi,\theta,\psi]^T
```

## Body Frame States

| State | Description       |
| ----- | ----------------- |
| u     | Forward velocity  |
| v     | Lateral velocity  |
| w     | Vertical velocity |
| p     | Roll rate         |
| q     | Pitch rate        |
| r     | Yaw rate          |

## Inertial Frame States

| State | Description      |
| ----- | ---------------- |
| x     | Position along X |
| y     | Position along Y |
| z     | Altitude         |
| φ     | Roll angle       |
| θ     | Pitch angle      |
| ψ     | Yaw angle        |

---

# Control Inputs

The aircraft is controlled using four virtual control inputs:

```math
U = [U_1,U_2,U_3,U_4]^T
```

| Input | Meaning      |
| ----- | ------------ |
| U₁    | Total thrust |
| U₂    | Roll moment  |
| U₃    | Pitch moment |
| U₄    | Yaw moment   |

---

# Dynamics Model

The VTOL plant simulates full nonlinear rigid-body dynamics.

## Translational Dynamics

### X-axis Dynamics

```math
\dot{u} = \frac{F_x}{m} - qw + rv - g\sin\theta
```

### Y-axis Dynamics

```math
\dot{v} = \frac{F_y}{m} - ru + pw + g\cos\theta\sin\phi
```

### Z-axis Dynamics

```math
\dot{w} = \frac{F_z}{m} - pv + qu - g\cos\theta\cos\phi
```

These equations account for:

* Gravity
* Vehicle mass
* Rotational coupling
* Thrust forces

---

# Rotational Dynamics

The rotational equations use the aircraft inertia matrix:

```text
Ixx
Iyy
Izz
Ixz
```

and include:

* Gyroscopic coupling
* Rotor angular momentum effects
* Applied control moments

---

# Numerical Integration

The aircraft state is propagated using:

```text
Runge-Kutta 4th Order (RK4)
```

Advantages:

* Higher accuracy than Euler integration
* Better stability
* Suitable for nonlinear aircraft dynamics

---

# Mission Planner

The planner generates reference trajectories.

Published Topic:

```text
/reference_states
```

Reference Vector:

```text
[
x_d
y_d
z_d

vx_d
vy_d
vz_d

ax_d
ay_d
az_d

ψ_d
]
```

---

# Transition Mission

The implemented transition mission contains:

## Phase 1 – Takeoff

```text
0–10 s
```

Vehicle climbs vertically to:

```text
10 m
```

---

## Phase 2 – Hover

```text
10–13 s
```

Vehicle maintains altitude.

---

## Phase 3 – Forward Transition

```text
13–18 s
```

Vehicle accelerates forward.

---

## Phase 4 – Cruise

```text
18–28 s
```

Constant forward flight.

---

## Phase 5 – Transition Back

Vehicle decelerates.

---

## Phase 6 – Hover

Vehicle stabilizes before landing.

---

## Phase 7 – Landing

Controlled descent to ground level.

---

# Controller

The controller package implements nonlinear feedback linearization.

## Position Error

```math
e_x = x_r - x
```

```math
e_y = y_r - y
```

```math
e_z = z_r - z
```

---

## Virtual Accelerations

```math
\ddot{x}_d = k_{1x}\dot e_x + k_{2x}e_x
```

```math
\ddot{y}_d = k_{1y}\dot e_y + k_{2y}e_y
```

```math
\ddot{z}_d = k_{1z}\dot e_z + k_{2z}e_z
```

---

## Thrust Command

```math
U_1 = \frac{m(\ddot z_d + g)}
{\cos\phi \cos\theta}
```

The controller automatically compensates for gravity and aircraft attitude.

---

# Sensor Simulation

The sensor simulation package emulates real onboard sensors.

---

## MPU6500

Outputs:

```text
Angular Velocity
Linear Acceleration
```

Includes:

* Gyro noise
* Gyro bias
* Gyro drift
* Accelerometer noise
* Accelerometer bias
* Accelerometer drift

Published Topic:

```text
/imu/data
```

---

## GPS

Published Topic:

```text
/gps/pose
```

Features:

* Position noise
* Update rate limitation
* GPS dropout simulation

Typical update rate:

```text
5 Hz
```

---

## MS5611

Published Topic:

```text
/barometer
```

Provides:

```text
Altitude
```

Includes:

* Noise
* Drift
* Bias

---

## QMC5883L

Published Topic:

```text
/magnetometer
```

Provides:

```text
Heading
```

Includes:

* Magnetic noise
* Magnetic disturbances

---

# State Estimator

The estimator consumes:

```text
/imu/data
/gps/pose
/barometer
/magnetometer
```

and produces:

```text
/estimated_state
```

Goal:

```text
estimated_state ≈ vtol_states
```

The estimator reconstructs:

* Position
* Velocity
* Attitude
* Heading

from noisy sensor measurements.

---

# Visualization

## Planned Trajectory

Topic:

```text
/trajectory_marker
```

Color:

```text
Red
```

---

## Actual Trajectory

Topic:

```text
/actual_trajectory_marker
```

Color:

```text
Green
```

---

## Estimated Trajectory

Topic:

```text
/estimated_trajectory_marker
```

Color:

```text
Blue
```

---

# ROS Topics

## Planner

```text
/reference_states
```

## Controller

```text
/control_inputs
```

## Plant

```text
/vtol_states
/vtol_odom
```

## Sensors

```text
/imu/data
/gps/pose
/barometer
/magnetometer
```

## Estimator

```text
/estimated_state
```

## Visualization

```text
/trajectory_marker
/actual_trajectory_marker
/estimated_trajectory_marker
```

---

# Launching the Simulation

## Build

```bash
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch vtol_planner vtol_sim.launch.py
```

---

# Future Work

* Extended Kalman Filter (EKF)
* Unscented Kalman Filter (UKF)
* LQR Flight Controller
* Hardware-In-The-Loop (HIL)
* Wind Disturbance Models
* Return-To-Home
* Fault Detection
* Autonomous Mission Planning
* Real Flight Testing

---

# Author

**SAMBHAV AGARWAL**

JETRAY VTOL Project

ROS 2 Humble • Python • Guidance, Navigation and Control • UAV Systems

