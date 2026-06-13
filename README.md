# JETRAY VTOL Simulation Framework

A complete ROS 2 Humble based VTOL UAV simulation framework developed for the JETRAY project.

The framework provides a full flight-control pipeline including trajectory planning, control, aircraft dynamics, sensor simulation, state estimation, and visualization.

---

## Features

### Flight Dynamics

* 6-DOF VTOL aircraft dynamics
* Body-frame translational states:

  * u, v, w
* Body-frame angular rates:

  * p, q, r
* Inertial-frame states:

  * x, y, z
  * φ (roll)
  * θ (pitch)
  * ψ (yaw)

---

### Trajectory Planning

Supported mission modes:

* Hover
* Transition Flight
* Circular Trajectory
* Custom trajectory generation

Planner publishes:

```text
/reference_states
```

containing:

```text
[x_d, y_d, z_d,
 vx_d, vy_d, vz_d,
 ax_d, ay_d, az_d,
 ψ_d]
```

---

### Flight Controller

Implements trajectory tracking and stabilization.

Responsibilities:

* Position tracking
* Velocity tracking
* Attitude command generation
* Transition flight support

Controller subscribes to:

```text
/reference_states
```

and aircraft states.

---

### Aircraft Dynamics Plant

Simulates the VTOL vehicle dynamics.

Publishes:

```text
/vtol_states
```

State vector:

```text
[u, v, w,
 p, q, r,
 x, y, z,
 φ, θ, ψ]
```

---

### Sensor Simulation

Realistic sensor emulation layer.

Implemented sensors:

#### MPU6500

Provides:

* Gyroscope
* Accelerometer

Includes:

* Sensor noise
* Bias
* Drift

---

#### QMC5883L

Provides:

* Magnetic heading

Includes:

* Noise
* Magnetic disturbances

---

#### MS5611

Provides:

* Altitude measurement

Includes:

* Noise
* Drift

---

#### GPS

Provides:

* Position measurement

Includes:

* Noise
* Dropout simulation
* Reduced update rate

Published topics:

```text
/imu/data
/magnetometer
/barometer
/gps/pose
```

---

### State Estimation

Sensor-fusion based state estimator.

Consumes:

```text
/imu/data
/magnetometer
/barometer
/gps/pose
```

Produces:

```text
/estimated_state
```

Estimated state vector:

```text
[u, v, w,
 p, q, r,
 x, y, z,
 φ, θ, ψ]
```

Designed so that simulated sensors can later be replaced by real hardware sensors without changing higher-level software.

---

### Visualization

RViz visualization includes:

#### Planned Trajectory

```text
/trajectory_marker
```

Red line.

---

#### Actual Aircraft Trajectory

```text
/actual_trajectory_marker
```

Green line.

---

#### Estimated Aircraft Trajectory

```text
/estimated_trajectory_marker
```

Blue line.

---

## Software Architecture

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

## ROS 2 Topics

### Planner

```text
/reference_states
```

### Plant

```text
/VTOL_states
```

### Sensors

```text
/imu/data
/gps/pose
/barometer
/magnetometer
```

### Estimator

```text
/estimated_state
```

### Visualization

```text
/trajectory_marker
/actual_trajectory_marker
/estimated_trajectory_marker
```

---

## Launch

Build workspace:

```bash
colcon build
source install/setup.bash
```

Launch complete simulation:

```bash
ros2 launch vtol_planner vtol_sim.launch.py
```

---

## Target Hardware

The simulation architecture is designed to transition directly to the JETRAY VTOL hardware platform.

Planned hardware:

### Flight Controller

* STM32F401CCU6

### Companion Computer

* Raspberry Pi Zero 2 W

### Sensors

* MPU6500
* QMC5883L
* MS5611
* NEO-6M GPS

The sensor simulation layer mirrors these real sensors to simplify deployment on physical hardware.

---

## Future Work

* Extended Kalman Filter (EKF)
* LQR Controller
* Hardware-in-the-Loop (HIL)
* GPS Return-To-Home
* Wind Disturbance Modeling
* Fault Detection
* Autonomous Mission Planning
* Real Flight Testing

---

## Author

ASAMBHAV

JETRAY VTOL Project

ROS 2 Humble | Python | UAV Guidance, Navigation and Control

