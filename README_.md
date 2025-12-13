# RFBL Controller: Robust Feedback Linearization for Quadrotor Control

![Status](https://img.shields.io/badge/Status-Active-brightgreen) ![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Python](https://img.shields.io/badge/Python-3.10+-blue)

## Group Members:

This project is completed with the help of: 

- Syed Sameeed Ahmed
- Qadeem Khan

## Overview

This repository implements a **Robust Feedback Linearization with Supertwisting (RFBL-STC)** controller for autonomous quadrotor flight control. The control architecture combines feedback linearization for nonlinear attitude and altitude regulation with a supertwisting sliding-mode controller (STC) for disturbance rejection and enhanced robustness.

The implementation is built for ROS 2 and provides a complete control stack including:
- **Altitude & Attitude Control**: Robust feedback linearization with supertwisting sliding mode
- **XY Position Control**: Outer-loop position tracking with disturbance compensation
- **Disturbance Observers**: Harmonic disturbance observers for altitude, attitude, and position channels
- **PX4/MAVROS Integration**: Direct integration with PX4 autopilot via MAVROS

---

## Technical Background

### System Dynamics

The quadrotor dynamics are modeled as follows:

**Translational dynamics:**
$$\ddot{z} = -g + \frac{F_z}{m} + d_z$$

$$\ddot{x} = \sin(\theta)\cos(\psi)\frac{F_x}{m} + \sin(\phi)\sin(\psi)\frac{F_y}{m} + d_x$$

$$\ddot{y} = \sin(\theta)\sin(\psi)\frac{F_x}{m} - \sin(\phi)\cos(\psi)\frac{F_y}{m} + d_y$$

**Rotational dynamics:**
$$\ddot{\phi} = \frac{\tau_\phi}{I_x} + d_\phi$$

$$\ddot{\theta} = \frac{\tau_\theta}{I_y} + d_\theta$$

$$\ddot{\psi} = \frac{\tau_\psi}{I_z} + d_\psi$$

where:
- $z, x, y$ are position states (vertical and horizontal)
- $\phi, \theta, \psi$ are attitude angles (roll, pitch, yaw)
- $F_z, \tau_\phi, \tau_\theta, \tau_\psi$ are control inputs
- $d_z, d_x, d_y, d_\phi, d_\theta, d_\psi$ represent disturbances and model uncertainties
- $m$ is quadrotor mass, $I_x, I_y, I_z$ are moment of inertia
- $g$ is gravitational acceleration

### Control Approach: Robust Feedback Linearization

#### Altitude & Attitude Controller

The inner-loop controller implements equations (11)-(14) from the referenced paper using feedback linearization with supertwisting sliding-mode control:

**Sliding Surfaces:**

For altitude:
$$s_z = e_2 + \lambda_z e_1$$

For attitude angles (φ, θ, ψ):
$$s_\phi = e_4 + \lambda_\phi e_3$$
$$s_\theta = e_6 + \lambda_\theta e_5$$
$$s_\psi = e_8 + \lambda_\psi e_7$$

where $e_i = x_i - x_{id}$ are tracking errors and $\lambda_i$ are positive design parameters.

**Supertwisting Control Law:**

$$u = u_{eq} - \alpha|s|^{1/2}\text{sign}(s) - \beta\int\text{sign}(s)dt$$

where:
- $u_{eq}$ is the equivalent control (linearizing part)
- $\alpha, \beta$ are supertwisting gains chosen such that $\alpha^2 > 4\beta$
- The supertwisting algorithm ensures finite-time convergence of sliding surfaces to zero

**Control Output (Feedback Linearization):**

$$F_z = m\left(z_{dd} + \lambda_z \dot{e}_1 + \alpha_z|s_z|^{1/2}\text{sign}(s_z) + \beta_z\int\text{sign}(s_z)dt - \hat{d}_z\right)$$

$$\tau_\phi = I_x\left(\phi_{dd} + \lambda_\phi \dot{e}_3 + \alpha_\phi|s_\phi|^{1/2}\text{sign}(s_\phi) + \beta_\phi\int\text{sign}(s_\phi)dt - \hat{d}_\phi\right)$$

$$\tau_\theta = I_y\left(\theta_{dd} + \lambda_\theta \dot{e}_5 + \alpha_\theta|s_\theta|^{1/2}\text{sign}(s_\theta) + \beta_\theta\int\text{sign}(s_\theta)dt - \hat{d}_\theta\right)$$

$$\tau_\psi = I_z\left(\psi_{dd} + \lambda_\psi \dot{e}_7 + \alpha_\psi|s_\psi|^{1/2}\text{sign}(s_\psi) + \beta_\psi\int\text{sign}(s_\psi)dt - \hat{d}_\psi\right)$$

The disturbance estimates $\hat{d}_i$ are computed by the harmonic disturbance observers.

#### XY Position Controller

The outer-loop controller provides reference trajectories for altitude and attitude to track desired XY positions. It implements a similar sliding-mode structure:

**Sliding Surfaces for Position:**
$$s_x = v_x + \lambda_x e_x$$
$$s_y = v_y + \lambda_y e_y$$

where $e_x = x - x_d$, $e_y = y - y_d$ are position errors.

**Desired Attitude References:**
$$\theta_d = \arcsin\left(\frac{F_x}{F_{total}}\right)$$
$$\phi_d = \arcsin\left(\frac{F_y}{F_{total}}\right)$$

where the control forces are computed via supertwisting control on the XY sliding surfaces.

### Disturbance Observer (Harmonic Model)

The harmonic disturbance observer estimates persistent/harmonic disturbances using a known disturbance frequency $\omega_{do}$:

**Observer Dynamics:**
$$\dot{z}_1 = z_2 + k_1(d_{meas} - z_1)$$
$$\dot{z}_2 = -\omega_{do}^2 z_1 + k_2(d_{meas} - z_1)$$

where:
- $z_1, z_2$ are observer states
- $d_{meas}$ is the measured disturbance (estimated from dynamics)
- $k_1, k_2$ are observer gains
- $\hat{d} = z_1$ is the estimated disturbance

The measured disturbance is computed from the state dynamics:
$$d_{meas} = \ddot{x}_{measured} - \ddot{x}_{model}$$

where $\ddot{x}_{measured}$ is obtained via numerical differentiation of velocity.

---
# Controller Architecture

```
          +--------------------------+
          |   Reference Trajectory   |
          +------------+-------------+
                       |
                       v
          +--------------------------+
          |  RFBL Controller         |
          |  (Dynamic Inversion)     |
          +------------+-------------+
                       |
                       v
          +--------------------------+
          | NHDO Disturbance Obs.    |
          +------------+-------------+
                       |
                       v
          +--------------------------+
          | PX4 Offboard Commands    |
          +--------------------------+
```

The RFBL ensures nonlinearities are canceled, and NHDO handles unknown disturbances.


# System Architecture Diagrams

Following is the sysmtem flow used to implement the control architecture:

## **High-Level Control Flow**

```
+------------------+      +----------------------+      +---------------+
| Trajectory Gen.  | ---> | RFBL Linearization   | ---> | NHDO Observer |
+------------------+      +----------------------+      +---------------+
                                    |
                                    v
                           +----------------------+
                           | PX4 Offboard Control |
                           +----------------------+
                                    |
                                    v
                               +---------+
                               |  UAV    |
                               +---------+
```


# Simulation Results

## **3D Trajectory Tracking**
![3D Trajectory](python/f_results/Figure_1_3D_Trajectory.png)

## **Position Tracking**
![Position Tracking](python/f_results/Figure_2_Position_Tracking.png)

## **Attitude Tracking**
![Attitude Tracking](python/f_results/Figure_3_Attitude_Tracking.png)

## **Control Inputs**
![Control Inputs](python/f_results/Figure_4_Control_Inputs.png)

## **Disturbance Estimation**
![Disturbance](python/f_results/Figure_5_Disturbance.png)

---

# Demo Video 

### Demo Video

[Click to watch the demo video](Demo/demo_Video_Gazebo.mp4)


## Workspace Directory Structure

Complete ROS 2 workspace layout:

```
ros2_ws/                                   # ROS 2 workspace root
├── src/                                   # Source code directory
│   ├── rfbl_controller/                   # Main package
│   │   ├── rfbl_controller/               # Python package directory
│   │   │   ├── __init__.py                # Package initialization
│   │   │   ├── rfbl_controller.py         # Inner-loop altitude & attitude controller
│   │   │   │                              # - RFBLAltitudeAttitudeController class
│   │   │   │                              # - Supertwisting sliding-mode implementation
│   │   │   │                              # - Implements equations (11)-(14) from paper
│   │   │   ├── xy_position_controller.py  # Outer-loop XY position controller
│   │   │   │                              # - XYPositionRFBLSTC class
│   │   │   │                              # - Generates attitude references
│   │   │   │                              # - Position tracking with disturbance compensation
│   │   │   ├── disturbance_observer.py    # Altitude & attitude disturbance observer
│   │   │   │                              # - HarmonicDisturbanceObserver class
│   │   │   │                              # - 2nd-order harmonic observer per axis
│   │   │   │                              # - Estimates [d_z, d_phi, d_theta, d_psi]
│   │   │   ├── xy_disturbance_observer.py # XY position disturbance observer
│   │   │   │                              # - HarmonicDisturbanceObserverXY class
│   │   │   │                              # - Estimates [d_x, d_y] disturbances
│   │   │   ├── params_refs.py             # Parameters and trajectory references
│   │   │   │                              # - QuadrotorParams dataclass
│   │   │   │                              # - Quadrotor mass, inertias, damping coefficients
│   │   │   │                              # - Supertwisting gain configurations
│   │   │   │                              # - Observer tuning parameters
│   │   │   ├── rfbl_offboard_node.py      # ROS 2 node integration (main entry point)
│   │   │   │                              # - RfblOffboardNode class
│   │   │   │                              # - MAVROS subscribers: state, pose, velocity
│   │   │   │                              # - Attitude setpoint publisher
│   │   │   │                              # - 50 Hz control loop
│   │   │   └── px4_rviz_bridge.py         # PX4-RViz visualization bridge
│   │   │                                  # - Real-time telemetry visualization
│   │   ├── package.xml                    # ROS 2 package metadata
│   │   │                                  # - Package name, version, dependencies
│   │   │                                  # - Maintainer information
│   │   ├── setup.py                       # Python setuptools configuration
│   │   │                                  # - Package installation settings
│   │   │                                  # - Entry point: rfbl_offboard_node
│   │   ├── setup.cfg                      # Setup configuration
│   │   ├── resource/                      # Resource files
│   │   │   └── rfbl_controller            # Package resource marker
│   │   └── test/                          # Unit tests
│   │       ├── test_copyright.py          # Copyright header tests
│   │       ├── test_flake8.py             # PEP8 linting tests
│   │       └── test_pep257.py             # Docstring format tests
│   ├── build/                             # Build directory (symlink)
│   └── install/                           # Install directory (symlink)
│
├── build/                                 # CMake build outputs
│   ├── rfbl_controller/                   # Package build folder
│   │   ├── build/                         # Intermediate build files
│   │   ├── colcon_build.rc                # Build configuration
│   │   ├── colcon_command_prefix_*.sh*    # Build environment scripts
│   │   ├── prefix_override/               # Prefix overrides
│   │   ├── rfbl_controller.egg-info/      # Python egg info
│   │   └── package.xml                    # Package metadata copy
│   └── COLCON_IGNORE                      # Marker to skip this directory
│
├── install/                               # Installation outputs
│   ├── rfbl_controller/                   # Installed package
│   │   ├── lib/                           # Binary and library files
│   │   │   ├── python3.x/                 # Python version-specific directory
│   │   │   │   └── site-packages/         # Installed Python packages
│   │   │   │       └── rfbl_controller/   # Installed module
│   │   │   └── rfbl_controller/           # Executable symlinks
│   │   │       └── rfbl_offboard_node     # Main node executable
│   │   └── share/                         # Documentation and metadata
│   │       └── rfbl_controller/           # Package documentation
│   │           ├── package.xml            # Installed package metadata
│   │           └── environment/           # Environment setup files
│   ├── setup.bash                         # Bash environment setup
│   ├── setup.sh                           # POSIX shell setup
│   ├── setup.ps1                          # PowerShell setup
│   ├── setup.zsh                          # Zsh environment setup
│   ├── local_setup.*                      # Local package environment
│   └── COLCON_IGNORE                      # Marker file
│
├── log/                                   # Build logs and timestamps
│   ├── latest                             # Symbolic link to latest build
│   ├── latest_build                       # Symbolic link to latest build logs
│   ├── latest_list                        # Symbolic link to latest package list
│   ├── build_YYYY-MM-DD_HH-MM-SS/         # Timestamped build logs (multiple)
│   │   └── rfbl_controller/               # Package-specific logs
│   │       └── stdout_stderr.log          # Build output
│   ├── list_YYYY-MM-DD_HH-MM-SS/          # Timestamped package lists
│   └── COLCON_IGNORE                      # Marker file
│
├── 2.pdf                                  # Research paper with theoretical foundation
│                                          # - Contains detailed equations (1)-(14)
│                                          # - Proof of finite-time stability
│                                          # - Experimental validation
│
└── README.md                              # This documentation file
```

### Directory Descriptions

#### **src/rfbl_controller/** (Source Package)
Main ROS 2 package containing all controller implementation code:
- **Python modules**: Core control algorithms
- **ROS 2 integration**: Node and message handling
- **Configuration**: Parameter definitions and trajectory references
- **Tests**: Unit tests and code quality checks

#### **build/** (Build Artifacts)
Intermediate build outputs generated by colcon:
- Compiled Python bytecode
- Generated scripts and configurations
- Temporary build files

#### **install/** (Installation Directory)
Final installed package ready for deployment:
- Executable node: `rfbl_offboard_node` (entry point)
- Library files: Python packages installed as site-packages
- Setup scripts: Environment initialization for bash, sh, ps1, zsh

#### **log/** (Build Logs)
Timestamped records of all builds and package lists:
- Individual build logs per package
- Diagnostic information for troubleshooting builds

---

## Core Components

### 1. **RFBLAltitudeAttitudeController** (`rfbl_controller.py`)

Implements the inner-loop altitude and attitude control with:
- Feedback linearization for nonlinear cancellation
- Supertwisting sliding-mode control for disturbance rejection
- Finite-time convergence guarantees
- Integration of supertwisting terms for sign-based control laws

**Key Methods:**
- `step(x, xd, xd_dot, d_hat, dt)`: Compute control outputs $F_z, \tau_\phi, \tau_\theta, \tau_\psi$
- `reset()`: Reset internal integrator states

### 2. **XYPositionRFBLSTC** (`xy_position_controller.py`)

Implements the outer-loop XY position control:
- Generates altitude and attitude references from desired XY positions
- Uses supertwisting control for position tracking
- Provides smooth reference trajectories to inner controller

**Key Methods:**
- `step(pos_error, vel, xd_ddot, d_hat_xy, dt)`: Generate attitude references
- `get_altitude_reference()`: Return desired altitude
- `get_attitude_reference()`: Return desired attitude angles

### 3. **HarmonicDisturbanceObserver** (`disturbance_observer.py`)

Estimates disturbances in altitude/attitude channels:
- Implements 2nd-order harmonic observer per axis
- Automatically tunes to disturbance frequency $\omega_{do}$
- Requires disturbance frequency knowledge

**Key Methods:**
- `step(x, u, dt)`: Update observer and return disturbance estimates
- `reset()`: Clear observer state

### 4. **HarmonicDisturbanceObserverXY** (`xy_disturbance_observer.py`)

Estimates disturbances in XY position channels:
- Similar architecture to altitude/attitude observer
- Adapted for translational dynamics

### 5. **RfblOffboardNode** (`rfbl_offboard_node.py`)

ROS 2 integration node:
- Subscribes to state, pose, and velocity from MAVROS
- Publishes attitude setpoints to PX4
- Implements main control loop (50 Hz)
- Integrates all controller components

---

## System Requirements

### Hardware Requirements

#### Minimum Specifications
- **Processor**: Intel i5/i7 or AMD Ryzen 5/7 (2.0 GHz+, quad-core recommended)
- **RAM**: 4 GB minimum (8 GB recommended)
- **Storage**: 10 GB free space (for ROS 2, packages, and build artifacts)
- **GPU**: Optional (can accelerate simulations if available)

#### Quadrotor Hardware (for real flight experiments)
- **Supported Platforms**:
  - DJI M100/M200/M300 (with custom PX4 firmware)
  - Pixhawk 4/Pixhawk 4 Mini
  - ArduPilot-compatible vehicles with MAVROS support
- **Telemetry Modules**: 3DR Radio, HolyBro, or equivalent (915 MHz or 433 MHz)
- **Ground Control Station**: PC with USB port for telemetry

#### Network Requirements
- **Local Network**: For SITL simulation and hardware communication
- **Bandwidth**: 1 Mbps sufficient for MAVROS telemetry
- **Latency**: <100 ms recommended for stable control

### Software Requirements

#### Operating System
- **Linux**: Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04 LTS
- **macOS**: Possible but not officially tested
- **Windows**: Use WSL 2 (Windows Subsystem for Linux) or Docker

#### ROS 2 Installation
| Component | Version | Notes |
|-----------|---------|-------|
| ROS 2 | Humble (recommended) | Installed and sourced |
| ROS Distribution | Iron/Jazzy | Compatible but not tested |
| Python | 3.10+ | Required for setup.py |
| pip | Latest | Package manager for Python |

#### Core Dependencies
```bash
# System packages
sudo apt install -y \
    build-essential \
    cmake \
    python3-dev \
    python3-venv \
    python3-colcon-common-extensions \
    git

# ROS 2 development tools
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-tf-transformations
```

#### Python Packages
| Package | Version | Purpose |
|---------|---------|---------|
| NumPy | >=1.20.0 | Numerical computations |
| SciPy | >=1.7.0 | Scientific computing (optional) |
| pytest | >=6.0 | Unit testing |
| setuptools | >=40.8 | Package distribution |

### Simulation Environment (Optional but Recommended)

For Software-in-the-Loop (SITL) testing without hardware:

#### PX4 SITL Setup
```bash
# Clone PX4 firmware
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Build SITL
make px4_sitl gazebo
```

#### Gazebo Simulator
```bash
# Install Gazebo
sudo apt install -y \
    gazebo \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins
```

### Development Tools (Optional)

| Tool | Purpose | Installation |
|------|---------|--------------|
| VS Code | Code editor | `sudo snap install code --classic` |
| RViz 2 | 3D visualization | `sudo apt install ros-humble-rviz2` |
| rqt | ROS graphical tools | `sudo apt install ros-humble-rqt*` |
| Terminator | Multi-terminal window | `sudo apt install terminator` |

### Verification of Requirements

Check your system setup:

```bash
# Check OS version
lsb_release -a

# Check ROS 2 installation
echo $ROS_DISTRO

# Check Python version
python3 --version

# Check key packages
python3 -c "import numpy, setuptools; print('NumPy:', numpy.__version__)"

# Check ROS 2 packages
ros2 pkg list | grep mavros
```

---

## Installation

### Prerequisites

- **ROS 2** (Humble or later)
- **Python 3.10+**
- **NumPy**
- **MAVROS** (for PX4 integration)
- **PX4** autopilot firmware

### Setup

1. **Clone the repository into your ROS 2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> rfbl_controller
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select rfbl_controller
   ```

4. **Source the setup file:**
   ```bash
   source install/setup.bash
   ```

---

## Configuration

### Quadrotor Parameters

Edit `params_refs.py` to set your quadrotor physical parameters:

```python
from rfbl_controller.params_refs import QuadrotorParams

params = QuadrotorParams(
    mass=1.5,              # kg
    Ixx=0.02,             # kg·m²
    Iyy=0.02,             # kg·m²
    Izz=0.04,             # kg·m²
    g=9.81,               # m/s²
    do_omega=2.0,         # disturbance observer frequency (rad/s)
    ...
)
```

### Controller Gains

Configure supertwisting and observer gains in the controller initialization:

```python
# Supertwisting gains (α² > 4β condition)
alpha_z = 5.0
beta_z = 3.0

# Observer gains
k1_observer = 40.0
k2_observer = 400.0
```

---

## Usage Guide

### Quick Start

#### 1. Build and Setup

```bash
# Navigate to workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select rfbl_controller

# Source the workspace
source install/setup.bash
```

#### 2. Run with PX4 SITL (Simulation)

**Terminal 1 - Start PX4 SITL:**
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

**Terminal 2 - Launch MAVROS bridge:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

**Terminal 3 - Run RFBL controller:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rfbl_controller rfbl_offboard_node
```

#### Example 2: Real-Time Telemetry Monitoring

```bash
# Monitor state topic
ros2 topic echo /mavros/state

# Monitor pose in real-time
ros2 topic echo /mavros/local_position/pose

# Monitor velocity
ros2 topic echo /mavros/local_position/velocity_local

# Monitor control outputs (if published)
ros2 topic echo /rfbl_controller/control_input
```

#### Example 3: Configuration Tuning

Edit `params_refs.py` to adjust controller gains:

```python
# Supertwisting gains for altitude (z)
ALPHA_Z = 5.0      # Increase for faster response
BETA_Z = 3.0       # Must satisfy: alpha^2 > 4*beta

# Supertwisting gains for attitude (phi, theta, psi)
ALPHA_PHI = 2.0
BETA_PHI = 1.0

ALPHA_THETA = 2.0
BETA_THETA = 1.0

ALPHA_PSI = 1.0
BETA_PSI = 0.5

# Disturbance observer frequency (rad/s)
# Tune based on expected disturbance characteristics
DO_OMEGA = 2.0     # Increase for faster disturbance estimation

# Observer gains (higher = faster response, lower = smoother)
K1_OBSERVER = 40.0
K2_OBSERVER = 400.0
```

#### Example 4: Running Unit Tests

```bash
# Run all tests
cd ~/ros2_ws
colcon test --packages-select rfbl_controller

# View detailed test results
cat log/latest_test/rfbl_controller/stdout_stderr.log

# Run specific test
python3 -m pytest src/rfbl_controller/test/test_flake8.py -v
```

#### Example 5: Logging and Debugging

Enable detailed logging in `rfbl_offboard_node.py`:

```python
#

### Hardware Deployment Workflow

#### Step 1: Prepare PX4 Vehicle

```bash
# Connect vehicle via USB/telemetry
# Upload latest PX4 firmware (v1.14+)
# Configure sensors and calibrate IMU
# Set vehicle parameters for your frame
```

#### Step 2: Configure MAVROS

Edit MAVROS launch file to match your setup:

```bash
# Standard connection (USB)
ros2 launch mavros px4.launch fcu_url:="/dev/ttyUSB0"

# Telemetry radio (serial port)
ros2 launch mavros px4.launch fcu_url:="/dev/ttyUSB0:57600"

# Network connection (UDP)
ros2 launch mavros px4.launch fcu_url:="udp://:14540@192.168.1.100:14557"
```

#### Step 3: Pre-Flight Checks

```bash
# Check MAVROS connection
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"

# Verify sensor data flow
ros2 topic hz /mavros/imu/data
ros2 topic hz /mavros/local_position/pose

# Check system status
ros2 topic echo /mavros/state --once
```

#### Step 4: Take-Off Procedure

```bash
# Arm vehicle (requires GPS lock and calibration)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Switch to Offboard mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# Command take-off altitude
# Use position reference publisher to set z = 1.0 m
```

### Troubleshooting Common Issues

#### Issue: MAVROS not connected
```bash
# Check if PX4 is running and MAVROS can see it
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"

# If fails, check connection parameters
netstat -tlnp | grep 14540  # For UDP
dmesg | tail                 # For serial connection
```

#### Issue: Unstable control / oscillations
```python
# Reduce supertwisting gains gradually
ALPHA_Z = 3.0  # Was 5.0
BETA_Z = 1.5   # Was 3.0

# Increase observer integration time
K1_OBSERVER = 20.0  # Was 40.0
K2_OBSERVER = 200.0 # Was 400.0
```

#### Issue: Slow convergence to setpoints
```python
# Increase control gains (carefully!)
ALPHA_Z = 7.0  # Increase from current value
BETA_Z = 4.0   # Adjust to maintain alpha^2 > 4*beta

# Reduce observer frequency if disturbances are slow
DO_OMEGA = 1.0  # Was 2.0
```

#### Issue: High CPU usage or timing delays
```bash
# Check node execution time
ros2 run rqt_graph rqt_graph  # Visualize node graph

# Profile the controller
python3 -m cProfile -s cumtime src/rfbl_controller/rfbl_offboard_node.py

# Reduce control loop frequency if needed
# In rfbl_offboard_node.py: self.control_rate = 25.0  # Instead of 50 Hz
```

### ROS 2 Commands Reference

| Command | Purpose |
|---------|---------|
| `colcon build` | Build all packages |
| `colcon build --packages-select rfbl_controller` | Build single package |
| `ros2 run rfbl_controller rfbl_offboard_node` | Run controller node |
| `ros2 topic list` | List all active topics |
| `ros2 topic echo /topic_name` | Display topic messages |
| `ros2 topic hz /topic_name` | Measure topic frequency |
| `ros2 service call /service_name ServiceType "{args}"` | Call service |
| `ros2 param list` | List all parameters |
| `rqt_graph` | Visualize node/topic graph |
| `ros2 doctor` | Check ROS 2 system health |

---

## Usage

---

## Features

 **Robust Control**: Supertwisting sliding-mode control for finite-time convergence and disturbance rejection

 **Feedback Linearization**: Nonlinear cancellation for accurate trajectory tracking

 **Disturbance Estimation**: Harmonic observers for persistent disturbance compensation

 **ROS 2 Integration**: Full integration with ROS 2 middleware for modularity

 **PX4 Compatible**: Direct integration with PX4 autopilot via MAVROS

 **Real-Time Control**: Fixed-rate control loop (50 Hz default, adjustable)

 **Numerical Stability**: Robust implementations with proper handling of edge cases

---

## Performance Characteristics

- **Control Loop Frequency**: 50 Hz (configurable)
- **Convergence Time**: Finite-time via supertwisting control
- **Disturbance Attenuation**: Depends on supertwisting gains and observer bandwidth
- **Attitude Tracking Error**: < 2° (typical under nominal conditions)
- **Position Tracking Error**: < 0.1 m (typical in controlled environment)

---

## Mathematical Guarantees

### Finite-Time Stability

The supertwisting sliding-mode control ensures **finite-time convergence** of the sliding surface to zero:

**Theorem (Robust Exact Differentiation):** If $\alpha^2 > 4\beta$, then for any bounded measurable disturbance, the supertwisting algorithm
$$\dot{s} = -\alpha|s|^{1/2}\text{sign}(s) - \beta\int\text{sign}(s)dt$$

drives $s$ to zero in finite time $T_f$ and maintains $s \equiv 0$ thereafter.

### Closed-Loop Stability

The combined feedback linearization + supertwisting controller ensures:
1. **Tracking error dynamics** converge to sliding surface $s_i = 0$
2. **Sliding surface dynamics** converge to $s_i \equiv 0$ in finite time
3. **Overall tracking error** $e_i \to 0$ in finite time

---

## Testing

Run the unit tests:

```bash
cd ~/ros2_ws
colcon build --packages-select rfbl_controller
colcon test --packages-select rfbl_controller
```

---

## Troubleshooting

### Issue: Controller outputs NaN
- **Cause**: Uninitialized state or extreme gains
- **Solution**: Ensure `reset()` is called before first use; check gain values satisfy $\alpha^2 > 4\beta$

### Issue: Oscillatory behavior
- **Cause**: Gains too aggressive or disturbance frequency mismatch
- **Solution**: Reduce supertwisting gains or adjust `do_omega` in parameters

### Issue: Slow convergence
- **Cause**: Gains too conservative
- **Solution**: Increase gains (while maintaining stability conditions)

---

## References

- **Primary Reference**: Research paper (2.pdf) - Contains detailed derivations of equations (1)-(14)
- **Feedback Linearization**: Isidori, A., "Nonlinear Control Systems," 3rd Ed., Springer, 1995
- **Sliding Mode Control**: Utkin, V.I., et al., "Sliding Mode Control in Electromechanical Systems," CRC Press, 2009
- **Supertwisting Algorithm**: Levant, A., "Robust Exact Differentiation via Sliding Mode Technique," Automatica, 1998
- **Disturbance Observer Design**: Thao, T.P., et al., "Harmonic Disturbance Observer Design," IEEE Transactions on Control Systems Technology, 2020

---

## Future Enhancements

- [ ] Model predictive control (MPC) layer for trajectory planning
- [ ] Adaptive disturbance observer frequency tuning
- [ ] Multi-agent swarm control
- [ ] Hardware-in-the-loop (HIL) testing
- [ ] Real flight experiments on physical quadrotors
- [ ] Advanced reference trajectory generation

---

## Contributing

Contributions are welcome! Please ensure:
1. Code follows PEP 8 style guidelines
2. Unit tests are provided for new features
3. Documentation is updated accordingly

---

## Appendix: Mathematical Notation

A comprehensive reference for all mathematical symbols and notation used throughout this documentation and implementation.

### Position & Velocity States

| Symbol | Definition | Unit | Description |
|--------|-----------|------|-------------|
| $x, y, z$ | Position | m | Inertial frame coordinates (x, y horizontal; z vertical) |
| $\dot{x}, \dot{y}, \dot{z}$ | Velocity | m/s | Time derivatives of position |
| $\ddot{x}, \ddot{y}, \ddot{z}$ | Acceleration | m/s² | Second time derivatives (inertial acceleration) |
| $e_x, e_y, e_z$ | Position Error | m | Tracking error: $e_i = i - i_d$ |

### Attitude & Angular Velocity

| Symbol | Definition | Unit | Description |
|--------|-----------|------|-------------|
| $\phi$ (roll) | Rotation about x-axis | rad | Bank angle for horizontal motion |
| $\theta$ (pitch) | Rotation about y-axis | rad | Pitch angle for horizontal motion |
| $\psi$ (yaw) | Rotation about z-axis | rad | Heading angle relative to inertial frame |
| $\dot{\phi}, \dot{\theta}, \dot{\psi}$ | Angular velocity | rad/s | Roll, pitch, yaw rates |
| $\ddot{\phi}, \ddot{\theta}, \ddot{\psi}$ | Angular acceleration | rad/s² | Roll, pitch, yaw angular accelerations |
| $e_\phi, e_\theta, e_\psi$ | Attitude Error | rad | Tracking error in attitude angles |

### Control Inputs

| Symbol | Definition | Unit | Description |
|--------|-----------|------|-------------|
| $F_z$ | Vertical thrust | N | Total lift force from all four rotors |
| $\tau_\phi$ | Roll torque | N·m | Torque about body x-axis |
| $\tau_\theta$ | Pitch torque | N·m | Torque about body y-axis |
| $\tau_\psi$ | Yaw torque | N·m | Torque about body z-axis |

### Quadrotor Physical Parameters

| Symbol | Definition | Unit | Description |
|--------|-----------|------|-------------|
| $m$ | Mass | kg | Total quadrotor mass (including batteries/payload) |
| $I_x, I_y, I_z$ | Moments of inertia | kg·m² | Rotational inertia about body axes |
| $g$ | Gravitational acceleration | m/s² | Standard: 9.81 m/s² |
| $c_d$ | Drag coefficient | dimensionless | Aerodynamic damping coefficient |

### Disturbances

| Symbol | Definition | Unit | Description |
|--------|-----------|------|-------------|
| $d_z, d_x, d_y$ | Position disturbances | m/s² | External forces (wind, measurement noise) |
| $d_\phi, d_\theta, d_\psi$ | Attitude disturbances | rad/s² | External torques and model uncertainties |
| $d_{meas}$ | Measured disturbance | varies | Disturbance estimated from dynamics |
| $\hat{d}$ | Estimated disturbance | varies | Observer output for disturbance compensation |

### Sliding-Mode Control Variables

| Symbol | Definition | Unit | Description |
|--------|-----------|------|-------------|
| $s_i$ | Sliding surface | varies | Error manifold for axis $i$ |
| $\lambda_i$ | Design parameter | varies | Sliding surface gain (positive) |
| $\alpha_i$ | Supertwisting gain | varies | First-order sliding gain: $\sqrt{\|\dot{s}\|}$ term |
| $\beta_i$ | Supertwisting gain | varies | Second-order sliding gain: integral term |
| $\text{sign}(s)$ | Sign function | {-1, 0, 1} | Defined as: $\text{sign}(x) = \begin{cases} 1 & x > 0 \\ 0 & x = 0 \\ -1 & x < 0 \end{cases}$ |

### Observer & Reference Signals

| Symbol | Definition | Unit | Description |
|--------|-----------|------|-------------|
| $x_d, \dot{x}_d, \ddot{x}_d$ | Desired state/derivatives | varies | Reference trajectory for state $x$ |
| $z_1, z_2$ | Observer states | varies | Internal states of harmonic disturbance observer |
| $k_1, k_2$ | Observer gains | varies | Feedback gains for disturbance observer |
| $\omega_{do}$ | Disturbance frequency | rad/s | Known frequency of periodic disturbances |

### Mathematical Operators & Functions

| Symbol | Definition | Description |
|--------|-----------|-------------|
| $\|·\|$ | Absolute value/Norm | Magnitude of a scalar or vector |
| $\|\|·\|\|$ | Euclidean norm | $\|\|v\|\| = \sqrt{v_x^2 + v_y^2 + v_z^2}$ |
| $\int (\cdot) dt$ | Integration | Time integral (cumulative sum in discrete form) |
| $\frac{d}{dt}$ | Time derivative | Rate of change with respect to time |
| $(\cdot)^{1/2}$ | Square root | Non-smooth function at zero |
| $(\cdot)_d$ | Desired/reference quantity | Subscript $d$ denotes desired/commanded values |
| $\hat{(\cdot)}$ | Estimated quantity | Hat notation denotes estimated/observed values |

### System Indices

| Notation | Meaning | Components |
|----------|---------|-----------|
| States $x_1, ..., x_8$ | Full state vector | $[z, \dot{z}, \phi, \dot{\phi}, \theta, \dot{\theta}, \psi, \dot{\psi}]$ |
| State $x_i$ | i-th state | Odd indices: position/angle; Even indices: velocity/rate |
| Errors $e_1, ..., e_8$ | Error vector | $e_i = x_i - x_{id}$ |
| First derivative: $\dot{x}$ | Velocity component | Odd-indexed states for position; even for velocity |
| Second derivative: $\ddot{x}$ | Acceleration | Computed from velocity derivatives |

### Key Inequalities & Conditions

| Condition | Requirement | Description |
|-----------|-------------|-------------|
| Supertwisting stability | $\alpha^2 > 4\beta$ | Critical condition for finite-time convergence |
| Sliding surface convergence | $s_i \to 0$ (finite time) | Achieved when control law is active |
| Tracking error boundedness | $e_i \to 0$ (finite time) | Follows from sliding surface convergence |
| Observer tunability | $\omega_{do}$ known/estimated | Harmonic observer requires disturbance frequency info |

### State Vector Definition

**Full state vector** (8-dimensional):
$$\mathbf{x} = [x_1, x_2, x_3, x_4, x_5, x_6, x_7, x_8]^T = [z, \dot{z}, \phi, \dot{\phi}, \theta, \dot{\theta}, \psi, \dot{\psi}]^T$$

**Position subsystem**:
$$\mathbf{x}_{pos} = [z, \dot{z}, x, \dot{x}, y, \dot{y}]^T$$

**Attitude subsystem**:
$$\mathbf{x}_{att} = [\phi, \dot{\phi}, \theta, \dot{\theta}, \psi, \dot{\psi}]^T$$

**Control input vector** (4-dimensional):
$$\mathbf{u} = [F_z, \tau_\phi, \tau_\theta, \tau_\psi]^T$$

**Disturbance vector** (6-dimensional):
$$\mathbf{d} = [d_z, d_\phi, d_\theta, d_\psi, d_x, d_y]^T$$

---

# References
1. Kamal, A. Arif, and M. Rafique, “Robust Feedback Linearization Based Disturbance Observer Control of Quadrotor UAV,” International Journal of Control, Automation and Systems, 2020
2. Quadrotor Dynamics Foundations
R. M. Murray, Z. Li, and S. S. Sastry, A Mathematical Introduction to Robotic Manipulation, CRC Press, 1994.
3. Nonlinear Systems & Feedback Linearization
A. Isidori, Nonlinear Control Systems, Springer, 1995.
4. Disturbance Observer Theory
K. Ohnishi, “A new servo method in mechatronics,” Trans. Japanese Soc. Electrical Engineers, vol. 107-D, pp. 83–86, 1987.
5. Quadrotor Modeling & Classical Control Reference
T. Lee, M. Leok, and N. H. McClamroch, “Geometric tracking control of a quadrotor UAV on SE(3),” Proceedings of the IEEE Conference on Decision and Control (CDC), 2010.
6. PX4 Documentation
PX4 Autopilot User Guide, https://docs.px4.io/
7. ROS2 Documentation
ROS 2 Documentation, https://docs.ros.org/
8. MAVLink / Offboard Control Reference
MAVLink Developer Guide, https://mavlink.io/en/

## Acknowledgments

This implementation is based on research and development in robust nonlinear control and sliding-mode techniques. Special thanks to the ROS 2 and PX4 communities for excellent middleware and autopilot platforms.

