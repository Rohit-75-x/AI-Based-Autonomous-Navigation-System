# AI-Based Autonomous Navigation System 🚗💨

[![Python Version](https://img.shields.io/badge/Python-3.9%20%7C%203.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue.svg?logo=python)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/Unit%20Tests-8%2F8%20Passed-brightgreen.svg?logo=pytest)](tests/)
[![Domain](https://img.shields.io/badge/Domain-Robotics%20%26%20Autonomous%20Vehicles-orange.svg)]()
[![Hardware Ready](https://img.shields.io/badge/Hardware-Raspberry%20Pi%20%7C%20Jetson%20Nano-blueviolet.svg)](docs/HARDWARE_DEPLOYMENT_GUIDE.md)

An industry-oriented, modular **Level-4 Autonomous Ground Vehicle (AGV) Navigation Software Stack** built in Python. Features real-time **360° LiDAR raycasting**, **Computer Vision lane & obstacle detection (OpenCV)**, **A\* global optimal path planning**, **gradient descent path smoothing (Thrun's algorithm)**, **kinematic bicycle model dynamics**, and **Pure Pursuit path tracking with reactive collision evasion**.

Works **100% in virtual simulation** without physical hardware, with complete bridging schematics for **Raspberry Pi 4 / NVIDIA Jetson Nano** deployment.

---

## 📸 Simulation Gallery & Proof of Work

### 1. Urban City Scenario: Path Planning, 360° LiDAR Raycasting & Telemetry
![City Simulation Proof](assets/images/simulation_demo.png)

### 2. Industrial Warehouse Scenario: Aisle Navigation & Dynamic Collision Avoidance
![Warehouse Simulation Proof](assets/images/warehouse_demo.png)

---

## 🌟 Key Features

- **🌐 360° Planar LiDAR Simulation**: Custom raycasting engine calculating omnidirectional obstacle distance, hazard sectors (Front, Left, Right), and emergency braking envelopes.
- **👁️ Computer Vision Perception Pipeline**:
  - Synthetic monocular dashboard camera projection.
  - Canny edge detection and Probabilistic Hough Transform (`cv2.HoughLinesP`) for road lane tracking.
  - YOLO-style bounding box projections with range and confidence scoring.
- **🧭 A\* Optimal Path Planning**:
  - 8-directional heuristic graph search with Octile distance metric.
  - Morphological obstacle inflation to maintain safety buffer from obstacles.
  - Strict diagonal adjacency checks preventing corner cutting.
- **〰️ Continuous Trajectory Smoothing**:
  - Sebastian Thrun's gradient descent path relaxation algorithm.
  - Equidistant waypoint spline interpolation.
- **🏎️ Kinematic Bicycle Model**:
  - Non-holonomic vehicle equations of motion ($\dot{x}, \dot{y}, \dot{\theta}$).
  - Dynamic steering rate limits, rolling friction, and throttle/braking dynamics.
- **🎯 Pure Pursuit & Reactive Evasion Controller**:
  - Geometric lookahead curvature calculation for steering.
  - PID longitudinal speed regulator.
  - Dynamic obstacle evasion and autonomous re-routing when paths are obstructed.
- **🖥️ Dual Visualization Engines**:
  - **Pygame + OpenCV Real-time Engine**: 60 FPS interactive driving arena with live camera picture-in-picture (PIP).
  - **Matplotlib Scientific Engine**: Zero-install radar plot and real-time kinematic graphs.

---

## 🏛️ System Architecture

```
                  +----------------------------------------------+
                  |         WORLD SIMULATION ENVIRONMENT         |
                  |  (Static Road Obstacles, Dynamic Pedestrians)|
                  +----------------------------------------------+
                                         |
                                         v
                  +----------------------------------------------+
                  |             PERCEPTION SUBSYSTEM             |
                  |  +---------------------+ +-----------------+ |
                  |  |  360° LiDAR Raycast | | Monocular Camera| |
                  |  |  Clearance Matrix   | | Lane / Bounding | |
                  |  +---------------------+ +-----------------+ |
                  +----------------------------------------------+
                                         |
                                         v
                  +----------------------------------------------+
                  |            PATH PLANNING SUBSYSTEM           |
                  |  +---------------------+ +-----------------+ |
                  |  | 2D Occupancy Grid   | | Thrun's Gradient| |
                  |  | A* Heuristic Search | | Path Smoothing  | |
                  |  +---------------------+ +-----------------+ |
                  +----------------------------------------------+
                                         |
                                         v
                  +----------------------------------------------+
                  |             CONTROL & KINEMATICS             |
                  |  +---------------------+ +-----------------+ |
                  |  | Pure Pursuit Tracker| | PID Speed Loop  | |
                  |  | Reactive Avoidance  | | Bicycle Model   | |
                  |  +---------------------+ +-----------------+ |
                  +----------------------------------------------+
                                         |
                                         v
                  +----------------------------------------------+
                  |            VISUALIZATION DASHBOARD           |
                  | (Arena HUD, Polar LiDAR Radar, Camera Feed)  |
                  +----------------------------------------------+
```

---

## 📂 Repository Structure

```
AI-Autonomous-Navigation-System/
├── assets/
│   └── images/
│       ├── simulation_demo.png         # Verified city simulation screenshot
│       └── warehouse_demo.png          # Verified warehouse simulation screenshot
├── docs/
│   ├── HARDWARE_DEPLOYMENT_GUIDE.md    # Raspberry Pi / Jetson Nano wiring & sim-to-real code
│   ├── ALGORITHM_EXPLANATION.md        # Mathematical derivations of A*, Pure Pursuit, Kinematics
│   └── INTERVIEW_PREPARATION.md        # Top 10 interview Q&A, HR elevator pitch, technical answers
├── outputs/
│   └── screenshots/                    # Real-time saved simulation frames
├── src/
│   ├── config.py                       # Global simulation parameters and hyperparameters
│   ├── simulation/
│   │   ├── environment.py              # City/Warehouse world maps and dynamic obstacles
│   │   ├── vehicle.py                  # Kinematic bicycle model & state tracker
│   │   └── sensors.py                  # 360 LiDAR raycaster & camera frustum simulator
│   ├── perception/
│   │   ├── vision_detector.py          # OpenCV lane detection & synthetic camera pipeline
│   │   └── lidar_processor.py          # Polar scan analysis & hazard categorization
│   ├── planning/
│   │   ├── a_star.py                   # 8-directional A* heuristic pathfinder
│   │   └── path_smoother.py            # Gradient descent path smoother & interpolator
│   ├── control/
│   │   ├── pid_controller.py           # Longitudinal throttle/braking PID controller
│   │   └── pure_pursuit.py             # Geometric lateral tracking & local evasion
│   └── ui/
│       ├── dashboard.py                # Pygame interactive 60 FPS graphics engine
│       └── matplotlib_dashboard.py     # Matplotlib scientific radar & telemetry dashboard
├── tests/
│   ├── test_a_star.py                  # A* path validity and obstacle clearance tests
│   ├── test_sensors.py                 # Raycasting and field-of-view unit tests
│   └── test_vehicle.py                 # Kinematic bicycle integration tests
├── .gitignore
├── LICENSE                             # MIT Open Source License
├── requirements.txt                    # Project dependencies
├── main.py                             # Main simulation application entry point
└── README.md                           # Documentation and portfolio overview
```

---

## 🚀 Quickstart & Installation

### 1. Clone the Repository
```bash
git clone https://github.com/your-username/AI-Autonomous-Navigation-System.git
cd AI-Autonomous-Navigation-System
```

### 2. Create Virtual Environment
#### On Windows:
```powershell
python -m venv venv
venv\Scripts\activate
```
#### On macOS / Linux:
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

---

## 🎮 How to Run

### Run Scientific Radar Dashboard (Out-of-the-box Matplotlib)
```bash
python main.py --backend matplotlib
```

### Run Full 60 FPS Interactive Simulator (with Pygame + OpenCV)
```bash
python main.py
```

### Switch Environment Map
```bash
python main.py --map warehouse
```

### Run Automated Headless Verification
```bash
python main.py --headless
```

### Export Real-Time Proof Screenshots
```bash
python main.py --export-proof
```

---

## ⌨️ Interactive Controls

| Control | Action |
| :--- | :--- |
| **`SPACE`** | Pause / Resume the autonomous simulation |
| **`R`** | Reset vehicle pose to starting point and re-plan |
| **`M`** | Toggle world map between **City** and **Warehouse** |
| **`S`** | Capture instantaneous proof screenshot to `outputs/screenshots/` |
| **`Left Click`** | Set new dynamic destination Goal anywhere on the map |
| **`Right Click`** | Spawn a dynamic obstacle block (triggers instant re-planning) |

---

## 🧪 Unit Testing Suite

Verify the algorithmic validity of all core modules:
```bash
python -m pytest tests/ -v
```

Expected output:
```
tests/test_a_star.py::test_astar_simple_path PASSED                     [ 37%]
tests/test_a_star.py::test_astar_with_wall PASSED                       [ 50%]
tests/test_a_star.py::test_astar_blocked_goal PASSED                    [ 62%]
tests/test_sensors.py::test_lidar_wall_detection PASSED                 [ 75%]
tests/test_sensors.py::test_camera_fov_filter PASSED                    [ 87%]
tests/test_vehicle.py::test_vehicle_forward_motion PASSED               [100%]
============================== 8 passed in 0.22s ==============================
```

---

## 🤖 Sim-to-Real Hardware Bridge

This software stack is designed with modular interfaces for physical deployment:
- See [`docs/HARDWARE_DEPLOYMENT_GUIDE.md`](docs/HARDWARE_DEPLOYMENT_GUIDE.md) for full electrical wiring diagrams, Raspberry Pi 4 GPIO pinouts, and the Python motor controller script.
- See [`docs/ALGORITHM_EXPLANATION.md`](docs/ALGORITHM_EXPLANATION.md) for complete mathematical proofs.
- See [`docs/INTERVIEW_PREPARATION.md`](docs/INTERVIEW_PREPARATION.md) for job interview prep and placement pitches.

---

## 📄 License
Distributed under the MIT License. See `LICENSE` for more information.
