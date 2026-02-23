# ACC QCar2 — Autonomous Driving Platform

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-green.svg)](https://www.python.org/)
[![NVIDIA Isaac ROS](https://img.shields.io/badge/NVIDIA-Isaac%20ROS-76b900.svg)](https://nvidia-isaac-ros.github.io/)
[![License](https://img.shields.io/badge/license-Apache%202.0-orange.svg)](LICENSE)

Autonomous navigation stack for the **Quanser QCar 2** in **Quanser Interactive Labs (QLabs)**.
Built for the **ACC Self-Driving Car Competition** — covers SLAM, waypoint following, path planning, real-time camera feeds, and a browser-based control dashboard.

<p align="center">
  <img src="https://img.shields.io/badge/status-active%20development-brightgreen" alt="Status">
</p>

---

## Table of Contents

- [Features](#-features)
- [Architecture](#%EF%B8%8F-architecture)
- [Quick Start](#-quick-start)
- [Usage Modes](#-usage-modes)
- [Web Dashboard](#%EF%B8%8F-web-dashboard)
- [Project Structure](#-project-structure)
- [Roadmap](#%EF%B8%8F-roadmap)
- [Contributing](#-contributing)
- [References](#-references)
- [License](#-license)

---

## ✨ Features

| Module | Description |
|--------|-------------|
| **Pure Pursuit Controller** | Waypoint-following with anti-zigzag tuning (v9), configurable look-ahead, steering rate limiter |
| **Stanley Controller** | Lateral control alternative for comparison |
| **Web Dashboard** | Browser-based control center with 4 live camera feeds, click-to-waypoint, telemetry charts |
| **Matplotlib GUI** | Desktop GUI for waypoint management on the circuit map |
| **SLAM (Cartographer)** | Real-time 2D mapping and localization via Google Cartographer |
| **Drive & Record** | Record routes via keyboard teleop, replay them autonomously |
| **RViz Waypoints** | Interactive waypoint placement in RViz with track background |
| **QLabs Integration** | Overhead camera, ground-truth pose, multi-scenario launcher |
| **Camera Pipeline** | CSI fisheye + RGBD color/depth streams, MJPEG over HTTP |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Terminal 1: QLabs Scenario (Quanser Docker)                    │
│  ./run_t1.sh → Competition Map / Real Scenario                 │
├─────────────────────────────────────────────────────────────────┤
│  Terminal 2: Isaac ROS Container                                │
│  ./run_t2.sh → QCar2 + Cartographer (SLAM)                     │
├─────────────────────────────────────────────────────────────────┤
│  Terminal 3: Autonomy Nodes (inside Isaac ROS)                  │
│  Pure Pursuit + Web Dashboard + Cameras                         │
├─────────────────────────────────────────────────────────────────┤
│  Browser: http://localhost:8085                                 │
│  Live cameras │ Click waypoints │ Telemetry │ Route control     │
└─────────────────────────────────────────────────────────────────┘
```

**ROS 2 Node Graph:**

```
/qcar2_hardware ──► /scan, /camera/csi_image, /camera/color_image, /camera/depth_image
/cartographer   ──► /map, TF (map → odom → base_link)
/qcar_pose_gt   ──► /qcar_pose_gt (ground truth from QLabs)
/pure_pursuit   ◄── /waypoints_path  ──► /cmd_vel_nav
/web_dashboard  ◄── all topics       ──► HTTP :8085
```

---

## 🚀 Quick Start

### Prerequisites

- **OS:** Ubuntu 22.04
- **GPU:** NVIDIA (for Isaac ROS container)
- **Software:** Docker, NVIDIA Container Toolkit
- **Simulation:** Quanser Interactive Labs with *ACC Self-Driving Car Competition* scenario

### Step-by-step

**1. Clone the repository:**

```bash
git clone https://github.com/AldonDC/acc-qcar2-autonomy.git
cd acc-qcar2-autonomy
```

**2. Open QLabs:**
- Launch Quanser Interactive Labs
- Enter *ACC Self Driving Car Competition*
- Open the scenario/workspace

**3. Terminal 1 — Launch QLabs scenario:**

```bash
./run_t1.sh
```

Select from the menu:
| Option | Scenario |
|--------|----------|
| 1 | 🗺️ Empty competition map |
| 2 | 🏁 Full scenario (traffic lights, stop signs, roundabout) |
| 3 | ⚡ Empty — Interleaved (better FPS) |
| 4 | 🚀 Full — Interleaved (better FPS) |

**4. Terminal 2 — Start QCar + SLAM:**

```bash
./run_t2.sh
```

**5. Terminal 3 — Enter the container and start autonomy:**

```bash
./isaac_ros_common/scripts/run_dev.sh ./Development -- bash
cd /workspaces/isaac_ros-dev/ros2
colcon build --packages-select qcar2_autonomy && source install/setup.bash
```

Then choose your mode:

| Mode | Command |
|------|---------|
| **Web Dashboard + Pure Pursuit** | `ros2 launch qcar2_autonomy qcar_web_dashboard_launch.py` |
| **GUI + Pure Pursuit** | `ros2 launch qcar2_autonomy waypoint_full_launch.py` |
| **Pure Pursuit only** | `ros2 run qcar2_autonomy waypoint_follower_pure_pursuit` |
| **Web Dashboard only** | `ros2 run qcar2_autonomy qcar_web_dashboard` |

**6. Open browser:** `http://localhost:8085`

---

## 📋 Usage Modes

### Web Dashboard (Recommended)

Click waypoints on the overhead map, view all cameras, monitor telemetry — all from the browser.

```bash
# Terminal 3 (inside container):
ros2 launch qcar2_autonomy qcar_web_dashboard_launch.py
```

Then open `http://localhost:8085`.

### Drive & Record

Record a route by driving with keyboard, then replay it autonomously.

**Record:**

```bash
# Terminal 3: Teleop keyboard
ros2 run qcar2_autonomy simple_teleop_keyboard

# Terminal 4: Record route
./record.sh my_route
# Press Ctrl+C to stop → saves to routes/my_route.json
```

**Replay:**

```bash
./navigate.sh routes/my_route.json       # Standard replay
./navigate.sh routes/my_route.json 5     # Smoother (1 of every 5 points)
./navigate.sh routes/my_route.json 1 true # Infinite loop
```

### RViz Waypoints

Interactive waypoints with track background image in RViz.

```bash
ros2 launch qcar2_autonomy waypoint_rviz_launch.py
```

- Click **Publish Point** on the map to add waypoints
- Send route: `ros2 service call /send_path std_srvs/srv/Empty`
- Clear: `ros2 service call /waypoint_rviz_node/clear_waypoints std_srvs/srv/Empty`

**Load a previously recorded route as reference:**

```bash
ros2 launch qcar2_autonomy waypoint_rviz_launch.py \
  reference_file:=/workspaces/isaac_ros-dev/ros2/routes/my_route.json
```

---

## 🖥️ Web Dashboard

Professional browser-based control center at `http://localhost:8085`.

**Features:**
- 🗺️ **Track Map** — Live QLabs overhead with car position, path, and waypoint overlay
- 📷 **CSI Camera** — Front fisheye camera feed
- 🎨 **RGBD Color** — Intel RealSense color stream
- 🌊 **RGBD Depth** — Colorized depth map (TURBO colormap)
- 📊 **Telemetry** — Linear/angular velocity charts, pose, heading
- 🎯 **Click-to-Waypoint** — Click on the map canvas to add waypoints
- ↶ **Undo / Clear / Send** — Route management buttons

**Architecture:** Python HTTP server + MJPEG streaming + Canvas-based map with frame polling. Zero external dependencies.

---

## 📁 Project Structure

```
ACC_Development/
├── README.md
├── run_t1.sh                          # T1: QLabs scenario launcher (interactive menu)
├── run_t2.sh                          # T2: Isaac ROS container + QCar + Cartographer
├── run_t2_web.sh                      # T2 variant: includes Web Dashboard
├── run_qcar2.sh                       # Standalone QCar launch
│
├── isaac_ros_common/                  # NVIDIA Isaac ROS development container
│
├── Development/
│   ├── python_resources/              # Quanser Python libraries (qcar, qcar2, roadmap)
│   └── ros2/
│       ├── record.sh                  # Route recording helper
│       ├── navigate.sh                # Route replay helper
│       ├── run_qcar2_with_map.sh      # Internal: QCar + Cartographer
│       ├── run_qcar2_with_web_dashboard.sh  # Internal: + Web Dashboard
│       ├── routes/                    # Saved routes (.json)
│       └── src/
│           ├── qcar2_autonomy/        # 🎯 Main autonomy package
│           │   ├── autonomy/
│           │   │   ├── waypoint_follower_pure_pursuit.py  # Pure Pursuit v9
│           │   │   ├── waypoint_follower_stanley.py       # Stanley controller
│           │   │   ├── qcar_web_dashboard.py              # Web Dashboard
│           │   │   ├── qcar_dashboard_gui.py              # Matplotlib GUI
│           │   │   ├── qcar_pose_from_qlabs_node.py       # QLabs ground truth
│           │   │   ├── route_recorder.py                  # Record routes
│           │   │   ├── route_player.py                    # Replay routes
│           │   │   ├── simple_teleop_keyboard.py          # Keyboard control
│           │   │   └── waypoint_rviz_node.py              # RViz waypoints
│           │   ├── launch/
│           │   │   ├── waypoint_web_launch.py             # QCar+SLAM+PP+WebDash
│           │   │   ├── waypoint_full_launch.py            # QCar+SLAM+PP+GUI
│           │   │   ├── qcar_web_dashboard_launch.py       # WebDash+PP standalone
│           │   │   └── waypoint_rviz_launch.py            # RViz waypoints
│           │   └── config/
│           ├── qcar2_nodes/           # Hardware drivers, TF, Cartographer
│           └── qcar2_interfaces/      # Custom ROS messages/services
│
├── docker/                            # Docker configurations
│   ├── quanser_docker/                # QLabs virtual QCar container
│   └── development_docker/            # Dev environment setup
│
└── scripts/
    └── setup_swap_8gb.sh              # Swap memory setup for low-RAM systems
```

---

## 🗺️ Roadmap

### ✅ Completed

- [x] Pure Pursuit controller v9 with anti-zigzag tuning
- [x] Stanley controller (alternative)
- [x] SLAM via Cartographer (2D mapping + localization)
- [x] Drive & Record (keyboard teleop → JSON routes → autonomous replay)
- [x] RViz waypoint placement with track background
- [x] Interactive `run_t1.sh` scenario menu (4 scenarios + stop/restart)
- [x] Matplotlib GUI for waypoint management
- [x] Web Dashboard with 4 camera feeds + telemetry + click-to-waypoint
- [x] QLabs overhead camera integration
- [x] Ground-truth pose from QLabs (`qcar_pose_from_qlabs_node`)

### 🔧 In Progress / Next Steps

- [ ] **Test Pure Pursuit v9** — Validate anti-zigzag parameters in QLabs
- [ ] **Traffic light detection** — Camera-based red/green/yellow classification
- [ ] **Stop sign detection** — YOLO or classical CV for stop sign recognition
- [ ] **Yield sign handling** — Detect and react to yield signs
- [ ] **Roundabout navigation** — Merge/exit logic for the roundabout section
- [ ] **Lane following** — Camera-based lane centering as fallback controller
- [ ] **Velocity adaptation** — Slow down at curves, speed up on straights
- [ ] **Obstacle avoidance** — Lidar-based reactive obstacle avoidance
- [ ] **Competition integration** — Full autonomous run: start → navigate all signs → finish

### 💡 Future Ideas

- [ ] Multi-vehicle coordination
- [ ] Real QCar 2 hardware deployment
- [ ] Path optimization (minimum curvature)
- [ ] Reinforcement learning controller

---

## 🤝 Contributing

Contributions are welcome via **Pull Requests**.

### Workflow

1. **Fork** the repository and clone locally
2. Check **Issues** for open tasks
3. Create a **feature branch**: `git checkout -b feature/my-feature`
4. Commit with descriptive messages
5. Open a **Pull Request** against `main`

### PR Description Template

| Field | Content |
|-------|---------|
| **Summary** | What problem does this solve / what feature does it add? |
| **Changes** | List of concrete changes (files, behaviors) |
| **Areas modified** | Which packages/modules? (autonomy, nodes, launch, config) |
| **Testing** | How was this tested? (QLabs sim, unit test, etc.) |

Include `Closes #N` in the PR description to auto-close issues.

---

## 📚 References

- [Quanser](https://www.quanser.com/) — QCar 2 & Interactive Labs
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/) — Development container
- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) — Pure Pursuit reference
- [Smart Mobility 2025](https://github.com/abrahammorohdez19/smart_mobility_2025) — QCar reference project
- [Google Cartographer](https://google-cartographer.readthedocs.io/) — SLAM framework
- [OpenCV](https://docs.opencv.org/) — Computer vision library

---

## 📄 License

This project is licensed under **Apache 2.0** — compatible with the ROS 2 ecosystem and Quanser academic resources.
