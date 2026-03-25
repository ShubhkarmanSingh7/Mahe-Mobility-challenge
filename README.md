# Project Vayu — Autonomous Campus Delivery Robot

> **MAHE Mobility Challenge 2026** · ROS 2 Jazzy · TurtleBot3 Waffle Pi

> **Status:** This repository currently holds our foundational Nav2, EKF, and ArUco perception boilerplate. The custom Semantic Social Awareness (YOLOv8) and YIELD logic will be integrated during Phase 2 of the hackathon.

Project Vayu is a fully autonomous delivery robot designed for indoor campus corridors. It navigates using an EKF-fused pose estimate, detects ArUco markers at corridor junctions for directional decisions, and uses the Nav2 stack for path planning and obstacle avoidance.

---

## Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                     simulation.launch.py                    │
│  Gazebo ─► RSP ─► spawn ─► EKF ─► Nav2 ─► Vayu Nodes        │
└─────────────────────────────────────────────────────────────┘

Sensors                    Nodes                     Outputs
─────────                 ──────                    ───────
/camera/image_raw  ──►  PerceptionNode      ──►  /vayu/detected_symbol
                        (ArUco + YOLOv8n)

/scan              ──►  Nav2 Costmaps       ──►  /cmd_vel (Standard)
/imu/data          ──►  EKF (robot_localization)
/odom              ──►  EKF ──► /odometry/filtered
                          │
                   LocalizationNode ──► TF (odom→base_link)
                          │
                          │
                    DecisionNode (FSM)  ──► navigate_to_pose action
                    (Social Awareness)  ──► /cmd_vel_override (YIELD)
```

## Package Structure

```
project_vayu/
├── config/
│   ├── ekf.yaml               # robot_localization EKF parameters
│   ├── nav2_params.yaml       # Nav2 DWB / costmap / planner config
│   ├── bt_navigator.xml       # Nav2 behavior tree
│   └── symbol_map.yaml        # ArUco ID → action mapping
├── launch/
│   ├── simulation.launch.py   # Full simulation bringup
│   ├── nav2.launch.py         # Nav2 stack only
│   └── vayu_nodes.launch.py   # Core Vayu nodes only
├── msg/
│   └── DetectedSymbol.msg     # Custom detection message
├── project_vayu/
│   ├── __init__.py
│   ├── localization_node.py   # EKF diagnostics + TF republisher
│   ├── perception_node.py     # Dual-path ArUco + YOLOv8n semantic detection
│   ├── decision_node.py       # FSM-driven navigation planner
│   └── utils/
│       ├── __init__.py
│       ├── fsm.py             # Finite state machine
│       └── confidence_gate.py # Temporal confidence filter
├── urdf/
│   └── vayu_robot.urdf.xacro  # TurtleBot3 Waffle Pi model
├── worlds/
│   └── campus_corridor.world  # L-shaped corridor environment
├── resource/
│   └── project_vayu
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## Prerequisites

- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy
- **Packages:**
  ```bash
  sudo apt install \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro
  ```
- **Python:** OpenCV with ArUco (`pip install opencv-contrib-python`)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select project_vayu --symlink-install
source install/setup.bash
```

## Quick Start — Simulation

Launch the full simulation (Gazebo + EKF + Nav2 + Vayu nodes):

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch project_vayu simulation.launch.py
```

The robot spawns at the **pickup zone** (origin) and waits for ArUco marker detections to decide its route through the L-shaped corridor to the **dropoff zone**.

### Launch Individual Components

```bash
# Nav2 only
ros2 launch project_vayu nav2.launch.py use_sim_time:=true

# Vayu nodes only
ros2 launch project_vayu vayu_nodes.launch.py use_sim_time:=true
```

## Sensor Configuration

| Sensor   | Topic              | Rate   | Frame                     |
|----------|--------------------|--------|---------------------------|
| Camera   | `/camera/image_raw`| 30 Hz  | `camera_rgb_optical_frame` |
| LiDAR    | `/scan`            | 5 Hz   | `base_scan`               |
| IMU      | `/imu/data`        | 200 Hz | `imu_link`                |
| Encoders | `/odom`            | 30 Hz  | `odom → base_footprint`   |

## FSM States (Semantic Social Awareness)

Our Finite State Machine integrates standard navigation with our custom "YIELD" logic for high-priority dynamic actors (e.g., emergency medical staff).

```
IDLE → NAVIGATING → DETECTING → ACTING → IDLE
          │             ↓
          │          STOPPED
          │
          └──► YIELD (Semantic Override) ──► NAVIGATING (Resume)

          any state ──emergency──► EMERGENCY_STOP
```

| Event                    | From       | To              | Action / Outcome                          |
|--------------------------|------------|-----------------|-------------------------------------------|
| `start_navigation`       | IDLE       | NAVIGATING      | Nav2 Action Client goal sent.             |
| `high_priority_actor_detected` | NAVIGATING | YIELD      | Preempt planner; publish zero `/cmd_vel`.|
| `actor_cleared`          | YIELD      | NAVIGATING      | Resume Nav2 goal tracking.                |
| `goal_reached`           | NAVIGATING | DETECTING       | Pause at junction; scan for ArUco.        |
| `symbol_detected`        | DETECTING  | ACTING          | Translate symbol to new directional goal. |
| `action_complete`        | ACTING     | IDLE            | Ready for next sequence.                  |
| `stop_requested`         | any        | STOPPED         | Controlled software halt.                 |
| `emergency`              | any        | EMERGENCY_STOP  | Immediate hardware/software lockout.      |

## Nav2 Tuning (DWB)

| Parameter            | Value                   |
|----------------------|-------------------------|
| `max_vel_x`          | 0.3 m/s                 |
| `max_vel_theta`      | 1.0 rad/s               |
| `min_turning_radius` | 0.0 (in-place rotation) |
| `sim_time`           | 1.7 s                   |
| `xy_goal_tolerance`  | 0.25 m                  |
| `yaw_goal_tolerance` | 0.25 rad                |

## EKF Configuration

- **Package:** `robot_localization`
- **Frequency:** 50 Hz
- **Mode:** `two_d_mode: true`
- **Sensors:**
  - Odom → position (x, y) + yaw rate
  - IMU → yaw + yaw rate + linear acceleration x

## Custom Message

```
# DetectedSymbol.msg
std_msgs/Header header
string          type        # e.g. "left_arrow", "forward"
float32         confidence  # [0.0, 1.0]
int32[4]        bbox        # [x_min, y_min, x_max, y_max]
bool            is_valid    # confidence gate result
```

## Gazebo World

The `campus_corridor.world` provides:
- **L-shaped corridor** (10 m + 8 m segments, 3 m width)
- **3 ArUco marker junctions** at walls
- **2 pedestrian actors** with looping walk trajectories
- **Pickup zone** (green, at origin)
- **Dropoff zone** (red, at corridor end)

## License

Apache-2.0
