# Checkpoint 9 - Advanced ROS 2

Two C++ ROS 2 packages that make an **RB1 robot** autonomously approach and attach to a shelf in a warehouse simulation. The robot uses **laser intensity detection** to find the shelf's reflective legs, publishes a **TF2 `cart_frame`** at the midpoint between them, and drives underneath the shelf using proportional control before lifting it with the elevator.

Part of the [ROS & ROS 2 Developer Master Class](https://www.theconstructsim.com/) certification (Phase 2).

<p align="center">
  <img src="media/attach_shelf.gif" alt="RB1 robot approaching and attaching to the shelf" width="550"/>
</p>

## How It Works

<p align="center">
  <img src="media/cart_frame.png" alt="RViz showing cart_frame TF published between the two shelf legs" width="550"/>
</p>

### Pre-Approach Phase

1. Robot drives forward at 0.2 m/s toward the shelf
2. Front laser reading drops below `obstacle` parameter threshold — robot stops
3. Waits 0.5 s for stabilization, captures current yaw from odometry
4. Rotates by `degrees` parameter (odometry-based yaw tracking with angle normalization)

### Final Approach Phase

1. `pre_approach_v2` calls the `/approach_shelf` service after rotation completes
2. The service server scans laser intensities for two high-intensity blobs (> 1000) — the shelf's reflective leg markers
3. Computes the midpoint between the two legs in the laser frame
4. Transforms the midpoint to the `odom` frame using TF2 and publishes it as a static `cart_frame`
5. Drives toward `cart_frame` with a +0.5 m offset (to position the robot under the shelf)
6. Stops when within 1.5 cm of the target, publishes `"up"` to `/elevator_up` to lift the shelf

## Tasks Breakdown

### Task 1 - Pre-Approach Node (`attach_shelf`, tag: `pre_approach`)

- Created `pre_approach.cpp` with ROS 2 parameters (`obstacle`, `degrees`)
- Subscribes to `/scan` (front laser) and `/diffbot_base_controller/odom` (yaw)
- State machine: forward drive → obstacle stop → rotation → shutdown
- XML launch file `pre_approach.launch.xml` with configurable parameters

### Task 2 - Service-Based Approach (`attach_shelf`, tag: `attach_to_shelf`)

- Created `approach_service_server.cpp` implementing the `/approach_shelf` service
- Laser intensity blob detection (threshold > 1000) to find two shelf legs
- TF2 coordinate transform from `robot_front_laser_base_link` to `odom` frame
- Static TF broadcast of `cart_frame` at the midpoint between shelf legs
- Proportional approach controller using `lookupTransform` from `robot_base_link` to `cart_frame`
- Publishes `"up"` to `/elevator_up` on arrival
- Created `pre_approach_v2.cpp` extending Task 1 with a service client (`final_approach` parameter)
- Python launch file `attach_to_shelf.launch.py` launching both nodes + RViz
- Custom service: `GoToLoading.srv` (`bool attach_to_shelf` → `bool complete`)

### Task 3 - ROS 2 Components (`my_components`)

- Refactored all nodes as **composable components** (shared library plugins)
- `PreApproach`, `AttachServer`, `AttachClient` registered with `RCLCPP_COMPONENTS_REGISTER_NODE`
- `ComposableNodeContainer` launch file loads `PreApproach` + `AttachServer` into a single process
- Manual composition executable (`manual_composition.cpp`) using `SingleThreadedExecutor`
- Header/source separation with `visibility_control.h` for DLL export macros

## ROS 2 Interface

| Name | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` (sub) | Laser scanner data (ranges + intensities) |
| `/diffbot_base_controller/odom` | `nav_msgs/Odometry` (sub) | Robot odometry for yaw tracking |
| `/diffbot_base_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` (pub) | Velocity commands to the robot |
| `/elevator_up` | `std_msgs/String` (pub) | Elevator lift command (`"up"`) |
| `/approach_shelf` | `GoToLoading` (service) | Triggers shelf detection and final approach |
| `odom` → `cart_frame` | TF2 (static broadcast) | Midpoint between the two shelf legs |

## Project Structure

```
checkpoint9/
├── attach_shelf/                    # Standard ROS 2 nodes
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   ├── pre_approach.launch.xml        # Task 1 launch (XML)
│   │   └── attach_to_shelf.launch.py      # Task 2 launch (Python)
│   ├── rviz/
│   ├── src/
│   │   ├── pre_approach.cpp               # Task 1: drive + rotate
│   │   ├── pre_approach_v2.cpp            # Task 2: drive + rotate + service call
│   │   └── approach_service_server.cpp    # Task 2: shelf detection + approach
│   └── srv/
│       └── GoToLoading.srv
├── my_components/                   # ROS 2 composable nodes
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/my_components/
│   │   ├── pre_approach.hpp
│   │   ├── attach_server.hpp
│   │   ├── attach_client.hpp
│   │   └── visibility_control.h
│   ├── launch/
│   │   └── attach_to_shelf.launch.py      # Component container launch
│   ├── rviz/
│   ├── src/
│   │   ├── pre_approach.cpp               # PreApproach component
│   │   ├── attach_server.cpp              # AttachServer component
│   │   ├── attach_client.cpp              # AttachClient component
│   │   └── manual_composition.cpp         # Manual composition executable
│   └── srv/
│       └── GoToLoading.srv
└── media/
```

## How to Use

### Prerequisites

- ROS 2 Humble
- Gazebo Classic 11
- RB1 robot simulation packages (`rb1_ros2_description`)

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select attach_shelf my_components
source install/setup.bash
```

### Task 1 - Pre-Approach Only

```bash
# Launch the RB1 simulation first, then:
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.3 degrees:=-90
```

### Task 2 - Full Shelf Attachment (Standard Nodes)

```bash
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.3 degrees:=-90 final_approach:=true
```

### Task 3 - Full Shelf Attachment (Components)

```bash
ros2 launch my_components attach_to_shelf.launch.py
```

## Git Tags

| Tag | Description |
|---|---|
| `pre_approach` | Pre-approach node with parameterized drive and rotation |
| `attach_to_shelf` | Service-based final approach with TF2, laser intensity detection, and elevator lift |

## Key Concepts Covered

- **Laser intensity detection**: finding reflective markers via `intensities[]` blob analysis
- **TF2 static broadcasting**: publishing `cart_frame` at runtime from computed coordinates
- **TF2 coordinate transforms**: `doTransform` from laser frame to odom frame
- **ROS 2 services**: custom `GoToLoading.srv` for decoupled approach triggering
- **ROS 2 parameters**: runtime-configurable `obstacle`, `degrees`, `final_approach`
- **State machines**: sequential phases (drive → stop → rotate → service call → approach → lift)
- **ROS 2 composition**: `rclcpp_components` shared libraries, `ComposableNodeContainer`, manual composition
- **Proportional control**: distance/yaw-based velocity commands with clamping

## Technologies

- ROS 2 Humble
- C++ 17
- TF2 (`tf2_ros`, `tf2_geometry_msgs`)
- `rclcpp_components` (node composition)
- Gazebo Classic 11
