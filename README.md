# TurtleBot3 Navigation Workspace

ROS2 Humble workspace for autonomous navigation with TurtleBot3 Waffle in Gazebo simulation. Covers SLAM mapping, Nav2 autonomous navigation, programmatic waypoint following, and reactive obstacle avoidance.

## Packages

### tb3_bringup
Launch files, configuration, and maps for bringing up the full navigation stack.

- **Launch files:** SLAM mapping (`slam.launch.py`), Nav2 navigation (`navigation.launch.py`)
- **Maps:** Pre-built occupancy grid maps for `turtlebot3_world`
- **Config:** Nav2 parameter file (`nav2_params.yaml`) — costmaps, planner, controller, AMCL, behavior server
- **RViz:** Custom visualization configs

### tb3_navigation
Custom navigation nodes implementing different ROS2 patterns.

| Node | Pattern | Description |
|------|---------|-------------|
| `teleop_node` | Timer + raw terminal I/O | Keyboard teleoperation using non-blocking `termios` input. WASD controls with real-time velocity publishing |
| `waypoint_follower` | Script (nav2_simple_commander) | Sequentially navigates through predefined waypoints using `BasicNavigator` API. Sets initial pose, waits for Nav2, iterates goals |
| `obstacle_avoidance` | Subscriber callback | Reactive obstacle avoidance using LaserScan data. Splits 360° scan into front/left/right regions, decides turn direction based on minimum distances |

## Architecture

```
                    ┌─────────────┐
                    │   Gazebo     │
                    │  Simulation  │
                    └──────┬──────┘
                           │
                    /scan  │  /odom
                           │
          ┌────────────────┼────────────────┐
          │                │                │
    ┌─────▼─────┐   ┌─────▼─────┐   ┌─────▼──────┐
    │ slam_     │   │   Nav2    │   │ obstacle_  │
    │ toolbox   │   │  Stack    │   │ avoidance  │
    │ (SLAM)    │   │ (Nav)     │   │ (Reactive) │
    └─────┬─────┘   └─────┬─────┘   └─────┬──────┘
          │               │               │
       /map          /cmd_vel          /cmd_vel
```

## Prerequisites

- ROS2 Humble
- TurtleBot3 packages (`turtlebot3_gazebo`, `turtlebot3_navigation2`)
- Nav2 (`nav2_bringup`, `nav2_simple_commander`)
- slam_toolbox

```bash
export TURTLEBOT3_MODEL=waffle
```

## Build & Run

```bash
cd ~/tb3_nav_ws
colcon build
source install/setup.bash
```

### SLAM Mapping
```bash
ros2 launch tb3_bringup slam.launch.py
# Drive around with teleop, then save map:
ros2 run nav2_map_server map_saver_cli -f ~/tb3_nav_ws/maps/tb3_world_map
```

### Autonomous Navigation
```bash
ros2 launch tb3_bringup navigation.launch.py
# In RViz: Set 2D Pose Estimate → Set Nav2 Goal
```

### Waypoint Following
```bash
ros2 launch tb3_bringup navigation.launch.py
# In another terminal:
ros2 run tb3_navigation waypoint_follower
```

### Obstacle Avoidance
```bash
ros2 launch tb3_bringup slam.launch.py
# In another terminal:
ros2 run tb3_navigation obstacle_avoidance
```

## Key Concepts Implemented

- **SLAM:** Simultaneous Localization and Mapping with slam_toolbox (online async mode)
- **Localization:** AMCL particle filter with manual initial pose estimation
- **Navigation:** Nav2 stack — static/obstacle/inflation costmap layers, NavFn planner, DWB controller
- **Reactive Control:** Real-time LaserScan processing for obstacle detection and avoidance
- **Sim Time:** All nodes synchronized with Gazebo simulation clock (`use_sim_time: true`)

## Environment

| Component | Detail |
|-----------|--------|
| ROS2 | Humble |
| Robot | TurtleBot3 Waffle |
| Simulator | Gazebo Classic |
| World | turtlebot3_world |
| Build | ament_python |
| License | Apache-2.0 |