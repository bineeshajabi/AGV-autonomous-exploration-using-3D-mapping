# frontier_explorer

Autonomous exploration for a TurtleBot3 waffle in Gazebo using RTAB-Map and Nav2. The robot maps an unknown environment on its own without any manual goal input.

Built as part of working toward autonomous warehouse exploration — using RTAB-Map for SLAM and Nav2 for navigation, with a custom frontier detection node that decides where the robot should go next.

---

## What it does

RTAB-Map builds a 2D occupancy grid in real time as the robot moves. The frontier explorer node reads that map, finds the boundary between explored and unexplored space, and sends the nearest unexplored region as a Nav2 goal. When the robot reaches it, the map grows and the process repeats until the space is fully mapped.


---

## How the frontier node works

The occupancy grid has three values per cell:
- `0` free space
- `100` obstacle  
- `-1` unknown

A frontier cell is a free cell (`0`) that has at least one unknown neighbour (`-1`). That's the edge of what the robot has seen so far.

The node:
1. Reads `/map` from RTAB-Map
2. Loops through every cell, flags frontier cells
3. Groups connected frontier cells into clusters using BFS
4. Converts the nearest cluster centroid to world coordinates
5. Sends it to Nav2's `navigate_to_pose` action server
6. Waits for the result, then picks the next frontier


---

## Stack

```
Gazebo (house world)
    ↓
TurtleBot3 waffle + depth camera + LIDAR
    ↓
RTAB-Map  —  fuses scan + depth, builds 2D occupancy grid
    ↓
frontier_explorer_node  —  reads map, picks next goal
    ↓
Nav2  —  drives robot to goal, avoids obstacles
    ↓
loop until fully mapped
```

---

## Run

**Terminal 1 — robot and world**
```bash
ros2 launch world_display house_bot.launch.py
```

**Terminal 2 — RTAB-Map**
```bash
ros2 launch obstacle_3d_world agv_rtab_map.launch.py
```

**Terminal 3 — Nav2**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

**Terminal 4 — frontier explorer**
```bash
ros2 run frontier_explorer frontier_explorer_node
```

Drive manually with teleop for the first few seconds to give RTAB-Map enough initial map data before the frontier node starts sending goals.

---

## Known issues

**Robot gets stuck** — the frontier centroid sometimes lands in an unreachable position (behind a shelf, too close to a wall). Nav2 fails the goal and the node picks the next frontier. Occasional manual teleop needed in tight spaces.

**Slow in large worlds** — RTAB-Map loop closure is compute-heavy. Reduce `Grid/RangeMax` if the system is struggling.

**Fix planned** — add a 60 second goal timeout so stuck goals are abandoned automatically instead of waiting for Nav2 to fail.

---

## What I learned building this

- How RTAB-Map builds a map — loop closure, graph optimization, what breaks in featureless environments
- How the occupancy grid is structured and how to read it in a ROS2 node
- BFS for clustering connected cells
- Nav2 action server interface — same pattern as Task 7 waypoint navigation but driven programmatically
- QoS mismatch between RTAB-Map (`TRANSIENT_LOCAL`) and default subscribers (`VOLATILE`) — cost a few hours of debugging
