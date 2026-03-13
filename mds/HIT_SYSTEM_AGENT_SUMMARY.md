# HIT System Agent Handoff Summary

## Purpose of this document

This file is a practical handoff summary of the **current integrated HIT navigation stack** in this workspace. It is meant for another coding/debugging agent that needs to understand how the system is wired, what each subsystem does, which files are authoritative, what data flows through ROS, and which recent fixes materially changed behavior.

This summary focuses on the code that is actually being used by the integrated launch path:

- `HIT_code/`
- `HIT_Integrated_test/`
- plus the directly referenced `DecisionNode/` MCU bridge workspace

---

## 1. High-level system overview

This system is a ROS1 Noetic autonomous navigation stack for a RoboMaster sentry robot. In the current integrated setup, it combines:

1. **Static map serving** from the HIT integration workspace
2. **LiDAR + localization** for live robot pose in the `map` frame
3. **Point-cloud filtering** for near-field obstacle perception
4. **Global path planning** using a topological graph planner on 2.5D map products
5. **Trajectory smoothing / reference generation** for the global route
6. **Local nonlinear MPC tracking** with obstacle constraints
7. **A velocity bridge** that converts HIT tracking output into `geometry_msgs/Twist`
8. **An MCU serial bridge** that sends navigation frames to the STM32 at fixed rate

In short:

**RViz goal / operator intent → global planner → reference trajectory → NMPC tracker → motion bridge → MCU serial frame**

---

## 2. Workspace layout and ownership

### Primary folders

- `HIT_code/sentry_planning_ws/`
  - HIT planning workspace
  - contains the global planner, local tracker, messages, waypoint tools, and OCS2 dependencies

- `HIT_Integrated_test/`
  - real-robot integration layer
  - provides the top-level launch file, workspace environment setup, and an overlay `sim_nav` workspace with helper nodes/scripts

- `DecisionNode/`
  - separate catkin workspace for MCU serial communication and decision-side I/O
  - currently used for `mcu_communicator`

- `RM_Sentry_2026/`
  - mirrored copy of the same stack for sync/deployment
  - when changing core planning/tracking logic, this mirror often needs to stay aligned

### Authoritative top-level runtime entrypoint

- `HIT_Integrated_test/HIT_intergration_test.launch`

### Authoritative environment setup

- `HIT_Integrated_test/hit_env.bash`

This script sources the integrated `sim_nav` overlay first, then adds `DecisionNode` to ROS path resolution. It is the preferred entrypoint for shell setup.

---

## 3. Runtime launch architecture

The integrated launch file is:

- `HIT_Integrated_test/HIT_intergration_test.launch`

It orchestrates the whole real-robot stack in this order.

### 3.1 Environment injection in launch

The launch file explicitly prepends:

- `DecisionNode/src`
- `HIT_code/sentry_planning_ws/src`

to `ROS_PACKAGE_PATH`, and adds `DecisionNode/devel` to `CMAKE_PREFIX_PATH`.

This is important because the integration launch is spanning **multiple workspaces**.

### 3.2 Launch sequence

1. **Map server**
   - include: `bot_sim/launch_real/map_server.launch`
   - current map_server YAML default inside that launch: `Mar13.yaml`

2. **IMU filter**
   - include: `bot_sim/launch_real/imu_filter.launch`

3. **Livox LiDAR driver**
   - include: `livox_ros_driver2/launch_ROS1/msg_mixed.launch`

4. **Localization**
   - include: `hdl_localization/launch/hdl_localization.launch`
   - gets `points_topic` from launch arg, default `/filted_topic_3d`

5. **Real robot TF bridge**
   - include: `bot_sim/launch_real/real_robot_transform.launch`

6. **Point-cloud filtering**
   - include: `bot_sim/launch_real/lidar_filter_pointcloud.launch`
   - produces filtered obstacle cloud for HIT planning

7. **Legacy D* Lite branch**
   - optional, disabled by default
   - `enable_dstarlite:=false`

8. **HIT global planner**
   - include: `trajectory_generation/launch/global_searcher.launch`

8.1 **Frame normalization / rebasing bridge**
   - node: `bot_sim/local_frame_bridge.py`
   - outputs planner-local topic names like `/odom_local`, `/clicked_point`, `/aligned_points_local`, `/filted_topic_3d_local`

9. **HIT local tracker / MPC**
   - include: `tracking_node/launch/trajectory_planning.launch`

9.5 / 9.6 / 9.7 **Map image publishers for RViz**
   - `bev_image_publisher`
   - `occ_image_publisher`
   - `topo_image_publisher`

10. **Velocity bridge to MCU bridge**
   - `tracking_node/hit_bridge`
   - converts `/sentry_des_speed` → `/cmd_vel`

10b. **MCU serial bridge**
   - `decision_node/mcu_communicator`
   - converts `/cmd_vel` + state flags into serial `NavigationFrame` (`0x93`)

10c. **Legacy ser2msg fallback**
   - available but disabled by default

---

## 4. Current launch defaults that matter

From `HIT_Integrated_test/HIT_intergration_test.launch`:

- `enable_rviz=false`
- `enable_ser2msg=false`
- `enable_real_robot_transform=true`
- `enable_dstarlite=false`
- `enable_mcu_communicator=true`
- `serial_port=/dev/ttyUSB0`
- `serial_baudrate=115200`
- `use_omega_output=false`

### Important current motion setting

`use_omega_output=false` means the system is currently in **OMNI mode** by default.

That means the active command chain is:

- tracking computes desired heading + speed
- bridge decomposes speed into `vx/vy`
- MCU receives strafing-capable chassis-frame commands

---

## 5. End-to-end ROS data flow

## 5.1 Pose / goal / cloud normalization flow

### Inputs

- `/odom`
- `/move_base_simple/goal`
- `/aligned_points`
- `/filted_topic_3d`

### `local_frame_bridge.py` outputs

- `/odom_local`
- `/clicked_point`
- `/aligned_points_local`
- `/filted_topic_3d_local`

### Current behavior of `local_frame_bridge.py`

The script supports coordinate rebasing/recentering, but in the integrated launch it is configured with:

- `center_on_first_odom=false`
- `passthrough=true`

So in the current deployment, it is effectively being used as a **topic normalization / compatibility bridge**, not as an active local-frame recentering transform.

It still provides the topic names the HIT planners expect.

## 5.2 Planning flow

### Goal input

A goal from RViz arrives on `/move_base_simple/goal`.

`local_frame_bridge.py` converts it to `/clicked_point`.

`trajectory_generation` subscribes to `/clicked_point` inside `ReplanFSM` and triggers a new global plan.

### Global planner output

The global planner publishes:

- `/trajectory_generation/global_trajectory` (`trajectory_generation/trajectoryPoly`)
- visualization markers / point clouds
- `/target_result`
- `/xtl_flag`

### Local tracker input and output

The local planner subscribes to:

- `/trajectory_generation/global_trajectory`

The tracker then publishes:

- `/sentry_des_speed` (`sentry_msgs/slaver_speed`)
- `/replan_flag`
- `/solver_status`
- `/robot_cur_yaw_reg`
- tracking visualization topics

### Motion bridge flow

`hit_bridge` subscribes to:

- `/sentry_des_speed`

and publishes:

- `/cmd_vel`
- `/dstar_status` (used here as an arrival flag)

### MCU bridge flow

`mcu_communicator` subscribes to:

- `/cmd_vel`
- `/dstar_status`
- `/nav_received`
- plus decision-side control topics such as `/motion`, `/recover`, `/bullet_up`, `/bullet_num`

It sends the resulting navigation command to the MCU as serial frame `0x93` at a fixed timer rate.

---

## 6. Core subsystem details

## 6.1 Integration layer (`HIT_Integrated_test/`)

### Key files

- `HIT_Integrated_test/HIT_intergration_test.launch`
  - top-level orchestration

- `HIT_Integrated_test/hit_env.bash`
  - recommended environment source script

- `HIT_Integrated_test/sim_nav/src/bot_sim/scripts/local_frame_bridge.py`
  - rebases or passes through odom/goal/cloud topics into planner-facing local topics

- `HIT_Integrated_test/sim_nav/src/bot_sim/scripts/map_image_publisher.py`
  - republishes PNG map products as `nav_msgs/OccupancyGrid` for RViz

- `HIT_Integrated_test/sim_nav/src/bot_sim/launch_real/lidar_filter_pointcloud.launch`
  - 3D point cloud obstacle filtering around the robot

- `HIT_Integrated_test/sim_nav/src/bot_sim/launch_real/real_robot_transform.launch`
  - real robot TF wiring; currently also publishes a static transform from `aft_mapped` to `base_link`

### Why this layer exists

The HIT planning stack was originally its own planning workspace. This integration layer adapts it to the real robot’s live localization, topic names, TF frames, map server, and serial communication chain.

---

## 6.2 Global planning (`trajectory_generation`)

### Key files

- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/launch/global_searcher.launch`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/trajectory_generator_node.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/replan_fsm.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/plan_manager.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/TopoSearch.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/Astar_searcher.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/path_smooth.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/reference_path.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/RM_GridMap.cpp`

### What it does

This subsystem takes the current robot pose and clicked target, performs topological global search on a 2.5D map, prunes/smooths the resulting path, and generates a reference trajectory for the local MPC tracker.

### Internal architecture

#### `ReplanFSM`

Responsible for:

- receiving target points from `/clicked_point`
- receiving odometry from `/odometry_imu` / gazebo
- deciding when to generate a new plan or replan
- publishing the final `trajectoryPoly` message

#### `planner_manager`

Owns:

- `GlobalMap`
- `AstarPathFinder`
- `TopoSearcher`
- `Smoother`
- `Refenecesmooth`

It handles:

- full global planning
- local replan patching when a global path segment becomes blocked
- reference trajectory generation

### Current planner inputs

In `global_searcher.launch`, planner-facing topics are remapped to the normalized `*_local` names:

- `/odometry_imu` → `/odom_local`
- `/cloud_registered_world` → `/aligned_points_local`
- `/realsense_pointcloud` → `/aligned_points_local`
- `/mbot/velodyne_points` → `/filted_topic_3d_local`

### Current planner map inputs

The global planner loads:

- `occfinal.png`
- `bevfinal.png`
- `occtopo.png`
- `map_meta.yaml`

from `trajectory_generation/map/`.

---

## 6.3 Local tracking / NMPC (`tracking_node`)

### Key files

- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/launch/trajectory_planning.launch`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/tracking_node.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/tracking_manager.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/local_planner.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/visualization_utils.cpp`
- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/hit_bridge.cpp`

### What it does

This subsystem subscribes to the global trajectory, constructs an MPC reference horizon, adds obstacle constraints, solves NMPC using OCS2 SQP, and emits motion commands in the HIT `slaver_speed` format.

### Model used by NMPC

The tracker uses a simplified mobile robot model with state roughly:

- $x$
- $y$
- $v$
- $\theta$

and control input roughly:

- acceleration $a$
- angular velocity $\omega$

### Important current local planner behavior

- It subscribes to `/trajectory_generation/global_trajectory`
- It constructs a horizon of 20 steps with `dt = 0.1`
- It keeps static obstacle constraints active even when no dynamic obstacle is nearby
- It uses a shared obstacle-constraint object instead of rebuilding the solver every cycle
- It tunes HPIPM into a more robust setting at runtime

### Important current tracking outputs

- `/sentry_des_speed`
- `/solver_status`
- `/replan_flag`
- `/tracking/mpc_predicted_path`
- `/tracking/mpc_reference_path`
- marker topics for candidate path, reference path, obstacle centers

---

## 6.4 Motion bridge (`hit_bridge`)

### Key file

- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/hit_bridge.cpp`

### Role

Converts the HIT tracker’s custom `sentry_msgs/slaver_speed` into a standard `geometry_msgs/Twist` that the MCU bridge can consume.

### Supported modes

#### OMNI mode (`use_omega_output=false`, current default)

- used for the old omnidirectional platform
- bridge computes:
  - `delta = angle_target - angle_current`
  - `vx = line_speed * cos(delta)`
  - `vy = line_speed * sin(delta)`
  - `angular.z = 0`

#### OMEGA mode (`use_omega_output=true`)

- used for a differential / unicycle-style platform
- bridge computes:
  - `vx = line_speed`
  - `vy = 0`
  - `angular.z = angle_target` where `angle_target` carries $\omega$

### Critical semantic clarification

**In the current code path, OMNI mode `vx` and `vy` are in chassis body coordinates, not gimbal coordinates.**

This is because `tracking_manager.cpp` now packs:

- `angle_target = desired heading`
- `angle_current = current chassis yaw`

so `delta` is effectively the desired heading error in the chassis frame.

### Important note about comments

There is an older explanatory comment in `hit_bridge.cpp` that still describes `angle_current` as LiDAR/gimbal yaw. That comment is stale relative to the active packing logic in `tracking_manager.cpp`. The **actual runtime semantics** are defined by `tracking_manager.cpp`, and currently use **chassis yaw**.

---

## 6.5 MCU serial bridge (`DecisionNode`)

### Key files

- `DecisionNode/src/decision_node/src/mcu_communicator.cpp`
- `DecisionNode/src/decision_node/include/decision_node/mcu_comm.hpp`
- `DecisionNode/src/decision_node/MCU_COMMUNICATOR_README.md`

### Role

This node owns the serial port and bridges ROS messages to and from the MCU.

It does two main jobs:

1. **Receives MCU state frames** and republishes them as ROS topics
2. **Sends navigation frames** to the MCU at fixed frequency

### Navigation command path

`mcu_communicator` subscribes to `/cmd_vel`, caches:

- `linear.x` as `vx`
- `linear.y` as `vy`
- `angular.z` as `z_angle`

and sends them in a `NavigationFrame`:

```c
struct NavigationFrame {
    uint8_t sof;      // 0x93
    float vx;
    float vy;
    float z_angle;
    uint8_t received;
    uint8_t arrived;
    uint8_t crc8;
    uint8_t eof;      // 0xFE
}
```

The frame is transmitted at a timer-controlled frequency, currently default `50 Hz`.

### Receive side

The MCU receive frame is `0x91` and includes:

- gimbal yaw
- chassis imu yaw
- robot/game status
- HP
- ammo
- occupy state
- enemy coordinates
- suggested radar target

and more.

The communicator republishes this onto ROS topics such as:

- `/mcu/yaw_angle`
- `/mcu/chassis_imu`
- `/referee/*`
- `/robot/*`
- `/enemy/*`
- `/radar/*`

---

## 7. Maps and map generation

## 7.1 Planner map products

The HIT planner consumes three PNG map products plus metadata:

- `occfinal.png`
  - occupancy map
- `bevfinal.png`
  - height map encoded into grayscale
- `occtopo.png`
  - topo/skeleton helper map
- `map_meta.yaml`
  - resolution and origin metadata

These live in:

- `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/map/`

## 7.2 Single source of truth for map geometry

The launch files now read map dimensions and origin directly from `map_meta.yaml` using `$(eval ...)`.

That means:

- map resolution
- map size
- map lower-left origin

are centrally defined in `map_meta.yaml`, instead of being hardcoded separately in multiple launch files.

## 7.3 Map generation tool

### Key file

- `HIT_code/sentry_planning_ws/tools/pcd_to_maps.py`

### What it does

Given a `.pcd`, it generates:

- `occ.png`
- `bev.png`
- `occtopo.png`
- `map_meta.yaml`

and, by default, deploys them as:

- `occfinal.png`
- `bevfinal.png`
- `occtopo.png`
- `map_meta.yaml`

to the planner map directory.

### Important notes

- It no longer patches XML launch files
- launch files are expected to consume `map_meta.yaml` directly
- this is now the correct workflow for changing maps

## 7.4 RViz visualization of maps

`map_image_publisher.py` republishes the PNGs as `nav_msgs/OccupancyGrid`.

Current visualization nodes in the integration launch:

- `bev_image_publisher`
- `occ_image_publisher`
- `topo_image_publisher`

This allows fast sanity checking of all three planner maps in RViz.

---

## 8. Key ROS interfaces

## 8.1 Important custom messages

### `sentry_msgs/slaver_speed`

```text
float32 line_speed
float32 angle_target
float32 angle_current
uint8 xtl_flag
uint8 in_bridge
```

Runtime meaning depends on output mode:

- **OMNI mode**: desired heading, current chassis yaw, speed magnitude
- **OMEGA mode**: angular velocity, unused, forward speed

### `trajectory_generation/trajectoryPoly`

```text
time start_time
uint8 motion_mode
float32[] coef_x
float32[] coef_y
float32[] duration
```

This is the global reference trajectory message consumed by the tracker.

### Other custom messages in active use

- `sentry_msgs/RobotStatus`
- `sentry_msgs/RobotsHP`

These are used for team-color inference, HP-triggered behavior changes, and special motion mode logic.

---

## 8.2 Important topics by stage

### Perception / localization

- `/odom`
- `/odometry_imu`
- `/aligned_points`
- `/filted_topic_3d`
- `/mcu/yaw_angle`
- `/mcu/chassis_imu`

### Normalized planner-facing topics

- `/odom_local`
- `/clicked_point`
- `/aligned_points_local`
- `/filted_topic_3d_local`

### Global planning

- `/trajectory_generation/global_trajectory`
- `/target_result`
- `/xtl_flag`
- `grid_map_vis`
- `local_grid_map_vis`
- topo/reference visualization topics

### Local tracking

- `/sentry_des_speed`
- `/solver_status`
- `/replan_flag`
- `/tracking/mpc_predicted_path`
- `/tracking/mpc_reference_path`

### Motion / MCU

- `/cmd_vel`
- `/dstar_status`
- `/nav_received`
- `/motion`
- `/recover`
- `/bullet_up`
- `/bullet_num`

---

## 9. Current planning and control logic in practical terms

## 9.1 Global planning logic

The global planner is topological rather than pure A*.

The code builds a graph of:

- guard nodes
- connection nodes

using map keypoints and sampled points, then performs graph search, then prunes/smooths the path into a usable reference route.

This architecture is intended to handle:

- multiple viable routes
- narrow passages
- ramps / stairs / bridge-like terrain features
- 2.5D height-aware visibility

## 9.2 Trajectory smoothing and reference generation

After topo search, the route is:

1. pruned
2. smoothed
3. resampled
4. converted into reference trajectory coefficients and durations

This reference is what the local MPC follows.

## 9.3 Local MPC logic

The local tracker:

1. receives global trajectory coefficients
2. builds a short horizon around the current time
3. derives reference pose/velocity samples
4. collects relevant obstacle points from the map and local cloud
5. solves OCS2 SQP MPC
6. emits command output in HIT message format

## 9.4 Replanning logic

The system supports multiple replan layers:

- continue tracking if safe
- do local patch planning if a segment becomes blocked
- do global replanning if local patch fails

This helps avoid unnecessary full replans while still recovering from blockage.

---

## 10. Important recent fixes and their impact

This section is especially important for another agent, because these fixes changed real robot behavior.

## 10.1 Map metadata now drives launch configuration

### Problem

Map origin / dimensions were previously easy to desynchronize across launch files.

### Current state

Launch files read `map_meta.yaml` directly via `$(eval ...)`.

### Effect

The deployed map PNGs and the planner’s geometric parameters now stay aligned.

---

## 10.2 `pcd_to_maps.py` no longer edits launch files

### Problem

Map generation and launch patching were coupled.

### Current state

The tool now only generates/deploys map files and metadata.

### Effect

Map conversion is cleaner and less fragile.

---

## 10.3 Local obstacle inflation now respects actual map resolution

### Problem

A hardcoded inflation radius in cells caused wrong physical inflation when map resolution changed.

### Current state

`LocalPlanner::swellOccMap()` computes inflation cells from:

- actual map resolution
- robot radius

### Effect

Obstacle inflation scales correctly with map resolution.

---

## 10.4 MPC no longer drops wall constraints when dynamic obstacles are absent

### Problem

The original local-planner logic could clear obstacle constraints if no unknown/dynamic obstacle was near the horizon.

### Effect of bug

The MPC became effectively blind to static walls.

### Current state

Static obstacle constraints are always retained.

### Result

Wall avoidance is materially more reliable.

---

## 10.5 OCS2 solver lifetime and robustness fixes

### Problem 1

Recreating the solver repeatedly could cause instability / thread lifetime issues.

### Current state

The MPC solver is now created once and reused.

### Problem 2

HPIPM default settings were too brittle for this workload.

### Current state

The code sets HPIPM to a more robust mode at runtime.

### Result

More stable NMPC behavior and fewer solver-related crashes/failures.

---

## 10.6 Gimbal/chassis yaw mismatch was fixed in OMNI output packing

### Problem

`vx/vy` decomposition used the wrong yaw reference when the gimbal rotated relative to the chassis.

### Current state

`tracking_manager.cpp` now packs the **chassis yaw** into `angle_current` for OMNI mode.

### Result

The output `vx/vy` are now consistent with the chassis frame, avoiding wrong-direction motion and oscillation when the gimbal moves.

---

## 10.7 Topological planner connectivity fallback was added

### Problem

Dijkstra could fail because start/end were in disconnected graph components even though a practical forced connection was available.

### Current state

After failure, the planner can run a BFS-style connectivity fallback and force-connect end/start to reachable guards when visibility permits.

### Result

Reduced incidence of `No Path` failures caused by graph disconnection.

---

## 10.8 Topo graph start/end `z` is now overridden to BEV height

### Problem

SLAM odometry `z` drifted away from the BEV map’s height convention.

This caused the topological visibility test to reject start-node connections after SLAM stabilized, because the height difference threshold was exceeded immediately.

### Current state

`TopoSearcher::createGraph()` and `createLocalGraph()` now override start/end `z` with `GlobalMap::getHeight()` from the BEV map.

### Result

Start/end nodes are height-consistent with the topo graph, restoring connectivity after SLAM settles.

---

## 10.9 Topo visibility now ignores dynamic occupancy for static graph construction

### Problem

The topo graph visibility check was using occupancy logic that also considered dynamic/local point-cloud obstacles.

### Effect of bug

Real-time obstacles could incorrectly block construction of the static topological graph.

### Current state

A static-only occupancy API was added:

- `GlobalMap::isStaticOccupied()`

and `TopoSearcher::lineVisib()` now uses static occupancy only.

### Result

The static topo graph is no longer destroyed by transient local obstacles.

---

## 10.10 Tracking visualization was improved

### Current additions

The tracker now publishes:

- `/tracking/mpc_predicted_path`
- `/tracking/mpc_reference_path`

in `nav_msgs/Path`, in addition to existing marker-based visualization.

### Result

It is easier to inspect MPC behavior in RViz.

---

## 11. Practical semantics of motion commands

This section is intentionally explicit because it is easy to get wrong.

## 11.1 Current default: OMNI mode

Because `use_omega_output=false` in the integration launch, the active output path is:

1. `tracking_manager` computes:
   - desired heading
   - current chassis yaw
   - speed magnitude

2. `publishSentryOptimalSpeed()` writes them into `slaver_speed`

3. `hit_bridge` computes:
   - `delta = angle_target - angle_current`
   - `vx = v * cos(delta)`
   - `vy = v * sin(delta)`

4. `mcu_communicator` sends:
   - `vx = twist.linear.x`
   - `vy = twist.linear.y`
   - `z_angle = twist.angular.z`

## 11.2 Coordinate frame meaning

In this default mode:

- `vx` = chassis forward-axis component
- `vy` = chassis lateral-axis component
- these are **chassis body-frame** commands
- they are **not gimbal-frame** commands

## 11.3 OMEGA mode alternative

If `use_omega_output=true`:

- `vx = forward speed`
- `vy = 0`
- `z_angle = angular velocity`

This is for a non-strafing platform.

---

## 12. Current map / frame assumptions

### TF and frame assumptions used by the integrated stack

- planner outputs and RViz overlays are in `map`
- `local_frame_bridge.py` republishes normalized planner topics with frame `map`
- `real_robot_transform.launch` introduces `aft_mapped`, `base_link`, and related static/dynamic transforms
- the obstacle filter launch uses:
  - `base_frame = gimbal_frame`
  - `laser_frame = aft_mapped`

### Operational takeaway

When debugging planning or RViz alignment issues, check:

1. map origin from `map_meta.yaml`
2. `map_server` YAML alignment
3. `odom` / `odom_local` consistency
4. `aligned_points_local` and `filted_topic_3d_local` frame IDs
5. TF between `map`, `aft_mapped`, `base_link`, and gimbal-related frames

---

## 13. Known file guide for common tasks

## 13.1 If goal clicks do nothing

Check:

- `HIT_Integrated_test/sim_nav/src/bot_sim/scripts/local_frame_bridge.py`
- `trajectory_generation/src/replan_fsm.cpp`
- `/move_base_simple/goal`
- `/clicked_point`

## 13.2 If global planning says no path

Check:

- `trajectory_generation/src/TopoSearch.cpp`
- `trajectory_generation/src/plan_manager.cpp`
- `trajectory_generation/src/RM_GridMap.cpp`
- planner map files in `trajectory_generation/map/`

Especially inspect:

- BEV height consistency
- topo PNG correctness
- start/end occupancy
- forced-connect fallback behavior

## 13.3 If robot tracks the path badly or oscillates

Check:

- `trajectory_tracking/src/tracking_manager.cpp`
- `trajectory_tracking/src/local_planner.cpp`
- `trajectory_tracking/src/hit_bridge.cpp`

Especially inspect:

- `use_omega_output`
- yaw source used for decomposition
- solver status on `/solver_status`
- predicted/reference RViz paths

## 13.4 If the MCU gets wrong motion values

Check:

- `trajectory_tracking/src/hit_bridge.cpp`
- `DecisionNode/src/decision_node/src/mcu_communicator.cpp`
- `DecisionNode/src/decision_node/include/decision_node/mcu_comm.hpp`

Trace:

- `/sentry_des_speed`
- `/cmd_vel`
- serial `NavigationFrame`

## 13.5 If map alignment is wrong after changing map files

Check:

- `HIT_code/sentry_planning_ws/tools/pcd_to_maps.py`
- `trajectory_generation/map/map_meta.yaml`
- `occfinal.png`
- `bevfinal.png`
- `occtopo.png`
- `HIT_Integrated_test/HIT_intergration_test.launch`
- `trajectory_generation/launch/global_searcher.launch`
- `trajectory_tracking/launch/trajectory_planning.launch`

---

## 14. Important implementation notes and quirks

1. **`global_searcher.launch` currently starts the global planner with a `gdb` batch launch-prefix.**
   - This is likely intended for automatic backtraces on crash.
   - It is unusual and worth remembering if startup or timing looks odd.

2. **The top-level system spans multiple workspaces.**
   - `sim_nav` overlay
   - `sentry_planning_ws`
   - `DecisionNode`

3. **The map server YAML and the planner PNG maps are separate assets.**
   - They must remain aligned in origin, scale, and interpretation.

4. **Legacy components still exist but are not the default path.**
   - D* Lite launch branch
   - `ser2msg` bridge path

5. **The duplicate mirror tree under `RM_Sentry_2026/` may need manual sync.**
   - Do not assume changes in `HIT_code/` automatically propagate there.

---

## 15. Suggested mental model for the next agent

If you need to understand the active robot stack quickly, think of it as three layers:

### Layer A: integration and sensor normalization

`HIT_Integrated_test/` + `sim_nav/`

This layer makes localization, filtered clouds, map serving, and topic names look the way the HIT planners expect.

### Layer B: planning and tracking intelligence

`HIT_code/sentry_planning_ws/src/sentry_planning/`

This layer performs:

- topological global path generation
- trajectory smoothing/reference generation
- local NMPC tracking and avoidance

### Layer C: hardware egress

`hit_bridge` + `DecisionNode/mcu_communicator`

This layer converts planner output into:

- ROS `Twist`
- serial navigation frames to the MCU

That separation is the cleanest way to reason about where a failure is coming from.

---

## 16. Minimal file shortlist to read first

If another agent only has time to inspect a handful of files, start here in this order:

1. `HIT_Integrated_test/HIT_intergration_test.launch`
2. `HIT_Integrated_test/hit_env.bash`
3. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/launch/global_searcher.launch`
4. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/launch/trajectory_planning.launch`
5. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/replan_fsm.cpp`
6. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/plan_manager.cpp`
7. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/TopoSearch.cpp`
8. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/tracking_manager.cpp`
9. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/local_planner.cpp`
10. `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/hit_bridge.cpp`
11. `DecisionNode/src/decision_node/src/mcu_communicator.cpp`
12. `HIT_code/sentry_planning_ws/tools/pcd_to_maps.py`

---

## 17. Final current-state summary

The current integrated system is a **real-robot HIT planning stack running on ROS1**, with:

- map-driven launch geometry from `map_meta.yaml`
- normalized planner topics via `local_frame_bridge.py`
- topological 2.5D global planning
- smoothed reference trajectory generation
- OCS2 SQP-based local MPC
- chassis-frame OMNI velocity decomposition by default
- serial MCU output through `DecisionNode/mcu_communicator`
- recent fixes for wall avoidance, yaw-frame consistency, topo connectivity, topo height consistency, and visualization

This is the operational baseline another agent should assume unless a later change explicitly says otherwise.
