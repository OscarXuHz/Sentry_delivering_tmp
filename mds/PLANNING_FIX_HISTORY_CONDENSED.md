# Sentry Planning System — Fix History (Condensed)

> **System:** ROS Noetic, OCS2 SQP-MPC (HPIPM), MINCO trajectory optimization  
> **State:** `[x, y, v, φ]` (4D), **Input:** `[accel, ω]` (2D)  
> **Map:** 0.05 m/cell, 20×20 m, BEV height map, robot_radius=0.35m

---

## Fix 1–2: Obstacle Persistence + l_data Refresh

**Files:** `trajectory_tracking/src/RM_GridMap.cpp`, `trajectory_generation/src/RM_GridMap.cpp`

- **Fix 1:** Replaced hard `memset(l_data, 0, ...)` with temporal decay (`OBS_PERSIST_FRAMES=5`). Obstacles persist ~0.5s after leaving scan.
- **Fix 2:** Changed swell-loop skip condition from `isOccupied()` (checks `data` AND `l_data`) to `data[...]==1` (static only). Fixes counter refresh bug where `l_data` flickered off for 1 frame.

## Fix 3: Height Filter Tuning

`search_height_min`: 0.1→-0.05m, `height_threshold`: 0.15→0.08m. Catches near-ground obstacles.

## Fix 4: Proximity Gate → Always-On Search

**File:** `local_planner.cpp` → `linearation()`

Removed binary `safe_constraint` gate (all-or-nothing obstacle search). Now always searches ±15 cells, sorts by distance, caps at `MAX_OBS_PER_STEP=5`.

## Fix 5–7: Solver Robustness

- **5:** HPIPM: SPEED→ROBUST mode, reg_prim 1e-12→1e-8
- **6:** One-time solver init (was recreating every cycle → SIGSEGV)
- **7:** NaN/Inf guards on polynomial coefficients, reference, and observation

## Fix 8: Collision Distance Threshold

**File:** `SentryRobotCollisionConstraint.h/cpp`

`distance_threshold_`: 0.36→0.09 (0.3m effective). Removed +0.15 static wall extra penalty. Total clearance: 0.35+0.3=0.65m (was 1.06m). MPC deviation dropped 0.90→0.32m mean.

## Fix 9–10: MPC Cost & Barrier Tuning

**Fix 9 (task.info):** Q(x,y)=150, Q(v)=50, Q(yaw)=80, R(a)=1.5, R(ω)=0.15.  
**Fix 10:** Collision barrier μ=2.5, δ=0.25. State-input barrier μ=0.5, δ=0.1.

## Fix 11: Speed & Accel Limits

max_accel: 6→3.5 m/s², max_vel: 6→2.5 m/s, max_angular: 7→6 rad/s.  
reference_desire_speed: 2.8→2.0 m/s.

## Fix 12: Topo Graph Dynamic Obstacle Visibility (Critical)

**File:** `TopoSearch.cpp` → `lineVisib()`

Changed `isStaticOccupied()` → `isOccupied()` so topo graph edges respect LiDAR-detected dynamic obstacles.

## Fix 13: Path Commitment / Anti-Oscillation (Critical)

**File:** `plan_manager.cpp` → `replanFinding()`

Three parts: (A) Skip replan when path safe + target unchanged. (B) Hysteresis: keep old path unless new is >20% shorter. (C) Dynamic obstacles in graph via Fix 12. Replans dropped from ~10/15s to 0.

## Fix 14: Reference Trajectory Re-Anchoring

**File:** `plan_manager.cpp` → `replanFinding()`

On skip-replan, trim `optimized_path` to current robot position before re-smoothing. Prevents polynomial from anchoring behind robot.

## Fix 15–16: Global Persistence + smoothTopoPath Fix

- **15:** `OBS_PERSIST` in trajectory_generation: 5→20→10 frames (1s).
- **16:** `smoothTopoPath()`: occupied cells treated as collision events instead of silently skipped.

## Fix 17–19: MPC Obstacle Window + Differentiated Clearance + Barrier

- **17:** `OBS_SEARCH_HALF`: 15→25 (±1.25m), `MAX_OBS_PER_STEP`: 5→8
- **18:** Added `distance_threshold_dynamic_=0.25` (0.5m effective) for moving obstacles
- **19:** Barrier μ: 2.5→3.5

## Fix 20–22: Goal Deceleration + Arrival

- **20:** Zero reference velocity past trajectory end
- **21:** Distance-proportional deceleration ramp (1.5m zone)
- **22:** arrival_goal threshold 0.1→0.3m, removed premature replan at 1.0m

## Fix 23–25: Replan Debounce + SQP + Speed Alignment

- **23:** Collision buffer 25→15, thresholds 15→10 collision, 6→4 tracking_low
- **24:** sqpIteration: 10→6
- **25:** reference_desire_speed aligned: 2.8→2.0 m/s

## Fix 26: Obstacle Persistence Reduction

Global planner `OBS_PERSIST`: 20→10 (2s→1s). Ghost obstacles blocked topo routing.

## Fix 27: Deceleration Ramp Relocation

Moved decel ramp from `linearation()` (velocity vector scaling, inconsistent with position) to `ref_speed` (scalar speed only). Position/velocity pair now consistent from polynomial.

## Fix 28: MINCO Weight Rebalancing & Safety Guards (Critical)

**Files:** `path_smooth.cpp`, `reference_path.cpp`

After MINCO Hessian fix, correct $1/T^4$ scaling caused energy gradient explosion (~42,700/wp vs obstacle ~2,000/wp). Nine sub-fixes:
- **A:** wSmooth 0.1→1e-3, wFidelity 1e3→5e3, wVel 1e4→1e3
- **B:** Energy gradient clipping at 2000/wp
- **C:** NaN/Inf guards in cost function
- **D:** L-BFGS max_iterations: unlimited→200
- **E:** Waypoint drift clamp: 0.5m radius from reference
- **F:** Velocity inflation cap: 5× per segment
- **G:** Per-segment time clamp: [0.01, 2.0]s
- **H:** Total trajectory time cap: 30s (prevents OOM)
- **I:** Diagnostic logging

## Fix 29: Corner Rounding + pathResample + Local-Time Fix

- **A:** wFidelity: 5e3→500 (energy now dominates at corners)
- **B:** Fixed `pathResample()` dead-code bug (decimated array computed but discarded). Now properly decimates every 2nd waypoint with merged MINCO times.
- **C:** Fixed inconsistent local-time in `reference_path.cpp` (`getRefTrajectory`/`getRefVel`). All terms now use `i*dt - total_time`.

## Fix 30: Replan Oscillation (Critical)

Near dynamic obstacles, each replan produced a different random PRM → zigzag.
- 1.0s replan cooldown in FSM
- Obstacle cache refresh every 50 L-BFGS iterations
- Collision window: 15→30 frames, threshold 10→20

## Fix 31: Zigzag at Turns

wFidelity: 500→100, wSmooth: 1e-3→5e-3, drift clamp: 0.5→0.8m, obstacle search: ±8→±12 cells.

## Fix 32: Localization Floor Z-Sinking

Removed z=0 clamp in `publish_odometry()`. Added `imu_filter_3` node. Switched hdl_localization IMU to match mapping IMU (`_105` → `_3`).

## Fix 33: Five-Part System Fix

- **A:** Cross-track deviation replan trigger (>1.0m from path → full replan)
- **B:** Guard/connection viz: removed `*2` z-multiplier, reduced z-scale
- **C:** Set `init_pos_z=0.3`, added soft z-constraint (±0.15m)
- **D:** wSmooth: 5e-3→2e-2, wFidelity: 100→20
- **E:** Barrier μ: 3.5→2.5, R(a): 1.0→1.5

## Fix 34: Two-Layer Guards + MPC Oscillation (4 parts)

- **A:** Removed dual-sampling (second_height=0.08 phantom layer) from `topoSampleMap()` and `createGraph()`
- **B:** speed_direction hysteresis: enter-reverse at 108°, exit at 63° (was hard ±90°)
- **C:** Time-reset debounce: threshold 0.4→0.6m, 0.3s cooldown
- **D:** Cold-start `mpcSolverPtr_->reset()` on new trajectory

## Fix 35: Observation Integrity + DISPENSE + Guard Flatten

- **A:** Flattened all guard/connection viz z to 0.0
- **B:** Removed `speed_direction` from observation state (was negating true speed)
- **C:** Time-reset threshold: 0.6→0.8m
- **D:** DISPENSE thresholds raised: tracking_time 1.2→2.0s, max_speed 0.3→0.15, avg 0.1→0.05

## Fix 36: MAX_LEAD_TIME Clamping

Added continuous time-clamping: cap `cost_time` to `closest_id * dt + MAX_LEAD_TIME(1.0s)`.

## Fix 37: Frame Mismatch — LiDAR-to-Chassis Offset (Critical)

**File:** `tracking_manager.cpp`

Removed LiDAR-to-chassis offset (dx=-0.011, dy=-0.172). Now both tracker and planner use raw `aft_mapped` position. Ref↔Pred distance: 7.7m→0.07m. "MPC is not safe" rate: 92/10s→19/10s.

## Fix 38: Replan on Clear + Off-Course + Startup + Obstacle Speed

- **a:** Off-course detection: `tracking_low_check_flag` now actually populated when min_dist>0.8m
- **b:** Periodic replan every 5.0s (for cleared obstacles)
- **c:** MAX_LEAD_TIME: 1.0→2.0s (faster startup — was 7s to 0.3 m/s, now instant)
- **d:** Obstacle-proximity speed reduction: scale=`max(0.35, dist/0.8)` for obs<0.8m

## Fix 39: MPC Through Walls — Predicted-Path Obstacle Search

- **a:** Search obstacles around prev_predictState_ (not just reference)
- **b:** OBS_SEARCH_HALF: 25→35, MAX_OBS_PER_STEP: 8→16
- **c:** Barrier μ: 2.5→3.5, δ: 0.25→0.5
- **d:** dist_static: 0.09→0.16, dist_dynamic: 0.25→0.36
- **e:** OBS_SLOW_DIST: 0.8→1.5m, OBS_MIN_SCALE: 0.35→0.15, horizon-wide look-ahead

## Fix 40: Comprehensive Obstacle Avoidance Overhaul

Fix 39 still failing: barrier cost (8.4) << tracking cost (37.5) per step.
- **a:** Reference repulsion from obstacles (REPEL_DIST=0.5m) — later disabled in Fix 42b
- **b:** Barrier μ: 3.5→20, δ: 0.5→1.5
- **c:** dist_static: 0.16→0.25, dist_dynamic: 0.36→0.49
- **d:** sqpIteration: 6→10
- **e:** prev_predictState_ zero-init guard
- **f:** OBS_SEARCH_HALF: 35→30, MAX_OBS_PER_STEP: 16→12

## Fix 42: MPC Stability Overhaul

Fix 41's sector obstacles (219 obs) overwhelmed solver. Speed reduction bug: step×horizon=0.015².
- 8 sectors, ±20 cells, 1.0m distance filter
- Reference repulsion disabled; barrier μ=20, δ=1.0
- 3s replan cooldown after new trajectory
- Speed reduction: min(step,horizon) instead of multiply, OBS_MIN_SCALE 0.15→0.25
- checkfeasible horizon: 50%→25%, sqpIteration: 10→12

## Fix 43: THE SMOKING GUN — Reference Look-Ahead Gap

`MAX_LEAD_TIME=2.0` pushed MPC reference 2-3m AHEAD of robot. Obstacle search (±1.0m around reference) couldn't see obstacles near the robot → predicted path through walls.
- **a:** MAX_LEAD_TIME: 2.0→0.5s
- **b:** Removed `exist_second_height` gate in `linearation()` (was disabling all obstacle search in bridge areas)
- **c:** Robot-position obstacle search for MPC steps 0-4

Result: "MPC is not safe" rate from ~8.5% to 0%. min_obs from 0.17m to 0.59-0.70m.

## Fix 44: Velocity Heading + Barrier Strength (Session 2)

- **a:** MPC φ from gimbal yaw → velocity heading via atan2(vy,vx). BUG: assumed world-frame twist.
- **b:** Barrier μ: 3.5→40, δ: 0.25→1.0
- **c:** dist_static: 0.09→0.36, dist_dynamic: 0.25→0.64

## Fix 45: Frame Rotation Corrections (Session 3)

- **a:** Body→world rotation for twist before velocity heading
- **b:** World→gimbal rotation for MPC output to MCU
- **c–e:** Velocity heading threshold 0.15→0.03, target-direction fallback, v_ctrl≥0 clamp

## Fix 46: Omnidirectional MPC + Narrow-Corridor Fixes (Session 3)

- **a:** Override MPC observation φ with reference direction when speed<0.3 m/s
- **b:** Removed gimbal yaw from velocity heading fallback
- **d:** Fixed polynomial timing: `0.3/desire_vel` → `path_distance/desire_vel` (8.3m segment was getting 0.189s)
- **e:** Intermediate point insertion for short paths (≤4 pts, segments >0.6m)
- **f:** Reduced collision thresholds for narrow corridors: static 0.36→0.04 (0.2m), dynamic 0.64→0.16 (0.4m). Barrier μ: 40→20, δ: 1.0→0.5.

## Fix 47: smoothTopoPath Safety Margin + Convergence

**File:** `Astar_searcher.cpp`, `Astar_searcher.h`

- **47a:** Added `cell_margin` parameter to `lineVisib()`. `smoothTopoPath()` passes `cell_margin=3` (0.15m buffer → 0.50m total clearance).
- **47b:** Reduced iteration limit 1000→500. On limit, returns partial smooth path + remaining raw tail.

## Fix 48: checkfeasible Dynamic-Only Detection

**File:** `local_planner.cpp`

Modified `checkfeasible()` to skip cells in inflated static map (`data[flat]==1`). Only `l_data>0 AND data!=1` (dynamic-only) triggers collision flag. Eliminates false-positive "MPC is not safe" from known static walls.

## Fix 49: Guard Point Density + Sampling + Path Clearance

**Date:** 2026-03-22  
**Files:** `TopoSearch.h`, `TopoSearch.cpp`, `Astar_searcher.cpp`  
**Build:** ✅ catkin_make exit 0

### Root Cause

Guard points were excessively dense (~200+ in a 20×20m map). Three creation paths had NO minimum inter-guard distance check:
1. **Topo keypoints with no visible guards** → unconditional add (no distance check)
2. **Keypoints near start/end** (`near_terminal` flag) → bypassed the `min_distance > 2.0` threshold
3. **Random samples with no visible guards** → unconditional add

Only the narrow-corridor path (`visib_guards.size()==1`) had a `min_distance < 0.5` check. Dense guards caused Dijkstra to route through overshoot waypoints past the goal, producing reference path loops and twists.

### Changes

- **49a:** Added `tooCloseToExistingGuard(pt, MIN_GUARD_SPACING)` helper (2D Euclidean check against all guard-type nodes). Applied to ALL guard creation paths in both `createGraph()` and `createLocalGraph()`. Removed `near_terminal` bypass. `MIN_GUARD_SPACING = 0.5m`.
- **49b:** Reduced `max_sample_num`: 1000→500.
- **49c:** Reduced `cell_margin` in `smoothTopoPath()`: 3→2 (0.15m→0.10m safety margin) for more direct shortcuts.

### Test Results

| Metric | Pre-Fix 49 | Post-Fix 49 |
|--------|-----------|-------------|
| Guard point count | ~200+ | **38–42** |
| Reference path points | 360 (with loop) | **48–50** (monotonic) |
| Reference path shape | Loop near goal (x oscillates) | **Smooth, no loop** |
| Raw topo path | Overshoots goal | **Direct to goal** |
| cmd_vel | Sometimes zero | **Non-zero (vx≈0.50, vy≈0.37)** |

---

## Current Parameter Summary

### MPC (task.info + local_planner.cpp)
| Parameter | Value |
|-----------|-------|
| Q(x,y) | 150 |
| Q(v) | 50 |
| Q(yaw) | 80 |
| R(accel) | 1.5 |
| R(omega) | 0.15 |
| sqpIteration | 12 |
| Barrier μ/δ | 20 / 0.5 |
| dist_static | 0.04 (0.2m effective) |
| dist_dynamic | 0.16 (0.4m effective) |
| MAX_LEAD_TIME | 0.5s |
| OBS_SEARCH_HALF | 20 cells (±1.0m) |
| MAX_OBS_PER_STEP | 8 sectors |
| OBS_SLOW_DIST | 1.5m |
| OBS_MIN_SCALE | 0.25 |

### Global Planner (TopoSearch + path_smooth)
| Parameter | Value |
|-----------|-------|
| MIN_GUARD_SPACING | 0.5m |
| max_sample_num | 500 |
| reference_desire_speed | 2.0 m/s |
| OBS_PERSIST (generation) | 10 frames (1s) |
| OBS_PERSIST (tracking) | 5 frames (0.5s) |
| wSmooth | 2e-2 |
| wFidelity | 20 |
| wVel | 1e3 |
| Drift clamp | 0.8m |
| L-BFGS max_iter | 200 |
| smoothTopoPath cell_margin | 2 |

### Speed Limits
| Parameter | Value |
|-----------|-------|
| max_state_velocity | 2.5 m/s |
| max_input_acceleration | 3.5 m/s² |
| max_input_angular | 6.0 rad/s |
| local_v_max | 2.0 m/s |
| reference_v_max | 2.5 m/s |

---

## Files Modified (All Fixes)

| File | Key Changes |
|------|-------------|
| `trajectory_tracking/src/RM_GridMap.cpp` | Fix 1-2: temporal decay, l_data refresh |
| `trajectory_generation/src/RM_GridMap.cpp` | Fix 1-2, 15, 26: persistence 5→20→10, dual-sample removal |
| `trajectory_tracking/src/local_planner.cpp` | Fix 4-7, 17, 19-21, 27, 34-36, 38-40, 42-44, 46, 48 |
| `trajectory_tracking/src/tracking_manager.cpp` | Fix 22-23, 35, 37-38, 45 |
| `trajectory_tracking/cfg/task.info` | Fix 9, 24, 40, 42 |
| `trajectory_tracking/include/.../SentryRobotCollisionConstraint.h` | Fix 8, 18, 39-40, 44, 46 |
| `trajectory_tracking/include/.../SentryRobotStateInputConstraint.h` | Fix 11 |
| `trajectory_generation/src/TopoSearch.cpp` | Fix 12, 34, 49 |
| `trajectory_generation/src/TopoSearch.h` | Fix 49 |
| `trajectory_generation/src/plan_manager.cpp` | Fix 13-14, 33 |
| `trajectory_generation/src/Astar_searcher.cpp` | Fix 16, 47, 49 |
| `trajectory_generation/src/path_smooth.cpp` | Fix 28-31, 46 |
| `trajectory_generation/src/reference_path.cpp` | Fix 28-29, 46 |
| `trajectory_generation/src/visualization_utils.cpp` | Fix 33, 35 |
| `trajectory_generation/launch/global_searcher.launch` | Fix 3, 25 |
| `trajectory_tracking/launch/trajectory_planning.launch` | Fix 3, 25 |
| `hdl_localization/apps/hdl_localization_nodelet.cpp` | Fix 32-33 |
