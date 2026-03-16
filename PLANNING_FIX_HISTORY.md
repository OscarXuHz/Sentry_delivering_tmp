# Sentry Planning System — Fix History

> Full debug log of all changes applied to the HIT sentry planning pipeline
> (trajectory_generation + trajectory_tracking) across multiple sessions.
>
> **Date range:** March 2026  
> **System:** ROS Noetic, OCS2 SQP-MPC, HPIPM QP solver  
> **State:** `[x, y, v, φ]` (4D), **Input:** `[accel, ω]` (2D)  
> **Map:** 0.05 m/cell, 20×20 m, BEV height map with bridge detection

---

## Problem Summary

Two persistent issues in the sentry robot's navigation:

1. **Global planner ignored real-time (LiDAR-detected) obstacles** — the robot drove straight through them.
2. **MPC predicted path fluctuated violently** near obstacles and veered far off the reference path.

---

## Fix 1: Obstacle Persistence (Temporal Decay)

**Files modified:**
- `trajectory_tracking/src/RM_GridMap.cpp` → `localPointCloudToObstacle()`
- `trajectory_generation/src/RM_GridMap.cpp` → `localPointCloudToObstacle()`

**Root cause:** `l_data` (local obstacle grid) was reset with `memset(l_data, 0, ...)` every LiDAR frame. Obstacles behind the robot (no longer in the current scan) vanished instantly, causing the MPC and global planner to plan through them.

**Fix:** Replaced hard `memset` reset with temporal decay:
```cpp
static const uint8_t OBS_PERSIST_FRAMES = 5;
for (int k = 0; k < GLXY_SIZE; ++k) {
    if (l_data[k] > 0) l_data[k]--;
}
```
And in `localSetObs()`:
```cpp
l_data[idx_x * GLY_SIZE + idx_y] = OBS_PERSIST;  // refresh counter
```

**Effect:** Obstacles persist for ~0.5s (5 frames at 10 Hz) after leaving the scan frustum.

---

## Fix 2: l_data Refresh Bug (Critical)

**Files modified:**
- `trajectory_tracking/src/RM_GridMap.cpp` → `localPointCloudToObstacle()` (swell loop)
- `trajectory_generation/src/RM_GridMap.cpp` → `localPointCloudToObstacle()` (swell loop)

**Root cause:** The point cloud processing loop used `isOccupied()` to skip cells that were already occupied. But `isOccupied()` checks `l_data[cell] > 0`, which means cells with a nonzero persist counter were skipped — **preventing the counter from being refreshed**. Result: every obstacle's counter ticked down from 5 to 0 and the obstacle flickered off for one frame before being re-detected and re-set. This created a periodic ~0.5s gap in obstacle visibility.

**Fix:** Changed the skip condition from `isOccupied(idx_x, idx_y, idx_z)` (which checks both `data` AND `l_data`) to `data[idx_x * GLY_SIZE + idx_y] == 1` (only skip static map walls):
```cpp
// Before (BUG):
if(isOccupied(idx_x, idx_y, idx_z) || GridNodeMap[idx_x][idx_y]->exist_second_height)
    continue;

// After (FIX):
if(data[idx_x * GLY_SIZE + idx_y] == 1 || GridNodeMap[idx_x][idx_y]->exist_second_height)
    continue;
```

**Effect:** Dynamic obstacles now persist continuously as long as LiDAR detects them. **This fixed the global planner's obstacle avoidance.**

---

## Fix 3: Height Filter Parameter Tuning

**Files modified:**
- `trajectory_tracking/launch/trajectory_planning.launch`
- `trajectory_generation/launch/global_searcher.launch`

**Changes:**
| Parameter | Before | After | Why |
|-----------|--------|-------|-----|
| `search_height_min` | 0.1 m | -0.05 m | Catch near-ground obstacles |
| `height_threshold` | 0.15 m | 0.08 m | Detect lower obstacles against BEV height |

**Effect:** Ground-level obstacles (boxes, low barriers) now detected by both planners.

---

## Fix 4: Proximity Gate (Replacing Binary safe_constraint)

**File:** `trajectory_tracking/src/local_planner.cpp` → `linearation()`

**Root cause:** The original code had a `safe_constraint` boolean gate: if the reference point's grid cell was occupied, ALL obstacles in a ±15-cell window were added as constraints. If NOT occupied, zero constraints were added. When an obstacle was near the edge of the path, one frame had 0 constraints, the next had hundreds → massive QP structure flip → solver oscillated between two solutions → visible path flicker.

**Fix:** Removed the binary gate. Instead:
1. **Always** search for obstacles around each reference point (±15 cells = ±0.75 m)
2. Sort by distance from the reference point
3. Cap to `MAX_OBS_PER_STEP` closest obstacles (now 5, was 10)

```cpp
static const int OBS_SEARCH_HALF = 15;
static const size_t MAX_OBS_PER_STEP = 5;
// Always search, sort by distance, keep closest MAX_OBS_PER_STEP
```

**Effect:** QP constraint set size remains approximately constant across frames → stable solver behavior.

---

## Fix 5: HPIPM Solver Robustness

**File:** `trajectory_tracking/src/local_planner.cpp` → `linearation()` (solver init block)

**Changes:**
```cpp
hSet.hpipmMode = hpipm_mode::ROBUST;  // was SPEED
hSet.reg_prim  = 1e-8;                // was 1e-12
```

**Reason:** With near-zero regularization and SPEED mode, the Riccati recursion failed on mildly ill-conditioned QPs (especially when obstacle constraints created near-singular Hessians). ROBUST mode adds defensive pivoting.

---

## Fix 6: Solver One-Time Initialization

**File:** `trajectory_tracking/src/local_planner.cpp` → `linearation()`

**Root cause:** The MPC solver was recreated every cycle with `mpcSolverPtr_.reset(new SqpMpc(...))`. OCS2's SqpMpc uses `nThreads=2`; if the previous solver was mid-solve when `reset()` destroyed it, internal threads accessed freed memory → SIGSEGV.

**Fix:** Create solver once on first call, reuse thereafter:
```cpp
if (!mpcSolverPtr_) {
    // ... setup collision constraint, HPIPM settings ...
    mpcSolverPtr_.reset(new ocs2::SqpMpc(...));
}
```

Obstacle updates work via `shared_ptr` — the solver sees updated obstacles through the shared constraint pointer.

---

## Fix 7: NaN Guards

**File:** `trajectory_tracking/src/local_planner.cpp`

**Added NaN/Inf checks at three critical points:**
1. **Polynomial coefficients** on receipt from global planner (`rcvGlobalTrajectory`)
2. **Evaluated reference trajectory** after polynomial interpolation
3. **Observation state** and **reference state** before feeding to solver

**Effect:** Prevents corrupt data from crashing the solver and provides diagnostic logging.

---

## Fix 8: Collision Distance Threshold (Critical)

**Files modified:**
- `trajectory_tracking/include/ocs2_sentry/constraint/SentryRobotCollisionConstraint.h`
- `trajectory_tracking/src/ocs2_sentry/constraint/SentryRobotCollisionConstraint.cpp`

**Root cause:** The collision constraint used `distance_threshold_ = 0.36` (i.e., 0.6 m radius from inflated obstacle center). Since the occupancy map already inflates obstacles by `robot_radius = 0.35 m`, the total clearance was **0.35 + 0.6 = 0.95 m** from actual walls. For static walls, an additional `+0.15` penalty made it `sqrt(0.51) = 0.71 m`, totalling **1.06 m from actual walls**. The MPC literally could not follow any reference path within 1 m of a wall without the collision barrier overpowering position tracking (Q=80).

**Fix:**
```cpp
// Header: reduced threshold
ocs2::scalar_t distance_threshold_ = 0.09;  // 0.3m radius (was 0.6m)

// Source: removed static wall extra penalty
// Before: constraintValue[i] = distance - distance_threshold_ - 0.15;  (type 0)
// After:  constraintValue[i] = distance - distance_threshold_;         (same for both types)
```

Total clearance now: 0.35m (inflation) + 0.3m (constraint) = **0.65m from actual walls** — adequate safety with much better tracking.

**Effect:** MPC deviation from reference dropped from 0.90m → 0.45m mean.

---

## Fix 9: MPC Cost Matrix Tuning

**File:** `trajectory_tracking/cfg/task.info`

### Tuning history:

| Parameter | Original | Round 1 | Round 2 | Round 3 (final) |
|-----------|----------|---------|---------|-----------------|
| Q(x,y) | 80 | 80 | 80 | **150** |
| Q(v) | 0 | 50 | 50 | 50 |
| Q(yaw) | 70 | 70 | 70 | 80 |
| R(accel) | 0.2 | 2.0 | 8.0 → 2.5 | **1.5** |
| R(omega) | 0.1 | 0.1 | 0.5 → 0.15 | **0.15** |

**Key insight:** The Q/R ratio determines tracking accuracy. Original Q/R = 80/0.2 = 400 (good tracking, no smoothing). R=8 gave Q/R = 10 (terrible tracking). R=1.5 with Q=150 gives Q/R = 100 (good tracking with meaningful smoothing).

**Final values:**
```
Q { (0,0) 150; (1,1) 150; (2,2) 50; (3,3) 80 }
R { (0,0) 1.5; (1,1) 0.15 }
```

---

## Fix 10: Barrier Penalty Tuning

### Collision barrier (local_planner.cpp):
```cpp
RelaxedBarrierPenalty::Config(2.5, 0.25);
// Original: (0.1, 0.2) — too weak
// Round 2:  (2.0, 0.25) — reasonable
// Round 3:  (5.0, 0.20) — too strong (overpowered Q)
// Final:    (2.5, 0.25) — balanced
```

### State-input barrier (SentryRobotInterface.cpp):
```cpp
RelaxedBarrierPenalty::Config(0.5, 0.1);
// Original: (0.1, 0.05) — near-zero, limits not enforced
// Round 2:  (2.0, 0.1)  — too strong, boundary oscillation
// Final:    (0.5, 0.1)  — soft enforcement
```

---

## Fix 11: Speed & Acceleration Limits

### Hard limits (`SentryRobotStateInputConstraint.h`):
| Parameter | Original | Final |
|-----------|----------|-------|
| `max_input_acceleration_` | 6.0 m/s² | **3.5 m/s²** |
| `max_input_angular_` | 7.0 rad/s | **6.0 rad/s** |
| `max_state_velocity_` | 6.0 m/s | **2.5 m/s** |

### Speed references (launch files):
| Parameter | Original | Final |
|-----------|----------|-------|
| `local_v_max` | 2.8 m/s | **2.0 m/s** |
| `local_a_max` | 6.0 m/s² | **3.5 m/s²** |
| `reference_desire_speed` | 2.8 m/s | **2.0 m/s** |
| `reference_v_max` | 3.5 m/s | **2.5 m/s** |
| `reference_a_max` | 10.0 m/s² | **4.0 m/s²** |

---

## Diagnostic Results (Before → After)

### Global planner obstacle avoidance:
- **Before:** Robot drove through real-time obstacles (invisible to planner)
- **After:** Global path avoids both static and real-time obstacles ✅

### MPC path deviation (MPC predicted vs reference):
| Metric | Before all fixes | After all fixes |
|--------|-----------------|-----------------|
| Mean deviation (overall) | 0.90 m | **0.32 m** |
| Peak deviation | 2.35 m | **1.14 m** |
| Step 0 (immediate) | 0.23 m | 0.24 m |
| Step 19 (horizon end) | 0.96 m | **0.23 m** |

### MPC path stability (frame-to-frame):
| Metric | Before | After |
|--------|--------|-------|
| Mean maxDelta | 0.44 m | **0.33 m** |
| Peak maxDelta | 1.92 m | **1.38 m** |

### Speed control:
| Metric | Before | After |
|--------|--------|-------|
| Mean cmd_vel | 1.29 m/s | **0.64 m/s** |
| Peak cmd_vel | 2.44 m/s | **0.81 m/s** |
| Peak acceleration | 14.6 m/s² | **8.4 m/s²** |

---

## Fix 12: Topo Graph Dynamic Obstacle Visibility (Critical)

**File:** `trajectory_generation/src/TopoSearch.cpp` → `lineVisib()`

**Root cause:** The topological PRM graph construction used `isStaticOccupied()` for visibility raycasting, which only checks static map obstacles (`data`). Dynamic/real-time obstacles (`l_data` — LiDAR-detected obstacles with temporal persistence) were completely invisible to graph construction. This meant:
- Topo edges were drawn straight through dynamic obstacles
- Dijkstra found shortest paths that passed through real-time obstacles
- The `checkPathCollision()` recheck detected collisions → triggered replan → but the new PRM also ignored dynamic obstacles → same result

**Fix:**
```cpp
// Before:
if (global_map->isStaticOccupied(pt_idx, pt_idy, second_height)){
    return false;
}

// After:
if (global_map->isOccupied(pt_idx, pt_idy, pt_idz, second_height)){
    return false;
}
```

`isOccupied()` checks BOTH `data[i]==1` (static) AND `l_data[i]>0` (dynamic), so:
- Topo graph edges now respect dynamic obstacles
- Dijkstra paths route around LiDAR-detected obstacles
- Combined with Fix 1 (temporal persistence) and Fix 2 (l_data refresh), dynamic obstacles are reliably avoided

**Effect:** Global planner no longer generates paths through real-time obstacles in the arena center.

---

## Fix 13: Path Commitment / Anti-Oscillation (Critical)

**File:** `trajectory_generation/src/plan_manager.cpp` → `replanFinding()`

**Root cause:** When a replan was triggered (via `/replan_flag`), `replanFinding()` called `checkPathCollision()` on the current path. If no collision was found, it **still called `pathFinding()` from scratch** — which rebuilds the entire topological PRM with 1000+ random samples, producing a different graph each time. Near symmetric obstacles (e.g., boxes in the arena center), Dijkstra would alternately find the left-side or right-side path depending on random sampling → the global path oscillated left↔right → the MPC tracked a constantly-changing reference → violent shaking.

**Fix — Three-part:**

### Part A: Skip replan when path is safe and target unchanged
```cpp
// When no collision and target distance <= 0.5m:
ROS_INFO("[Manager REPLAN] path safe & target unchanged, skipping replan");
return true;  // Keep existing path and trajectory
```

### Part B: Hysteresis when target has moved
When the target changes (distance > 0.5m), a full replan IS needed. But to prevent oscillation during replanning:
```cpp
// Save old path
std::vector<Eigen::Vector3d> old_path = optimized_path;
double old_len = pathLength(old_path);

// Generate new path
pathFinding(start_point, target_point, start_vel);
double new_len = pathLength(optimized_path);

// Hysteresis: only switch if new path is >20% shorter OR old path has collision
if(old_len > 0 && new_len > old_len * 0.8){
    if(!checkPathCollision(old_path)){
        optimized_path = old_path;  // Keep old path
        // Re-generate reference trajectory from current position
    }
}
```

### Part C: Dynamic obstacles in replan paths (via Fix 12)
When replanning IS triggered, the new topo graph now respects dynamic obstacles, so the replanned path actually avoids them.

**Effect:**
- Replans dropped from ~10 per 15s to **0** during normal navigation
- Path side flips: **0** (was oscillating left↔right)
- Path midpoint delta: **0.000m** (perfectly stable)
- Path length std: **0.00m** (zero variance across all frames)

---

## Fix 14: Reference Trajectory Re-Anchoring (Critical)

**File:** `trajectory_generation/src/plan_manager.cpp` → `replanFinding()`

**Root cause:** Fix 13 introduced a "skip replan" optimization: when the current path is collision-free and the target hasn't moved, `replanFinding()` returned `true` without regenerating the reference trajectory. However, the FSM **always publishes the polynomial trajectory** when `replanFinding()` returns `true`, and the tracker's `rcvGlobalTrajectory()` **always resets `start_tracking_time = ros::Time::now()`**. This meant:

1. Robot moves from position A to B
2. Replan triggered → path is safe → `return true` (no trajectory update)
3. FSM publishes the SAME polynomial (still anchored at position A)
4. Tracker resets `start_tracking_time` → evaluates polynomial from t=0 → reference point is at position A (behind the robot)
5. MPC tries to track a point behind the robot → robot veers or reverses → "tracking time low" recovery kicks in → violent oscillation

The hysteresis branch (keeping old path when target has moved) had a similar but milder issue: the old path was re-smoothed from its original start point, not trimmed to the current robot position.

**Fix — Two-part:**

### Part A: "Skip replan" branch — always re-anchor from current position
```cpp
// Instead of bare "return true":
// 1. Find closest waypoint on optimized_path to current robot position
int closest_idx = 0;
double min_dist = 1e9;
for (int i = 0; i < (int)optimized_path.size(); i++) {
    double d = (optimized_path[i].head<2>() - start_point.head<2>()).norm();
    if (d < min_dist) { min_dist = d; closest_idx = i; }
}
// 2. Trim: keep waypoints ahead of closest, prepend current position
int trim_idx = std::min(closest_idx + 1, (int)optimized_path.size() - 1);
std::vector<Eigen::Vector3d> trimmed;
trimmed.push_back(start_point);
for (int i = trim_idx; i < (int)optimized_path.size(); i++)
    trimmed.push_back(optimized_path[i]);
optimized_path = trimmed;
// 3. Re-smooth and regenerate reference trajectory
path_smoother->init(optimized_path, start_vel, reference_speed);
path_smoother->smoothPath();
path_smoother->pathResample();
// ... setGlobalPath, getRefTrajectory ...
```

### Part B: Hysteresis "keep old path" branch — add trimming before re-smoothing
Same trimming logic applied to `old_path` before the existing re-smoothing code.

**Effect:** The polynomial trajectory always starts from the robot's current position. The tracker's `start_tracking_time` reset now correctly initializes tracking from where the robot actually is.

---

## Fix 15: Global Planner Obstacle Persistence (5→20 frames)

**File:** `trajectory_generation/src/RM_GridMap.cpp` → `localPointCloudToObstacle()` + `localSetObs()`

**Root cause:** `OBS_PERSIST_FRAMES = 5` (from Fix 1) gives ~0.5s obstacle persistence at 10 Hz LiDAR rate. The global planner's replan cycle can take longer than 0.5s, and obstacles behind the robot (not in the current scan) vanish before the next replan checks them. This caused the global planner to generate paths through recently-seen obstacles in the center of the map.

**Fix:**
```cpp
// trajectory_generation/src/RM_GridMap.cpp:
static const uint8_t OBS_PERSIST_FRAMES = 20;  // was 5 → 2.0s at 10 Hz
static const uint8_t OBS_PERSIST = 20;          // was 5 → match decay constant
```

**Note:** `trajectory_tracking/src/RM_GridMap.cpp` keeps `OBS_PERSIST = 5` (0.5s) — the MPC local planner benefits from faster decay to react to obstacle changes quickly. Only the global planner needs longer persistence for map consistency across replan cycles.

**Effect:** Global planner sees obstacles for 2 seconds after they leave the LiDAR frustum, covering the full replan cycle window.

---

## Fix 16: smoothTopoPath Occupied-Cell Collision Handling

**File:** `trajectory_generation/src/Astar_searcher.cpp` → `smoothTopoPath()`

**Root cause:** The visibility-shortcutting algorithm walks along topo path waypoints and tries to "shortcut" by connecting distant waypoints with straight lines (if `lineVisib()` returns true). When an occupied cell was encountered on the topo path, the code used `continue` to silently skip it:

```cpp
// Before (BUG):
if(global_map->isOccupied(temp_id.x(), temp_id.y(), temp_id.z(), second_height)){
    continue;  // TODO 暂时处理为不考虑这种情况
}
```

This meant the occupied cell was never recorded as a collision event. Later, if a subsequent waypoint was visible from `head_pos`, the shortcut line could cut through the occupied area — producing a path that grazes obstacles.

**Fix:** Treat occupied cells as collision events (same as visibility failure):
```cpp
// After (FIX):
if(global_map->isOccupied(temp_id.x(), temp_id.y(), temp_id.z(), second_height)){
    if(!collision){
        collision = true;
        temp = tail_pos;
        bool easy = getNearPoint(temp, head_pos, temp);
        iter_idx = i;
    }
    continue;
}
```

When an occupied cell is hit:
1. If it's the first collision point, mark it and find a nearby free point via `getNearPoint()`
2. Insert a waypoint before the obstacle, forcing the shortcut to route around it
3. Later `lineVisib()` checks naturally prevent the shortcut from crossing the occupied area

**Effect:** Visibility shortcuts can no longer create paths that graze or pass through occupied cells on the topo path.

---

## ConstraintOrder::Linear — No Change Needed

**Investigation result (not a fix):** The user suspected that changing collision constraints from Quadratic to Linear may have caused tracking issues. Analysis confirmed:

- `SentryRobotCollisionConstraint.cpp` uses `ConstraintOrder::Linear` at construction (line 33)
- This was **never** `ConstraintOrder::Quadratic` in any version (checked backup, RM_Sentry_2026, and current)
- `Linear` is the correct choice: OCS2's SQP-MPC applies Gauss-Newton approximation (`J^T J`) which guarantees positive semi-definite Hessians
- `Quadratic` would require hand-crafted second derivatives and risks indefinite Hessians → solver instability

**Conclusion:** No change was made. `ConstraintOrder::Linear` is correct and should be kept.

---

## Fix 17: Widen MPC Obstacle Search Window

**File:** `trajectory_tracking/src/local_planner.cpp` → `linearation()`

**Root cause:** `OBS_SEARCH_HALF = 15` (±0.75m) meant obstacles beyond 0.75m from each reference point were invisible to the MPC. At 2.5 m/s, an obstacle at 0.8m arrives in 320ms — the solver couldn't even see it. `MAX_OBS_PER_STEP = 5` was too few for clustered moving obstacles.

**Fix:**
```cpp
static const int OBS_SEARCH_HALF = 25;        // ±1.25 m (was 15/±0.75m)
static const size_t MAX_OBS_PER_STEP = 8;     // was 5
```

**Effect:** MPC sees obstacles 67% further out. More constraints per step allows better coverage of obstacle clusters and moving objects.

---

## Fix 18: Differentiated Dynamic Obstacle Clearance

**Files:**
- `trajectory_tracking/include/.../SentryRobotCollisionConstraint.h`
- `trajectory_tracking/src/.../SentryRobotCollisionConstraint.cpp`

**Root cause:** Both static and dynamic obstacles used identical `distance_threshold_ = 0.09` ($\sqrt{0.09}$ = 0.3m effective clearance). With map inflation of 0.35m, total clearance was 0.65m. For static walls this is adequate, but for **moving** obstacles it's dangerously tight — a robot at 2.0 m/s covers 0.3m in only 150ms.

**Fix:**
```cpp
// Header:
ocs2::scalar_t distance_threshold_ = 0.09;         // static: 0.3m effective → 0.65m total
ocs2::scalar_t distance_threshold_dynamic_ = 0.25;  // dynamic: 0.5m effective → 0.85m total

// Source (getValue):
if(obs[i].first == 1){  // Dynamic
    constraintValue[i] = distance - distance_threshold_dynamic_;
}else if(obs[i].first == 0){  // Static
    constraintValue[i] = distance - distance_threshold_;
}
```

**Effect:** Moving obstacles get 0.85m total clearance (vs 0.65m for static), giving the MPC ~425ms reaction buffer at 2.0 m/s.

---

## Fix 19: Stronger Collision Barrier Penalty

**File:** `trajectory_tracking/src/local_planner.cpp` → solver init block

**Change:** Collision `RelaxedBarrierPenalty` µ increased from 2.5 → 3.5 (δ stays 0.25).

```cpp
RelaxedBarrierPenalty::Config barriercollisionPenaltyConfig(3.5, 0.25);  // was 2.5
```

**Rationale:** With Q=150, the Q/µ ratio is 43 (was 60) — still safely above the 10–20 range where obstacles overpower tracking, but strong enough that the MPC won't trade a small barrier violation for better reference following. This is critical for approaching moving obstacles where the penalty needs to dominate before contact.

---

## Fix 20: Zero Reference Velocity at Goal

**File:** `trajectory_tracking/src/local_planner.cpp` → `linearation()`

**Root cause:** When the MPC horizon extends past the trajectory end (i.e., robot is near goal), the reference velocity was set to the **last polynomial segment's velocity** — typically the cruise speed (2.0 m/s). The MPC had literally no incentive to decelerate because its v-reference was non-zero.

```cpp
// Before (BUG):
ref_velocity.push_back(reference_velocity[reference_velocity.size() - 1]);

// After (FIX):
ref_velocity.push_back(Eigen::Vector3d::Zero(3));
```

Combined with Q_v=60, the solver now actively decelerates when the horizon reaches the goal.

---

## Fix 21: Distance-Proportional Deceleration Ramp

**File:** `trajectory_tracking/src/local_planner.cpp` → `linearation()`

**Root cause:** Even with Fix 20 providing v=0 at the trajectory end, the reference velocity transitions abruptly from cruise speed to zero. The MPC can only decelerate at `max_input_acceleration_ = 3.5 m/s²`, and with stopping distance $v^2/(2a) = 2.0^2/7.0 ≈ 0.57$m, the solver can't physically stop in time if the deceleration starts only at the trajectory endpoint.

**Fix:** After computing each reference point's velocity, scale it proportionally to distance from goal:
```cpp
Eigen::Vector3d goal_pos = reference_path.back();
double dist_to_goal = (position_temp.head<2>() - goal_pos.head<2>()).norm();
static const double DECEL_RAMP_DIST = 1.5;  // start slowing 1.5m from goal
if (dist_to_goal < DECEL_RAMP_DIST) {
    double scale = std::max(0.05, dist_to_goal / DECEL_RAMP_DIST);
    velocity_temp *= scale;
}
```

**Effect:** Reference velocity smoothly decreases from 100% at 1.5m to 5% at 0m, giving the MPC a feasible deceleration profile. The 1.5m ramp distance provides 0.93m margin beyond the physical stopping distance.

---

## Fix 22: Increased Arrival Threshold + Removed Premature Replan

**File:** `trajectory_tracking/src/tracking_manager.cpp`

### Part A: `arrival_goal` threshold 0.1m → 0.3m

The old 0.1m threshold was far below the physical stopping distance (0.57m at 2.0 m/s). The robot consistently overshot the goal before `arrival_goal` triggered.

### Part B: Removed pre-emptive replan at 1.0m

The old code set `replan_now = true` when `target_distance < 1.0m`. This triggered a full global replan near the goal, which:
1. Regenerated the polynomial trajectory starting from the current position
2. Reset `start_tracking_time` in the tracker
3. Re-initialized the MPC reference from the new trajectory
4. Created an oscillation loop: approach goal → replan → MPC gets new trajectory → approaches again → replan

```cpp
// Before:
if(target_distance < 1.0 && !arrival_goal) replan_now = true;   // REMOVED
if(target_distance < 0.1)  arrival_goal = true;                  // was 0.1m

// After:
if(target_distance < 0.3)  arrival_goal = true;                  // 0.3m
```

---

## Fix 23: Faster Replan Debounce

**File:** `trajectory_tracking/src/tracking_manager.cpp` → `checkReplanFlag()`

**Root cause:** The collision detection buffer was 25 frames deep, requiring 15 collision frames (60%) to trigger a replan. At 100 Hz MPC, this means **minimum 0.15s** from first collision detection to replan trigger. For a moving obstacle approaching at 1.5 m/s, 0.15s = 0.225m of unplanned motion.

**Fix:**
```cpp
// Buffer size: 25 → 15
if(replan_check_flag.size() > 15) replan_check_flag.pop_back();

// Thresholds: collision 15→10, tracking_low 6→4
if(num_collision_error > 10 || num_tracking_low > 4 || replan_now)
```

**Effect:** Collision detection latency drops from 0.15s → 0.10s (10 of 15 frames at 100 Hz). Tracking-low detection: 4 of 10 frames (was 6 of 10).

---

## Fix 24: Reduced SQP Iterations

**File:** `trajectory_tracking/cfg/task.info`

```
sqpIteration    6    ; was 10
```

**Rationale:** With warm-start from the previous MPC solution, the SQP typically converges in 3-4 iterations. The remaining 6-7 iterations were wasted compute that risked exceeding the 10ms frame budget at 100 Hz (especially in HPIPM ROBUST mode). Reducing to 6 provides ample convergence margin while freeing ~40% of solver compute time, improving reaction latency.

---

## Fix 25: Aligned Global/Local Speed Limits

**Files:**
- `trajectory_generation/launch/global_searcher.launch`
- `trajectory_tracking/launch/trajectory_planning.launch`

| Parameter | Before | After | Why |
|-----------|--------|-------|-----|
| `reference_desire_speed` | 2.8 m/s | **2.0 m/s** | Was above `max_state_velocity_=2.5` → persistent tracking error |
| `local_v_max` | 2.8 m/s | **2.0 m/s** | Aligned with global speed reference |

**Root cause:** The global planner generated reference trajectories at 2.8 m/s, but the MPC hard-capped velocity at 2.5 m/s. The robot was always "behind" its reference, creating persistent tracking error that reduced obstacle avoidance effectiveness (the position cost Q dominated over the collision barrier because the robot was already far from reference).

---

## Fix 26: Reduce Obstacle Persistence in Global Planner

**File:** `trajectory_generation/src/RM_GridMap.cpp`

| Parameter | Before | After | Why |
|-----------|--------|-------|-----|
| `OBS_PERSIST_FRAMES` (line ~590) | 20 | **10** | Ghost obstacles lasted 2 s → 1 s |
| `OBS_PERSIST` (line ~696) | 20 | **10** | Same counter in `localSetObs()` |

**Symptom:** After a moving obstacle passed through an area and left, the global
planner still treated those cells as occupied for 2 full seconds.  During that
window the topo-PRM could not route through the cleared space, so it produced a
violent detour path that veered far off the normal topo map.

**Root cause:** Fix 15 raised `OBS_PERSIST` from 5 to 20 in
`trajectory_generation` to stop stale-obstacle flickering, but 20 frames at
100 ms/frame = 2 s is too long for a moving obstacle that crosses the area in
<0.5 s.  The "ghost" cells block every topo shortest-path query until they
naturally expire.

**Fix:** Lower both counters to **10** (≈ 1 s).  This is long enough to bridge
a single missed lidar scan yet short enough that a cleared corridor re-opens
within one global planning cycle (≈ 0.5 s).

**Trade-off:** If the lidar drops more than 10 consecutive frames (>1 s gap) on
a stationary obstacle, that obstacle will flicker.  In practice this is rare
because the lidar publishes at 10 Hz.

---

## Fix 27: Relocate Deceleration Ramp from linearation() to ref_speed

**File:** `trajectory_tracking/src/local_planner.cpp`

### Problem

Fix 21 added a deceleration ramp inside `linearation()` that scaled
`velocity_temp` (a 2-D vector) when the reference position was within 1.5 m of
the goal.  However, `position_temp` was still computed from the **un-scaled**
polynomial, meaning:

| MPC ref channel | Value | Consistent? |
|-----------------|-------|-------------|
| `ref_trajectory[i]` (position) | Full-speed polynomial evaluation | — |
| `ref_velocity[i]` (2-D velocity) | **Scaled down** by decel ramp | ✗ |
| `ref_speed[i]` (scalar, into MPC state) | Derived from `ref_velocity` norm | ✗ |

The MPC reference told the solver "your position should be advancing at full
speed, but your velocity should be very low."  These two statements are
mutually inconsistent → the predicted (optimal) trajectory diverges from the
reference trajectory because the solver cannot satisfy both simultaneously.

### Solution

| Step | What changed | Detail |
|------|-------------|--------|
| 27-A | **Removed** decel ramp from `linearation()` | `velocity_temp` is now pushed to `ref_velocity` without any scaling — positions and velocities are naturally consistent from the polynomial |
| 27-B | Added `goal_pos` variable | `goal_pos = reference_path.back()` computed once before the `ref_speed` loop in `getFightTrackingTraj()` |
| 27-C | **Added** decel ramp to `ref_speed` | At the end of each iteration of the `ref_speed` loop, `ref_speed.back()` is scaled by `max(0.05, dist_to_goal / 1.5)` when the reference position is within 1.5 m of the goal |

This keeps the **position/velocity** reference pair fully consistent (both come
directly from the polynomial), while still ramping down the **scalar speed** that
enters the MPC cost.  The MPC now smoothly decelerates near the goal without
the solver fighting an infeasible reference.

```cpp
// End of ref_speed loop body in getFightTrackingTraj():
// (Fix 27) Deceleration ramp: scale ref_speed near goal.
{
    double dist_to_goal = (ref_trajectory[i].head<2>() - goal_pos.head<2>()).norm();
    static const double DECEL_RAMP_DIST = 1.5;
    if (dist_to_goal < DECEL_RAMP_DIST) {
        double scale = std::max(0.05, dist_to_goal / DECEL_RAMP_DIST);
        ref_speed.back() *= scale;
    }
}
```

---

## Diagnostic Results (Updated)

### Global planner path stability (before Fix 12+13 → after):
| Metric | Before | After |
|--------|--------|-------|
| Replans per 15s | ~10 | **0** |
| Side flips per 15s | frequent | **0** |
| Path midpoint delta | variable | **0.000 m** |
| Path length std | variable | **0.00 m** |

---

## Files Modified (Summary)

| File | Changes |
|------|---------|
| `trajectory_tracking/src/RM_GridMap.cpp` | Temporal decay, l_data refresh bug fix |
| `trajectory_generation/src/RM_GridMap.cpp` | Same temporal decay, l_data refresh bug fix, **OBS_PERSIST 5→20→10** |
| `trajectory_tracking/src/local_planner.cpp` | Proximity gate, obs cap, HPIPM tuning, solver one-time init, NaN guards, barrier config, **obs search ±25, MAX_OBS 8, barrier µ=3.5, v=0 at goal, decel ramp moved to ref_speed** |
| `trajectory_tracking/cfg/task.info` | Q/R cost matrix tuning, **sqpIteration 10→6** |
| `trajectory_tracking/include/.../SentryRobotCollisionConstraint.h` | Distance threshold 0.36→0.09, **+ dynamic threshold 0.25** |
| `trajectory_tracking/src/.../SentryRobotCollisionConstraint.cpp` | Removed static wall extra penalty, **differentiated static/dynamic thresholds** |
| `trajectory_tracking/include/.../SentryRobotStateInputConstraint.h` | Speed/accel hard limits tightened |
| `trajectory_tracking/src/.../SentryRobotInterface.cpp` | State-input barrier penalty tuned |
| `trajectory_tracking/launch/trajectory_planning.launch` | Height params, speed limits, **local_v_max 2.8→2.0** |
| `trajectory_generation/launch/global_searcher.launch` | Height params, speed limits, **reference_desire_speed 2.8→2.0** |
| `trajectory_generation/src/TopoSearch.cpp` | `lineVisib()`: `isStaticOccupied` → `isOccupied` (dynamic obstacle visibility) |
| `trajectory_generation/src/plan_manager.cpp` | `replanFinding()`: path commitment + hysteresis + re-anchoring |
| `trajectory_generation/src/Astar_searcher.cpp` | `smoothTopoPath()`: occupied-cell collision handling |
| `trajectory_tracking/src/tracking_manager.cpp` | **arrival 0.1→0.3m, removed premature replan, debounce 25→15/15→10/6→4** |
| `HIT_Integrated_test/HIT_intergration_test.launch` | Speed reference defaults |
| `trajectory_generation/src/path_smooth.cpp` | **Fix 28: MINCO weight rebalancing, gradient clipping, safety guards; Fix 29: wFidelity 5e3→500, pathResample fix** |
| `trajectory_generation/src/reference_path.cpp` | **Fix 28: MAX_TRAJ_TIME=30s safety cap; Fix 29: local-time consistency fix in getRefTrajectory/getRefVel** |

---

## Fix 28: MINCO Weight Rebalancing & Safety Guards (Critical)

**Files modified:**
- `trajectory_generation/src/path_smooth.cpp` → `costFunction()`, `smoothPath()`
- `trajectory_generation/src/reference_path.cpp` → `getRefTrajectory()`

### Symptoms

After the MINCO Hessian fix (Post-Implementation Bug Fix in PLAN_1_MINCO_SPACETIME.md):
1. **Routes went into a crazy zigzag** — worse than before the Hessian fix
2. **RViz became extremely laggy then crashed** — OOM from trajectory marker data
3. **Continuous ROS log spam:** `"tracking time low!"` and `"[TrackingManager] solveNMPC failed, skipping this cycle"` repeating indefinitely

### Root Cause Analysis (5 interconnected causes)

#### Cause 1: Energy gradient explosion from correct Hessian

The prior session's Hessian fix was **mathematically correct** (4/4 gradient tests pass), but the correct Hessian produces much larger gradient magnitudes than the old (wrong) one. For a typical segment with $T \approx 0.15\text{s}$ and $\Delta p \approx 0.3\text{m}$:

$$\text{MINCO energy gradient} \propto \frac{720 \cdot \Delta p}{T^4} \approx \frac{720 \times 0.3}{0.15^4} \approx 427{,}000 \text{ per waypoint per axis}$$

At the initial weight `wSmooth = 0.1`:

$$\text{Weighted energy gradient} \approx 0.1 \times 427{,}000 = 42{,}700 \text{ per waypoint}$$

Meanwhile, the obstacle gradient (from `obstacleTerm()` with `wObstacle = 1e5` baked in) is typically:

$$\text{Obstacle gradient} \approx 2{,}000 \text{ per waypoint}$$

**The energy term was 20× larger than the obstacle term**, completely dominating L-BFGS. The optimizer aggressively moved waypoints to minimize jerk energy, ignoring obstacles.

#### Cause 2: Stale obstacle cache invalidation

`obstacleTerm()` computes obstacles at the **initial** waypoint positions via `getObsEdge()` (8-cell grid search + 100-ray cast within R=4.0m) and **never updates** the cache during optimization. As waypoints drifted >0.3m from their initial positions (pushed by the dominant energy gradient), the cached obstacle positions became irrelevant. Waypoints moved into actual obstacles that weren't in the cache.

#### Cause 3: Unlimited L-BFGS iterations

`lbfgs.hpp` defaults to `max_iterations = 0` (unlimited). With conflicting cost term gradients (energy pulling waypoints one way, stale obstacle pulling another), L-BFGS never converged — it ran indefinitely, blocking the planning loop.

#### Cause 4: Unbounded velocity post-processing inflation

After optimization, `smoothPath()` enforces velocity limits by inflating segment durations:
```
scale = maxVel / v_max_limit
T_opt(i) *= scale
```
For zigzag trajectories with extreme velocities (100+ m/s sampled), `scale` could reach 28×+. With no cap, a 0.15s segment could become 4.2s. Over many segments, total trajectory time could exceed 100s.

#### Cause 5: RViz crash from memory explosion

`getRefTrajectory()` in `reference_path.cpp` samples at `dt = 0.05` (20 Hz), generating `time/dt` trajectory points. With uncapped total trajectory time:

$$\text{Points} = \frac{100\text{s}}{0.05\text{s}} = 2{,}000{,}000$$

The CUBE_LIST marker with millions of points caused RViz to exhaust memory and crash.

### Gradient Magnitude Budget (Pre-Fix vs Post-Fix)

| Cost Term | Weight (old) | Gradient/wp (old) | Weight (new) | Gradient/wp (new) |
|-----------|-------------|-------------------|-------------|-------------------|
| Energy (jerk) | 0.1 | **~42,700** | 1e-3 | ~300 (+ clip at 2000) |
| Obstacle | 1e5 (baked) | ~2,000 | 1e5 (baked) | ~2,000 |
| Fidelity | 1e3 | ~200 (at 0.1m) | 5e3 | ~1,000 (at 0.1m) |
| Velocity | 1e4 | ~100 | 1e3 | ~10 |
| Min-time | 1e3 | ~100 | 1e3 | ~100 |

**Old regime:** Energy (42,700) >>> Obstacle (2,000) → optimizer ignores obstacles.
**New regime:** Obstacle (2,000) > Fidelity (1,000) > Energy (≤300) → obstacles dominate correctly.

### Fixes Applied

#### A. Weight rebalancing (`costFunction()`)

```cpp
const double wSmooth   = 1e-3;   // was 0.1 — reduces energy gradient from ~42,700 to ~300/wp
const double wFidelity = 5e3;    // was 1e3 — at 0.1m displacement: grad = 2×5e3×0.1 = 1,000
const double wVel      = 1e3;    // was 1e4 — velocity penalty is secondary
const double wMinTime  = 1e3;    // unchanged
```

#### B. Energy gradient clipping (`costFunction()`)

Per-waypoint energy gradient norm capped at 2000:
```cpp
const double energyGradClip = 2000.0;
for (int i = 0; i < N-1; i++) {
    double eg_norm = energy_grad.col(i).norm();
    if (eg_norm > energyGradClip) {
        energy_grad.col(i) *= energyGradClip / eg_norm;
    }
}
// Time gradient also clamped to ±2000 per entry
```

#### C. NaN/Inf guards (`costFunction()`)

1. After computing MINCO energy: if `!isfinite(energy)`, return `cost = 1e18` with zero gradient
2. After assembling final cost+gradient: if `!isfinite(cost) || !g.allFinite()`, set `cost = 1e18` and zero gradient

#### D. Bounded L-BFGS iterations (`smoothPath()`)

```cpp
lbfgs_params.max_iterations = 200;  // was 0 (unlimited)
// Also accept LBFGSERR_MAXIMUMITERATION as valid result
```

#### E. Waypoint drift clamping (`smoothPath()`)

After L-BFGS returns, clamp each waypoint to 0.5m radius from its reference position:
```cpp
const double clamp_radius = 0.5;
for (int i = 0; i < N-1; i++) {
    double drift = (wp_opt - refWaypoints[i+1]).norm();
    if (drift > clamp_radius) {
        wp_opt = refWaypoints[i+1] + (wp_opt - refWaypoints[i+1]) * (clamp_radius / drift);
    }
}
```
This keeps waypoints within the obstacle cache's validity radius.

#### F. Velocity inflation cap (`smoothPath()`)

```cpp
double scale = std::min(maxVel / v_max_limit, 5.0);  // cap at 5× inflation
```
Maximum time inflation per segment is 5×. A 0.15s segment can grow to at most 0.75s.

#### G. Per-segment time clamping (`smoothPath()`)

```cpp
const double T_lower = 0.01;
const double T_upper = 2.0;
if(T_opt(i) < T_lower) T_opt(i) = T_lower;
if(T_opt(i) > T_upper) T_opt(i) = T_upper;
```

#### H. Total trajectory time cap (`getRefTrajectory()`)

```cpp
const double MAX_TRAJ_TIME = 30.0;
if (time > MAX_TRAJ_TIME) {
    ROS_WARN("[RefTraj] total trajectory time %.1fs exceeds %.0fs cap, scaling down", time, MAX_TRAJ_TIME);
    double scale = MAX_TRAJ_TIME / time;
    for (int i = 0; i < m_numSegment; i++)
        m_time(i) *= scale;
    time = MAX_TRAJ_TIME;
}
```
Prevents generating more than `30 / 0.05 = 600` trajectory points.

#### I. Diagnostic logging (`smoothPath()`)

Added post-optimization diagnostic:
```
[Smoother] cost=XXX total_T=XXX max_drift=XXX max_vel=XXX segs=N
```

### Verification

| Test | Result |
|------|--------|
| `catkin_make --pkg trajectory_generation` | ✅ PASS (zero errors) |
| v_mid = 1.875 for 2-piece straight path | ✅ PASS |
| Waypoint gradient vs finite differences | ✅ PASS (max error 7e-9) |
| Time gradient vs finite differences | ✅ PASS (max error 3e-9) |
| Zigzag energy > straight energy | ✅ PASS (4.45×) |

### Mathematical Justification

The key insight is that MINCO's correct Hessian produces gradients with $1/T^4$ and $1/T^5$ scaling. With typical segment times $T \approx 0.15\text{s}$:

$$T^4 = 5.06 \times 10^{-4}, \quad T^5 = 7.59 \times 10^{-5}$$

Even a modest spatial displacement $\Delta p = 0.3\text{m}$ produces gradient terms of order $10^5$. The energy weight must be set extremely low ($\sim 10^{-3}$) to bring these values into the same range as the obstacle gradient ($\sim 2{,}000$ from `wObstacle = 10^5 \times \nabla d$).

The original pre-MINCO code (RM_Sentry_2026 backup) used a cubic spline energy at weight 1.0 — but cubic spline energy has $1/T$ scaling, not $1/T^4$, so a weight of 1.0 was fine. The transition to MINCO quintic requires a proportionally smaller weight.

### Known Remaining Limitation

The stale obstacle cache is a fundamental architectural issue. `obstacleTerm()` computes obstacles at initial waypoint positions and never updates them during L-BFGS optimization. The drift clamping (0.5m radius) is a band-aid that keeps waypoints close enough to their initial positions that the cache remains approximately valid. A proper fix would recompute obstacles every N iterations, but this is expensive ($O(N \times \text{raycast\_cost})$) and deferred to future work.

---

## Fix 29: Sharp Turns, Loops & MPC Deviation (Critical)

**Files modified:**
- `trajectory_generation/src/path_smooth.cpp` → `costFunction()`, `pathResample()`
- `trajectory_generation/src/reference_path.cpp` → `getRefTrajectory()`, `getRefVel()`

### Symptoms

After Fix 28 (no more RViz crashes), the trajectory still exhibited:
1. **Sharp turns** — the path preserved A* grid-aligned right-angle corners
2. **Meaningless loops and turns** — the cubic spline reference oscillated around waypoints
3. **MPC predicted path veered off from reference** — the tracker couldn't follow the looping reference

### Root Cause Analysis (3 causes)

#### Cause 1: wFidelity too high — corners can't be rounded

`wFidelity = 5e3` (from Fix 28) was set high to prevent waypoint drift beyond the obstacle cache radius. But it was *far too high* relative to the smoothing force:

| Force | Gradient at 0.1m displacement |
|-------|------------------------------|
| Fidelity (wFidelity=5e3) | `2 × 5e3 × 0.1 = **1,000**/wp` |
| Energy (wSmooth=1e-3) | `~400/wp` |
| Obstacle | `~2,000/wp` |

Energy (400) < Fidelity (1,000) → the optimizer could NOT move waypoints to smooth corners. The drift clamping at 0.5m is the actual hard safety mechanism — fidelity should be a soft bias, not a dominant force.

#### Cause 2: pathResample was broken — dead-code bug

`pathResample()` built a decimated `trajectory_point` (every 2nd waypoint) but then **discarded it**:
```cpp
// BUG: trajectory_point is computed but never used!
finalpath.assign(path.begin(), path.end());  // copies ALL waypoints
```

All ~17 waypoints (at 0.3m spacing for a 5m path) went to the cubic spline. Dense waypoints with sharp corners cause cubic spline **overshoot** (Runge phenomenon) — the cubic polynomial overshoots at sharp angular changes between closely-spaced control points, creating visible loops.

#### Cause 3: Inconsistent local-time evaluation in reference_path.cpp

The cubic spline position evaluation used inconsistent local times:
```cpp
// Cubic term used: (i * dt - total_time)
// Linear term used: (i - (int)(total_time / dt)) * dt  ← BUG
```

With MINCO's variable segment times, `total_time/dt` is often non-integer. Truncation via `(int)(...)` causes the linear term to differ from the cubic/quadratic terms by up to `dt`. Example:
- `total_time = 0.178`, `dt = 0.05`, `i = 4`
- Cubic local_t: `4×0.05 - 0.178 = 0.022`
- Linear local_t: `(4 - (int)(3.56)) × 0.05 = (4-3)×0.05 = 0.05`
- **Difference: 0.028** → position error `v × 0.028 ≈ 0.056m` at 2 m/s

This caused visible position jumps in the reference trajectory visualization and incorrect acceleration checks in `checkfeasible()`.

### Fixes Applied

#### A. wFidelity: 5e3 → 500 (`costFunction()`)

New gradient balance:
| Force | Gradient at 0.1m | Gradient at 0.3m |
|-------|-----------------|-----------------|
| Obstacle | ~2,000 | ~2,000 |
| Energy (wSmooth=1e-3) | ~400 | ~400 |
| Fidelity (wFidelity=500) | **100** | **300** |

Energy (400) > Fidelity (100) at corners → optimizer CAN round corners.
Drift clamp (0.5m) remains the hard safety net for obstacle cache validity.

#### B. Fix pathResample — proper decimation with time merging

Rewrote `pathResample()` to:
1. Build decimated control points (every 2nd waypoint via `keep_indices`)
2. Merge MINCO durations: each resampled segment's time = sum of spanned original segment times
3. Log maximum turning angle for diagnostics
4. Handle edge case: paths with ≤4 points are kept undecimated

```cpp
// Decimated: path[0], path[2], path[4], ..., path[last]
// Merged times: times[0]+times[1], times[2]+times[3], ...
finalpath.assign(trajectory_point.begin(), trajectory_point.end());
m_trapezoidal_time = merged_times;
```

Effect: ~17 → ~9 control points. Cubic spline segments go from 0.3m to 0.6m, dramatically reducing overshoot risk. MINCO-optimized times are preserved (just summed pairwise).

#### C. Fix reference_path.cpp local-time consistency

Replaced all instances of `(i - (int)(total_time / dt)) * dt` with `(i * dt - total_time)`:
```cpp
double local_t = i * dt - total_time;  // consistent across all terms
ref_point(0) = c0 * local_t³ + c1 * local_t² + c2 * local_t + c3;
```
Applied in both `getRefTrajectory()` (position) and `getRefVel()` (velocity). Also replaced `pow()` calls with direct multiplication for clarity and performance.

### Verification

| Test | Result |
|------|--------|
| `catkin_make --pkg trajectory_generation` | ✅ PASS (zero errors) |
| v_mid = 1.875 for 2-piece straight path | ✅ PASS |
| Waypoint gradient vs finite differences | ✅ PASS (max error 7e-9) |
| Time gradient vs finite differences | ✅ PASS (max error 3e-9) |
| Zigzag energy > straight energy | ✅ PASS (4.45×) |

### Weight Hierarchy (Updated)

$$\text{Obstacle} (2{,}000) > \text{Energy} (400) > \text{Fidelity} (100\text{--}300) > \text{MinTime} (100) > \text{Velocity} (10)$$

The energy term now dominates fidelity at corners, enabling smooth rounding. Obstacle still dominates everything for safety. Fidelity provides a gentle return-to-reference bias without preventing corner smoothing.

---

## Fix 30: Replan Oscillation — Dynamic Obstacle Zigzag (Critical)

**Files modified:**
- `trajectory_generation/include/replan_fsm.h` → added `last_replan_time` member
- `trajectory_generation/src/replan_fsm.cpp` → `checkReplanCallback()`, `execFSMCallback()`
- `trajectory_generation/src/path_smooth.cpp` → `costFunction()`, `smoothPath()`
- `trajectory_generation/include/path_smooth.h` → added `cost_call_count` member
- `trajectory_tracking/src/tracking_manager.cpp` → `checkReplanFlag()`

### Symptoms

After Fixes 28-29, the trajectory saw significant improvement for static navigation. However, near **live (LiDAR-detected) obstacles**, the route zigzagged abnormally — oscillating between different paths around the obstacle.

### Root Cause Analysis — The Smoking Gun (3 interacting causes)

#### Cause 1: Replan oscillation (the MAIN zigzag driver)

The replan chain near dynamic obstacles:
1. Dynamic obstacle appears → MPC predicted trajectory collides → `checkfeasible()` returns `true`
2. After 10+ collision flags in 15 frames (~0.15s) → `checkReplanFlag()` publishes `/replan_flag=true`
3. FSM transitions to `REPLAN_TRAJ` → calls `replanFinding()`
4. `replanFinding()` detects collision on `optimized_path` → calls `localPathFinding()` → **creates a NEW Topo PRM graph with RANDOM samples**
5. Each new PRM graph produces a **different** topological path around the obstacle
6. New path → `smoothTopoPath` → MINCO → new reference → MPC tracks it
7. But the obstacle is still there (or moved slightly) → collision detected again → goto step 2
8. **Each replan produces a DIFFERENT random path** → path alternates between routes → **zigzag**

#### Cause 2: No replan cooldown

After a successful replan, the FSM set `replan_flag=true` (member variable) to prevent the `checkReplanCallback` from immediately re-entering `REPLAN_TRAJ`. However, the tracker continuously publishes `/replan_flag` at ~100Hz. As soon as ONE `false` message arrived (no collision in that frame), `replan_flag` was reset to `false`, re-arming the trigger. With dynamic obstacles, collision detection flickered — one frame clear, next frame collision — so replans fired **every other FSM cycle** (~30ms).

#### Cause 3: Stale obstacle cache in MINCO smoother

`obstacleTerm()` cached obstacles at initial waypoint positions (`allobs[]`) and never updated during 200 L-BFGS iterations. As waypoints moved during optimization, the cached obstacles became irrelevant — the optimizer pushed waypoints using gradients from "ghost" obstacles at positions the waypoints had already left.

### Solution — 4 targeted changes

| Fix | Location | Change |
|-----|----------|--------|
| Replan cooldown | `replan_fsm.cpp` → `checkReplanCallback()` | **1.0s minimum cooldown** between replans. New replans within 1s of the last successful replan are rejected with `ROS_INFO_THROTTLE`. |
| Replan timestamp | `replan_fsm.cpp` → `execFSMCallback()` | `last_replan_time = ros::Time::now()` set after each successful replan in `REPLAN_TRAJ` state. |
| Obstacle cache refresh | `path_smooth.cpp` → `costFunction()` | **Refresh `allobs[]` and `mid_distance[]` every 50 L-BFGS iterations** — rebuilds obstacle cache at waypoints' CURRENT positions instead of stale initial positions. Counter reset in `smoothPath()`. Only the cache-building pass runs on refresh iterations; gradient computation uses the freshly rebuilt cache in the same call. |
| Wider collision window | `tracking_manager.cpp` → `checkReplanFlag()` | Window 15→**30** frames (0.3s), threshold 10→**20**. Same 67% ratio but over a longer window, reducing flicker-triggered replans near dynamic obstacles. |

### Key Code

```cpp
// replan_fsm.cpp — checkReplanCallback()
double elapsed = (ros::Time::now() - last_replan_time).toSec();
if(elapsed < 1.0){
    ROS_INFO_THROTTLE(0.5, "[FSM] replan cooldown: %.1fs remaining", 1.0 - elapsed);
    return;  // Reject replan request within cooldown
}
```

```cpp
// path_smooth.cpp — costFunction()
instance->cost_call_count++;
if(!instance->init_obs || (instance->cost_call_count % 50 == 0)) {
    instance->allobs.clear();
    instance->mid_distance.clear();
    instance->init_obs = false;
    for (int i = 0; i < points_num; ++i) {
        instance->obstacleTerm(i, inPs.col(i), dummy);  // rebuild cache
    }
    instance->init_obs = true;
}
```

### Verification

| Test | Result |
|------|--------|
| `catkin_make --pkg trajectory_generation` | ✅ PASS |
| `catkin_make --pkg tracking_node` | ✅ PASS |

---

## Fix 31: Zigzag Near Turns and Start/End Points

**Files modified:**
- `trajectory_generation/src/path_smooth.cpp` → `costFunction()`, `smoothPath()`, `getObsEdge()`

### Symptoms

After Fix 30, the route around dynamic obstacles improved significantly. However, **meaningless curves and zigzags persisted near turns and at destination/starting points**.

### Root Cause Analysis

#### Cause 1: wFidelity too high for corner rounding

`wFidelity = 500` (from Fix 29) anchored EVERY 0.3m-spaced waypoint to its A* reference position. At turns, the A* reference path has sharp corners. The MINCO smoother tried to round these corners (minimize jerk), but fidelity pulled every waypoint back to the corner. This tug-of-war created small oscillations/dimples at every turn:

| Force | Gradient at 0.1m displacement |
|-------|------------------------------|
| Fidelity (wFidelity=500) | `2 × 500 × 0.1 = 100/wp` |
| Energy (wSmooth=1e-3) | `~300/wp` |

Energy (300) > Fidelity (100) looks OK, but at corners the energy gradient always points "round the corner" while fidelity points "back to the sharp corner," creating oscillation. The ratio needs to be more decisively in favor of smoothing.

#### Cause 2: Drift clamp too tight

The 0.5m drift clamp (from Fix 28) physically prevented waypoints from rounding corners that required >0.5m of lateral displacement. This was originally necessary because the obstacle cache was stale. With Fix 30's periodic cache refresh, the clamp can be relaxed.

#### Cause 3: Obstacle search radius too small

`getObsEdge()` searched ±8 grid cells = ±0.4m. With the increased drift clamp, waypoints could move up to 0.8m from their initial position, beyond the obstacle search radius. Obstacles near the waypoint's final position weren't in the cache.

### Solution — 4 targeted changes

| Fix | Location | Change |
|-----|----------|--------|
| Reduce fidelity weight | `costFunction()` | `wFidelity`: 500 → **100** (gentle bias, not rigid anchor) |
| Increase smoothing weight | `costFunction()` | `wSmooth`: 1e-3 → **5e-3** (5× stronger jerk reduction) |
| Relax drift clamp | `smoothPath()` | Drift clamp: 0.5m → **0.8m** (allows corner rounding) |
| Widen obstacle search | `getObsEdge()` | Grid search: ±8 → **±12** cells (±0.6m at 0.05m/cell) |

### Weight Hierarchy (Updated)

$$\text{Obstacle} (2{,}000) > \text{Energy} (\leq 1{,}500) > \text{Fidelity} (200) > \text{MinTime} (100) > \text{Velocity} (10)$$

Energy now dominates fidelity at corners, enabling smooth rounding. Obstacle still dominates everything for safety. Fidelity provides a gentle return-to-reference bias without preventing corner smoothing.

### Verification

| Test | Result |
|------|--------|
| `catkin_make --pkg trajectory_generation` | ✅ PASS |

---

## Fix 32: Localization Floor Sinking Below Z=0

**Files modified:**
- `sim_nav/src/hdl_localization/apps/hdl_localization_nodelet.cpp` → `publish_odometry()`
- `sim_nav/src/bot_sim/launch_real/imu_filter.launch`
- `sim_nav/src/hdl_localization/launch/hdl_localization.launch`

### Symptoms

1. The real-time LiDAR point cloud floor appears below z=0 in RViz (in the `map` frame)
2. The `/odom` tf origin is at z=0, making the floor appear sunken
3. Related: PROBLEM_REPORT_LOCALIZATION_TILT.md documented an IMU mismatch fix that was never deployed

### Root Cause Analysis

**Three compounding causes:**

#### Cause 1: Z=0 clamp in `publish_odometry()` (Critical)

The `hdl_localization_nodelet.cpp` had:
```cpp
Eigen::Matrix4f pose = raw_pose;
pose(2, 3) = 0.0f;   // Force Z=0 in published pose
```

The PCD map floor is at z ≈ 0 (verified: 10th percentile z = -0.011m in `current.pcd`, 232k points). The LiDAR is mounted ~0.3m above ground. NDT correctly converges to z ≈ 0.3m (matching the PCD), but the clamp forces the published tf to z=0.

Result: The `map → aft_mapped` tf has z=0. The live point cloud in `aft_mapped` frame has floor at z ≈ -0.3m (LiDAR height above ground). After transformation to `map` frame with zero z-offset, the floor appears at z ≈ -0.3m instead of z ≈ 0.

#### Cause 2: IMU mismatch between mapping and localization

Documented in PROBLEM_REPORT_LOCALIZATION_TILT.md but **never deployed**:
- **Mapping** (Point-LIO): uses `/livox/imu_192_168_1_3` (right LiDAR IMU)
- **Localization** (hdl_localization): was using `/livox/imu_192_168_1_105_filtered` (left LiDAR IMU)

Different IMU gravity references introduce systematic tilt in the NDT initial guess each cycle, contributing to Z drift and floor appearance issues.

#### Cause 3: Missing `imu_filter_3` node

The `imu_filter.launch` only had one filter node (for `_105`). The `_3` IMU had no filtered topic available, preventing the IMU alignment fix.

### Solution — 3 changes

#### Part A: Remove Z=0 clamp
```cpp
// Before (BUG):
Eigen::Matrix4f pose = raw_pose;
pose(2, 3) = 0.0f;   // Force Z=0 — hides real LiDAR height

// After (FIX):
Eigen::Matrix4f pose = raw_pose;
// NDT determines real Z height: LiDAR at z≈0.3m → floor at z≈0 in map frame
```

#### Part B: Add `imu_filter_3` node
```xml
<!-- Before: only _105 filtered -->
<node pkg="bot_sim" type="imu_filter" name="imu_filter">
    <param name="input_imu_topic" value="/livox/imu_192_168_1_105" />
    <param name="output_imu_topic" value="/livox/imu_192_168_1_105_filtered" />
</node>

<!-- After: both IMUs filtered -->
<node pkg="bot_sim" type="imu_filter" name="imu_filter_105">...</node>
<node pkg="bot_sim" type="imu_filter" name="imu_filter_3">...</node>
```

#### Part C: Switch hdl_localization to mapping IMU
```xml
<!-- Before -->
<arg name="imu_topic" default="/livox/imu_192_168_1_105_filtered" />

<!-- After: matches Point-LIO mapping IMU source -->
<arg name="imu_topic" default="/livox/imu_192_168_1_3_filtered" />
```

### PCD Floor Verification

```
PCD: current.pcd (232,324 points)
Z min:   -39.89 (outliers)
Z 1st%:   -0.24
Z 5th%:   -0.02
Z 10th%:  -0.01  ← Floor level ≈ 0
Z median:  0.24
Z max:     3.59
```

### Expected Result

| Before Fix 32 | After Fix 32 |
|----------------|--------------|
| tf z = 0.0 (clamped) | tf z ≈ 0.3 (NDT-determined LiDAR height) |
| Floor at z ≈ -0.3m in map frame | Floor at z ≈ 0m in map frame |
| IMU: `_105` (left, different from mapping) | IMU: `_3` (right, same as mapping) |
| One IMU filter node | Two IMU filter nodes |

### Verification

| Test | Result |
|------|--------|
| `catkin_make --pkg hdl_localization` | ✅ PASS |

---

## Fix 33: Five-Part System-Wide Fix

**Files modified:**
- `trajectory_generation/src/plan_manager.cpp` → `replanFinding()`
- `trajectory_generation/src/visualization_utils.cpp` → `visTopoPointGuard()`, `visTopoPointConnection()`, `visTopoPath()`
- `trajectory_generation/src/path_smooth.cpp` → `costFunction()`
- `trajectory_tracking/src/local_planner.cpp` → solver initialization
- `trajectory_tracking/cfg/task.info` → R weight matrix
- `hdl_localization/apps/hdl_localization_nodelet.cpp` → `publish_odometry()`
- `hdl_localization/launch/hdl_localization.launch` → `init_pos_z`

### Part A: Cross-Track Deviation Replan Trigger

**Root cause:** The global trajectory never replanned when the robot veered far off the planned path. The existing replan triggers were:
1. Path collision detected (via `checkPathCollision`)
2. Target moved (endpoint distance > 0.5m)
3. Tracking-low flag from MPC tracker (robot >0.4m from first reference point, 4/10 frames)

**Why #3 failed:** When the robot exceeds 0.4m from the first reference point, `getFightTrackingTraj()` snaps the reference time to the closest-point time (`start_tracking_time` reset). This immediately reduces the distance to ~0, preventing `tracking_low_check_flag` from accumulating enough frames (4/10) to trigger `/replan_flag`.

**Why no path-deviation check existed:** `replanFinding()` only checked collision and target-moved. The "safe path + unchanged target" branch re-anchored the polynomial from the current position without checking HOW FAR the robot had drifted from the planned path.

**Fix:** Added cross-track deviation check in `replanFinding()`, BEFORE the re-anchoring logic:
```cpp
// Compute distance from robot to nearest waypoint on planned path
double min_dist = 1e9;
for (int i = 0; i < optimized_path.size(); i++) {
    double d = (optimized_path[i].head<2>() - start_point.head<2>()).norm();
    if (d < min_dist) min_dist = d;
}
if (min_dist > 1.0) {  // robot >1m from path — full replan
    pathFinding(start_point, target_point, start_vel);
    return true;
}
```

### Part B: Guard/Connection Point Visualization Z Doubling

**Root cause:** `visualization_utils.cpp` multiplied all topo graph point z-coordinates by 2:
```cpp
pt.z = (*iter)->pos(2) * 2;  // present in visTopoPointGuard, visTopoPointConnection, visTopoPath
```

This created two visible z-layers in RViz:
- Ground-level points (z ≈ 0.03–0.16) rendered at z ≈ 0.06–0.32
- Bridge-level points (z ≈ 0.36–0.40) rendered at z ≈ 0.72–0.80

Additionally, `scale.z = 0.5` made each cube extend ±0.25m vertically, further blurring the layers.

**Fix:**
- Removed all `* 2` multipliers (7 instances across 3 functions)
- Changed `scale.z` from `0.5` to `0.05` (guard) and `0.03` (connection) for proportional cubes

### Part C: /3DLidar Floor Sinking Below Z=0

**Root cause:** Fix 32 removed the z=0 hard clamp, allowing NDT to determine z freely. However:
1. The PCD floor was shifted to z=0 via `level_floor: true` in `avia.yaml` (Point-LIO `pcd_save` config)
2. The LiDAR is mounted ~0.3m above ground — the correct z in the shifted-PCD frame is ~0.3
3. `init_pos_z = 0.0` gave NDT a wrong initial guess
4. The ground filter in `livox_cloudpoint_processor` removes floor points from `/filted_topic_3d` (NDT input)
5. Without floor points, NDT cannot constrain z well — it stayed near init_pos_z = 0.0
6. Result: live cloud floor at z ≈ -0.3 in aft_mapped transforms to z ≈ -0.3 in map frame

**Fix (two changes):**
1. Set `init_pos_z = 0.3` in `hdl_localization.launch` — correct initial z for the shifted PCD frame
2. Added soft z-constraint in `publish_odometry()` — clamps z within ±0.15m of init_pos_z:
```cpp
float z_center = static_cast<float>(private_nh.param<double>("init_pos_z", 0.0));
float z_tolerance = 0.15f;
pose(2, 3) = std::max(z_center - z_tolerance, std::min(z_center + z_tolerance, pose(2, 3)));
```

This prevents NDT drift while allowing minor z adjustment. The floor should now appear at z ≈ 0 ± 0.15m in map frame.

### Part D: Sharp Turns in Global Trajectory (Weight Rebalance)

**Root cause:** Fix 31's wFidelity=100 was still too strong. With 0.3m waypoint spacing, each anchor point had a spring of strength 200 (2 × wFidelity × num_waypoints). The jerk energy (wSmooth=5e-3) was 20,000× weaker than fidelity per waypoint, making corner rounding impossible.

**Fix:**
| Weight | Fix 31 | Fix 33 | Ratio change |
|--------|--------|--------|--------------|
| wSmooth | 5e-3 | **2e-2** | ×4 |
| wFidelity | 100 | **20** | ÷5 |

$$\text{New ratio: } \frac{wFidelity}{wSmooth} = \frac{20}{0.02} = 1000 \text{ (was 20,000)}$$

Energy now has 20× more relative influence, enabling smooth corner rounding while fidelity still prevents large drift (clamped at 0.8m).

### Part E: MPC Predicted Path Divergence and Oscillation

**Root causes identified:**
1. Collision barrier `(3.5, 0.25)` was too aggressive — overpowered position tracking Q=150
2. `R(a) = 1.0` was too low — MPC used excessively aggressive acceleration to track positions
3. Q_pos/R_a = 150/1 = 150 (ratio too high → aggressive control → overshoot → oscillation)

**Fix (two changes):**
| Parameter | Before | After | Rationale |
|-----------|--------|-------|-----------|
| Collision barrier μ | 3.5 | **2.5** | Restores Fix 10 value; avoids overpowering Q |
| R(a) | 1.0 | **1.5** | Restores Fix 9 value; Q/R = 100 (smooth tracking) |

### Verification

| Test | Result |
|------|--------|
| `catkin_make --pkg trajectory_generation` | ✅ PASS |
| `catkin_make --pkg tracking_node` | ✅ PASS |
| `catkin_make --pkg hdl_localization` | ✅ PASS |
