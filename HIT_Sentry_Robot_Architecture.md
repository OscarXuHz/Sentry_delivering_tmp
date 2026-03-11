# HIT Sentry Robot Open-Source Technical Architecture
*Source: Harbin Institute of Technology (HIT) Sentry Robot Open-Source Technical Documentation*

---

## System Overview

The system is a full autonomous navigation stack for a competition sentry robot, consisting of three major subsystems:
1. **Global Path Planning** — Topological path search
2. **Trajectory Generation** — Trajectory optimization (MINCO-inspired)
3. **Local Trajectory Tracking** — Nonlinear MPC (NMPC)
4. **Perception & Map Processing** — 2.5D map with terrain handling

---

## 1. Global Path Planning — Topological Path Search

### Motivation
A* search only finds optimal paths under its heuristic function, not geometrically shortest paths. The competition field has many traversable paths between any two points, and some tasks require finding multiple viable paths. Therefore, a topological path search (inspired by fast-planner) is used as the global planner.

### Node Types
- **Guard Node (G):** Responsible for exploration. Can connect to multiple Connection nodes but NOT to other Guard nodes directly. Two Guard nodes must be mutually invisible or far apart.
- **Connection Node (C):** Connects exactly two Guard nodes. Has no adjacent Connection node neighbors.
- Each node also maintains a **non-connectable neighbor set** for recording unidirectional edges (e.g., stairs).

### Sampling Efficiency Enhancement
- A **skeleton extraction** is applied to the occupancy grid map to derive **lane centerlines**.
- Sampling is restricted to lane centerlines to greatly improve sampling efficiency (full-map random sampling was found to be too slow in practice).
- Additionally, points within a 6m radius of the lane center (lidar confidence range) are also sampled to handle dynamic obstacles.

### Global Path Search Pipeline
```
Start/End Points + Global Occupancy Map
       ↓
Random map sampling → point P
       ↓
Check visibility of P against all Guard nodes (count N)
   N=0: P becomes new Guard node
   N=1: Check neighborhood occupancy count
         → High occupancy (narrow terrain): Create new Guard node
         → Low occupancy + exist_visual > 2: Discard (redundant)
   N=2: Check if Q1,Q2 already connected
         → Already connected: Discard (redundant connection), exist_visual++
         → Not connected: P becomes Connection node linking Q1 and Q2
       ↓
Height check for connection validity
       ↓
Global Topological Map
       ↓
Dijkstra search → approximate shortest path
       ↓
Path simplification/pruning → Global Path
```

### Visibility Check Algorithm
1. Connect new point `p` to candidate Guard point `p2`.
2. Check for obstacles between them.
3. If no obstacle: perform step-wise sampling along the segment.
4. If no excessive height difference between consecutive samples → points are visible.
5. Any condition failure → not visible.

### Redundancy Handling
- If `exist_visual > 2` at end of visibility scan: new sample point is redundant → discard.
- If only one Guard is visible and occupancy count of neighborhood is high: create new Guard node (narrow corridor / underpass terrain).

### Height Check (Terrain-Aware Connectivity)
When creating a Connection node between `p` and Guard nodes `p1`, `p2`:
- Step-sample from `p→p1` and from `p1→p`.
- If height difference > threshold AND current point height < previous point height → **impassable upward terrain** (e.g., ascending stairs): do NOT add `p1` to `p`'s neighbor set.
- If height difference within range AND current > previous → **passable terrain change** (e.g., descending stairs, ramp): allowed.
- Effect: the planner naturally avoids going up stairs but can plan going down stairs or launching off ramps.

### Shortest Path & Multi-Path Extraction
- After building the topological map: **Dijkstra search** for approximate shortest path.
- For multiple viable paths: **depth-first search** + redundant path merging.
- **Path pruning** applied after: straightens winding segments (see fast-planner).

### Future Optimization Notes
- Dynamic lane centerline extraction (real-time update when static map is insufficient).
- Reduce topological map complexity to improve DFS performance for multi-path queries.

---

## 2. Trajectory Generation — Trajectory Optimization

### Input
Global path from topological search, discretized at uniform step length into `point_set` (used as initial values for optimization).

### Method: Cubic Spline Interpolation (MINCO-inspired modeling)

Each segment `i` is a cubic polynomial:

```
S_i(x)   = a_i + b_i(x - x_i) + c_i(x - x_i)^2 + d_i(x - x_i)^3
S_i'(x)  = b_i + 2c_i(x - x_i) + 3d_i(x - x_i)^2
S_i''(x) = 2c_i + 6d_i(x - x_i)
```

### Derivation of Spline Coefficients
From continuity conditions:
- `a_i = y_i`
- Step size: `h_i = x_{i+1} - x_i`
- Setting `m_i = S_i''(x_i) = 2c_i`:
  - `d_i = (m_{i+1} - m_i) / (6h_i)`
  - `b_i = (y_{i+1} - y_i)/h_i - (h_i/2)m_i - (h_i/6)(m_{i+1} - m_i)`
  - `c_i = m_i / 2`

This yields a **linear system** for unknowns `m`:

```
h_i * m_i + 2(h_i + h_{i+1}) * m_{i+1} + h_{i+1} * m_{i+2}
  = 6 * [(y_{i+2} - y_{i+1})/h_{i+1} - (y_{i+1} - y_i)/h_i]
```

The coefficient matrix is **tridiagonal, diagonally dominant** (dimension n+1), solvable efficiently.

### Boundary Conditions

**Natural boundary:** `m_0 = 0`, `m_n = 0`

**Clamped boundary** (given end derivatives A and B):
- `2h_0 * m_0 + h_0 * m_1 = 6[(y_1 - y_0)/h_0 - A]`
- `h_{n-1} * m_{n-1} + 2h_{n-1} * m_n = 6[B - (y_n - y_{n-1})/h_{n-1}]`

### Alternative Formulation (Direct Solve, No Intermediate Variables)
Using parameterization `x = a + bs + cs^2 + ds^3`, directly assemble a linear system from:
- Endpoint conditions: `P_0 = a_0`, `v_0 = b_0`, `P_n` at final segment end
- Velocity/acceleration continuity across segment boundaries
- Zero velocity/acceleration at terminal endpoint

Matrix size is `4(n+1)`, banded structure, directly solvable.

### MINCO Modeling Insight
MINCO demonstrates that all trajectory physical quantities are controlled by:
- **Control point positions P**
- **Per-segment time durations T**

During optimization, only P and T need to be adjusted to achieve: smoothing, obstacle avoidance, speed control at specific points.

**Implementation note:** The current code does NOT use full MINCO. It only optimizes control point positions (P), treats T as constant, and considers obstacle avoidance and smoothness. Full MINCO (joint space-time optimization) is a suggested future improvement.

### Obstacle Avoidance in Optimization
- Obstacle cost: L2 distance to surrounding occupied grid cells.
- Search radius: 1m around path (tunable; affects speed and avoidance quality).
- **Activation radius R = 0.3m**: only compute loss if obstacle is within R of path. This prevents path from being pulled into sparser-obstacle sides in narrow corridors.
- No explicit obstacle modeling (convex hull decomposition is NP-hard and reduces passable space).
- Additional 2.5m radius check via **raycast** to prevent optimizer from pushing points out of 1m radius and into obstacles beyond.
- Optimizer: **L-BFGS** (future suggestion: replace with Ceres solvers).

### Future Optimization Notes
- Joint space-time optimization (original MINCO): enables per-segment velocity/acceleration constraints (e.g., decelerate before underpass, slow on descent).
- Flight corridor (convex decomposition): better formal guarantee, OSQP-solvable, comparable practical passability.
- Replace L-BFGS with Ceres.

---

## 3. Local Trajectory Tracking — NMPC

### Framework: Tube-MPC
The robot kinematic model is simplified to a **two-wheel Ackermann model**:

```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
v̇ = a
θ̇ = w
```

**State:** position (x, y), linear velocity v, heading angle θ  
**Control input:** linear acceleration a, angular velocity w

### MPC Cost Function
```
J = Σ_{j=0}^{N} [ ||x(k+j|k) - x_ref(k+j)||_Q^2 + ||u(k+j|k)||_{Rε}^2 ]
```

### Safety Constraints
Obstacles within 1m of reference trajectory are treated as **hard constraints**:
```
||p(x(k+j|k)) - o_i||_P^2 > r^2
```

### Full NMPC Problem
```
min J(x_k, u_k)
s.t.  |x(k+j|k)| ≤ x_max
      ||p(x(k+j|k)) - o_i||_P^2 > r^2
```

### Solver
- **OCS2** (ETH Zürich large-scale MPC solver)
- **SQP solver** (Sequential Quadratic Programming): iterative line search solving multiple QP subproblems
- Note: Many OCS2 algorithms only support soft constraints; hard constraint handling requires care (Lagrange multiplier methods generally work).

### Motion Modes
The controller supports multiple modes triggered by game/chassis state:
- **Gyroscope mode** (spinning chassis while moving)
- **Reverse mode**
- **Anti-stuck (escape) mode**

**Anti-stuck mode detail:** If chassis speed remains very low over an extended period despite expected motion, the robot is deemed stuck. MPC's computed desired angular velocity/steering angle and forward speed are negated, simulating a reverse-and-turn escape maneuver, followed by triggering replanning.

### Future Optimization Notes
- Dynamic obstacle avoidance (currently not implemented; poor perception reliability made it infeasible).
- Flight corridor integration: if dynamic obstacle avoidance remains unreliable, switch to flight corridor + OSQP solver.
- Current soft-constraint approach: obstacle constraints only activated when reference trajectory is inside an obstacle, reducing computation but sacrificing collision-free guarantee when tracking error is large.

---

## 4. Obstacle Avoidance & Replanning — Three-Stage Replanning Logic

### Architecture
```
NMPC Dynamic Avoidance
    ↓
Local predicted trajectory safety check
    → No collision: continue
    → Collision detected: trigger Local Trajectory Replanning
        → Find colliding segment in global path
        → Local topological re-search for that segment
        → Success: merge local path into global path → trajectory re-optimization
        → Failure: trigger Global Trajectory Replanning
```

### Design Rationale
- Frequent replanning on multi-solution trajectory systems risks **topological inconsistency** between successive plans.
- Three-stage design minimizes unnecessary replanning while handling progressively severe scenarios.
- Soft constraints preferred over flight corridor to prioritize **motion continuity** over conservatism.

---

## 5. Perception & Map Processing — 2.5D Map with Terrain Handling

### Map Representation: 2.5D
- Pure 2D grid: insufficient for multi-terrain (stairs, ramps, underpasses).
- Full 3D map: redundant and computationally expensive for ground robot.
- **Solution:** 2D occupancy grid augmented with a **height map** (prior height per cell).

### Dynamic Obstacle Detection
Using real-time LiDAR point cloud + depth camera confidence point cloud:
```
δ_min < z1 - z < δ_max  →  obstacle present at this cell
```
- `z1`: prior height from built map
- `z`: current measured height
- Points too high or too low relative to prior are considered passable (e.g., ceiling, ground noise).

### Map Processing Pipeline
```
Post-mapping point cloud
    ↓ height correction + z-axis projection
Height map
    ↓ edge detection + dilation
Occupancy grid
    ↓ skeleton extraction
Lane centerline map
    ↓ topological search
Topological map (handles stairs, ramps, slopes correctly)
```

### Underpass (Bridge) Handling
**Problem:** Underpass cells are non-occupied (passable), but height check would incorrectly fail them.

**Detection method:** Underpass entrance/exit cells are non-occupied, but their neighborhood contains large height differences → mismatch between occupancy and height variation uniquely identifies underpass boundaries.

**Solution:** Each grid cell is given two additional attributes:
- `has_second_height`: bool
- `second_height`: float

When sampling reaches an underpass area during topological search, both the **above-bridge** and **below-bridge** points undergo a topo connectivity check simultaneously.

### Future Optimization Notes
- Full 3D perception: main challenge is map degradation when dynamic objects (people, robots) are present during mapping → need to filter moving objects during map building, or use processed 2D as prior for 3D.
- Better adaptive underpass identification: current method only reliably detects entrance/exit points; connecting the underpass region into a coherent topological structure remains unsolved.

---

## System Integration Summary

| Module | Algorithm | Solver/Tool | Key Parameters |
|--------|-----------|-------------|---------------|
| Global Path Search | Topological PRM (fast-planner inspired) | Dijkstra + DFS | Lane centerline sampling, 6m lidar radius |
| Trajectory Optimization | Cubic spline + MINCO-inspired control point opt. | L-BFGS | R=0.3m activation, 1m search radius |
| Local Tracking | Nonlinear MPC (Tube-MPC) | OCS2/SQP | N-step horizon, 1m obstacle hard constraint |
| Replanning | 3-stage: NMPC → local topo re-search → global replan | — | Topological consistency priority |
| Perception | 2.5D height map + LiDAR/depth camera fusion | — | δ_min/δ_max height thresholds |

## Key References
- **fast-planner**: Topological path search framework
- **MINCO**: Multi-segment polynomial trajectory modeling (space-time joint optimization)
- **OCS2**: ETH Zürich open-source MPC/optimal control solver
- HIT 2023 Youth Engineering Conference: 2.5D perception method
