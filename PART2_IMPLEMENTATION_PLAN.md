# Part 2 Implementation Plan: Dynamics and Control

## Executive Summary

This document provides a comprehensive analysis of the lab implementation status and a detailed roadmap for completing Part 2 (Dynamics). The analysis confirms that **Part 1 is fully implemented** (up to II.c), and this plan focuses on implementing Part 2, starting with **Task I: From Path to Trajectory**.

---

## Table of Contents

1. [Part 1 Verification Results](#part-1-verification)
2. [Part 2 Overview and Requirements](#part-2-overview)
3. [Detailed Implementation Plan for Part 2 Task I](#task-i-implementation)
4. [Detailed Implementation Plan for Part 2 Task II](#task-ii-implementation)
5. [Relevant Tutorials and Resources](#tutorials)
6. [Pseudocode and Algorithms](#pseudocode)
7. [Team Collaboration Strategy](#collaboration)
8. [Common Questions and Answers](#qa)

---

<a name="part-1-verification"></a>
## 1. Part 1 Verification Results

### Status: ✅ FULLY IMPLEMENTED

#### Task I: Computing Target Configurations (Inverse Geometry)

**File:** `inverse_geometry.py`

**Implementation Status:** ✅ COMPLETE

**What's Implemented:**
- `computeqgrasppose()` function (lines 21-80)
- Uses nullspace projection to control both arms simultaneously
- Iterative inverse kinematics with Jacobians
- Collision checking and joint limit validation
- Returns (configuration, success_flag)

**Evidence:**
```python
# Key implementation details:
- Uses pin.log() for SE3 error computation
- Nullspace projection: P = I - pinv(J_right) @ J_right
- Secondary task: vq += pinv(J_left @ P) @ (-error_left - J_left @ vq)
- Convergence threshold: EPSILON = 1e-3
```

**Git Evidence:** Commits show "Task 1 complete" and "Task 1 bug fix"

---

#### Task II.a: Sampling Configurations

**File:** `path.py`

**Implementation Status:** ✅ COMPLETE

**What's Implemented:**
- `RAND_CONF()` function (lines 29-48)
- Samples random cube placements near target
- Generates valid robot configurations via `computeqgrasppose`
- Recursive retry on failure

**Key Features:**
- Bounded sampling near target configuration
- Position-based sampling with rotation preserved
- Success validation before returning

---

#### Task II.b: Path Projection

**File:** `path.py`

**Implementation Status:** ✅ COMPLETE

**What's Implemented:**
- `NEW_CONF()` function (lines 72-89)
- Linear interpolation (lerp) between cube positions
- Discretized path validation
- Returns farthest collision-free configuration

**Key Features:**
- Delta_q limiting for incremental growth
- Discretized validation at each step
- Early termination on collision/failure

---

#### Task II.c: Solution Path Generation

**File:** `path.py`

**Implementation Status:** ✅ COMPLETE

**What's Implemented:**
- `computepath()` function (lines 97-119)
- RRT-style sampling-based planner
- Graph structure with parent pointers
- Path extraction via `getpath()`

**Algorithm:**
1. Initialize graph G with q0
2. Sample random configurations (k iterations)
3. Find nearest vertex
4. Extend toward sample
5. Check if goal reachable
6. Extract and return path

**Git Evidence:** Commit "part 1, task 1,2" indicates completion

---

<a name="part-2-overview"></a>
## 2. Part 2 Overview and Requirements

### Current Status: ❌ NOT IMPLEMENTED

**File:** `control.py`

### What Exists (Placeholders):
1. Basic Bezier trajectory example (doesn't follow path)
2. Empty `controllaw()` function
3. Defined gains (Kp=300, Kv=2*sqrt(Kp)) but not used

### What's Needed:

#### Task I: From Path to Trajectory
Convert the geometric path (list of configurations) into a time-parametrized trajectory with:
- Position trajectory q(t)
- Velocity trajectory vq(t)
- Acceleration trajectory aq(t)

**Three implementation levels:**
- **Level 0:** Manual time parametrization (basic, gets you started)
- **Level 1:** QP-based trajectory optimization (recommended, bonus marks)
- **Level 2:** Collision-aware trajectory optimization (advanced, more bonus marks)

#### Task II: Implementing Control Law
Design and implement a torque control law to track the trajectory:
- Read robot state from PyBullet
- Compute control torques
- Apply torques to simulator
- Handle grasping constraints

---

<a name="task-i-implementation"></a>
## 3. Detailed Implementation Plan: Task I - Path to Trajectory

### Recommended Approach: Start with Level 0, then Level 1

---

### LEVEL 0: Manual Time Parametrization

**Goal:** Create a simple time-parametrized trajectory that follows your path.

**Why Start Here:**
1. Quick to implement (1-2 hours)
2. Allows you to test control law immediately
3. Provides baseline for comparison
4. Builds intuition for trajectory behavior

**Implementation Strategy:**

#### Step 0.1: Understand Your Path
```python
# Your path is a list of configurations
path = [q0, q1, q2, ..., qn]  # from computepath()
```

#### Step 0.2: Assign Time to Each Waypoint
```python
def assign_times(path, segment_duration=1.0):
    """
    Assign time stamps to path waypoints

    Args:
        path: list of configurations [q0, q1, ..., qn]
        segment_duration: time between waypoints (seconds)

    Returns:
        times: list of time stamps [t0, t1, ..., tn]
    """
    n = len(path)
    times = [i * segment_duration for i in range(n)]
    return times
```

**Alternative: Distance-based timing**
```python
def assign_times_by_distance(path):
    """
    Assign times proportional to configuration distance
    Slower motion through tight spaces
    """
    times = [0.0]
    v_max = 0.5  # max velocity (rad/s)

    for i in range(1, len(path)):
        dist = np.linalg.norm(path[i] - path[i-1])
        dt = dist / v_max
        times.append(times[-1] + dt)

    return times
```

#### Step 0.3: Interpolate Between Waypoints with Bezier Curves
```python
def create_trajectory_level0(path, total_time):
    """
    Create trajectory using cubic Bezier curves between waypoints
    Ensures zero velocity at start/end
    
    when the first two control points are the same, and the last two are the same, the tangent (derivative) at both ends is zero → no velocity at the endpoints.

    Args:
        path: list of configurations
        total_time: total trajectory duration

    Returns:
        q_traj: function q(t) -> configuration
        v_traj: function v(t) -> velocity
        a_traj: function a(t) -> acceleration
    """
    n = len(path)
    segment_time = total_time / (n - 1)

    # Create piecewise Bezier curves
    segments = []
    for i in range(n - 1):
        q0 = path[i]
        q1 = path[i + 1]

        # Cubic Bezier with zero velocity endpoints
        # Control points: [q0, q0, q1, q1]
        bezier = Bezier([q0, q0, q1, q1],
                       t_min=i*segment_time,
                       t_max=(i+1)*segment_time)
        segments.append(bezier)

    def q_traj(t):
        # Find which segment t belongs to
        idx = min(int(t / segment_time), n - 2)
        return segments[idx](t)

    def v_traj(t):
        idx = min(int(t / segment_time), n - 2)
        return segments[idx].derivative(1)(t)

    def a_traj(t):
        idx = min(int(t / segment_time), n - 2)
        return segments[idx].derivative(2)(t)

    return q_traj, v_traj, a_traj
```

#### Step 0.4: Improved Version - Smooth Velocities
```python
def create_trajectory_level0_smooth(path, total_time):
    """
    Smoother trajectory with matched velocities at waypoints
    Uses cubic Bezier with velocity continuity
    """
    n = len(path)
    segment_time = total_time / (n - 1)

    # Compute velocity at each waypoint (finite differences)
    velocities = [np.zeros_like(path[0])]  # zero at start
    for i in range(1, n - 1):
        v = (path[i + 1] - path[i - 1]) / (2 * segment_time)
        velocities.append(v)
    velocities.append(np.zeros_like(path[-1]))  # zero at end

    # Create Bezier segments with matched velocities
    segments = []
    for i in range(n - 1):
        q0 = path[i]
        q1 = path[i + 1]
        v0 = velocities[i]
        v1 = velocities[i + 1]

        # Control points from Hermite -> Bezier conversion
        dt = segment_time
        cp0 = q0
        cp1 = q0 + dt * v0 / 3.0
        cp2 = q1 - dt * v1 / 3.0
        cp3 = q1

        bezier = Bezier([cp0, cp1, cp2, cp3],
                       t_min=i*segment_time,
                       t_max=(i+1)*segment_time)
        segments.append(bezier)

    # Return trajectory functions (same as before)
    ...
```

**Pros of Level 0:**
- Fast to implement
- Works immediately
- No optimization needed
- Easy to debug

**Cons of Level 0:**
- No velocity/acceleration limits
- Suboptimal timing
- May violate physical constraints
- No optimality guarantees

---

### LEVEL 1: QP-Based Trajectory Optimization

**Goal:** Optimize trajectory to minimize acceleration while respecting constraints.

**Why This Level:**
1. **Better performance** - Smoother, faster trajectories
2. **Respects constraints** - Joint limits, velocity/acceleration bounds
3. **Bonus marks** - Shows advanced understanding
4. **Still tractable** - Convex optimization, guaranteed solution

**Recommended for:** Students aiming for strong marks (70-85%)

#### Mathematical Formulation

**Decision Variables:** Control points of Bezier curve
```
x = [p0, p1, p2, ..., pN]  # N control points, each in R^nq
```

**Cost Function:** Minimize squared accelerations
```
min  ∫₀ᵀ ||aq(t)||² dt
```

For Bezier curves, this becomes:
```
min  Σᵢ ||pᵢ₊₂ - 2pᵢ₊₁ + pᵢ||²
```

**Constraints:**

1. **Path following:** Trajectory must pass through path waypoints
```
q(tᵢ) = path[i]  for i = 0, 1, ..., M
```

2. **Boundary conditions:** Zero velocity at start/end
```
p0 = p1  (zero initial velocity)
pN-1 = pN  (zero final velocity)
```

3. **Acceleration bounds:**
```
||pᵢ₊₂ - 2pᵢ₊₁ + pᵢ|| ≤ a_max
```

4. **Velocity bounds:**
```
||pᵢ₊₁ - pᵢ|| ≤ v_max * dt
```

#### Implementation Strategy

**Step 1.1: Set Up the Optimization Problem**

```python
import cvxpy as cp
import numpy as np

def optimize_trajectory_qp(path, T, N=50):
    """
    Optimize trajectory using QP

    Args:
        path: list of waypoint configurations
        T: total time duration
        N: number of control points

    Returns:
        control_points: optimized Bezier control points
        cost: optimization cost
    """
    nq = len(path[0])  # configuration dimension

    # Decision variables: N control points
    P = cp.Variable((N, nq))

    # Build cost function: minimize acceleration
    cost = 0
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        cost += cp.sum_squares(accel)

    constraints = []

    # Boundary conditions: zero velocity
    constraints.append(P[0] == P[1])
    constraints.append(P[N-1] == P[N-2])

    # Pass through waypoints
    M = len(path)
    for i, waypoint in enumerate(path):
        # Map waypoint index to control point index
        t = i / (M - 1)  # normalized time
        idx = int(t * (N - 1))
        constraints.append(P[idx] == waypoint)

    # Velocity limits (optional)
    dt = T / (N - 1)
    v_max = 2.0  # rad/s
    for i in range(N - 1):
        vel = (P[i+1] - P[i]) / dt
        constraints.append(cp.norm(vel) <= v_max)

    # Solve
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, verbose=True)

    if prob.status != cp.OPTIMAL:
        print(f"Warning: Optimization status: {prob.status}")

    return P.value, prob.value
```

**Step 1.2: Handle Path Waypoints Properly**

The key challenge: Your path has M waypoints, but you want N control points (N > M).

**Solution A: Interpolate path to N points first**
```python
def interpolate_path(path, N):
    """
    Interpolate path to N evenly-spaced configurations
    """
    M = len(path)
    t_path = np.linspace(0, 1, M)
    t_interp = np.linspace(0, 1, N)

    # Linear interpolation for each joint
    path_interp = []
    for i in range(N):
        t = t_interp[i]
        # Find surrounding waypoints
        idx = np.searchsorted(t_path, t)
        if idx == 0:
            path_interp.append(path[0])
        elif idx >= M:
            path_interp.append(path[-1])
        else:
            # Linear interpolation
            t0, t1 = t_path[idx-1], t_path[idx]
            alpha = (t - t0) / (t1 - t0)
            q_interp = (1-alpha)*path[idx-1] + alpha*path[idx]
            path_interp.append(q_interp)

    return path_interp
```

**Solution B: Soft constraints (recommended)**
```python
def optimize_trajectory_qp_soft(path, T, N=50, weight_path=1000.0):
    """
    Optimize with soft path-following constraints
    Allows small deviations for smoother trajectories
    """
    nq = len(path[0])
    P = cp.Variable((N, nq))

    # Acceleration cost
    cost_accel = 0
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        cost_accel += cp.sum_squares(accel)

    # Path-following cost (soft constraint)
    cost_path = 0
    M = len(path)
    for i, waypoint in enumerate(path):
        t = i / (M - 1)
        idx = int(t * (N - 1))
        cost_path += cp.sum_squares(P[idx] - waypoint)

    # Combined cost
    cost = cost_accel + weight_path * cost_path

    # Hard constraints: boundary conditions only
    constraints = [
        P[0] == path[0],      # start at q0
        P[N-1] == path[-1],   # end at qgoal
        P[0] == P[1],         # zero initial velocity
        P[N-1] == P[N-2]      # zero final velocity
    ]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)

    return P.value, prob.value
```

**Step 1.3: Create Trajectory from Control Points**

```python
def control_points_to_trajectory(control_points, T):
    """
    Convert optimized control points to trajectory functions

    Args:
        control_points: array of shape (N, nq)
        T: total time

    Returns:
        q_traj, v_traj, a_traj: trajectory functions
    """
    bezier = Bezier(list(control_points), t_min=0.0, t_max=T)
    v_bezier = bezier.derivative(1)
    a_bezier = bezier.derivative(2)

    return bezier, v_bezier, a_bezier
```

**Step 1.4: Time Optimization (Optional)**

So far we've assumed fixed time T. We can also optimize T!

**Trick:** Use τ = t² scaling to handle acceleration constraints linearly.

```python
def optimize_trajectory_with_time(path, N=50):
    """
    Optimize trajectory and total time jointly
    Uses τ = t² trick to keep problem convex
    """
    nq = len(path[0])

    # Decision variables
    P = cp.Variable((N, nq))
    tau = cp.Variable()  # tau = T²

    # Acceleration cost (scaled by time)
    cost_accel = 0
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        cost_accel += cp.sum_squares(accel) / tau

    # Want fast trajectory: minimize time
    cost = cost_accel + 0.1 * tau  # trade-off parameter

    constraints = [
        # Boundary conditions
        P[0] == path[0],
        P[N-1] == path[-1],
        P[0] == P[1],
        P[N-1] == P[N-2],

        # Time is positive
        tau >= 1.0,  # minimum 1 second
        tau <= 100.0  # maximum 10 seconds (tau = T²)
    ]

    # Acceleration constraints: ||a|| ≤ a_max
    # With τ scaling: ||pᵢ₊₂ - 2pᵢ₊₁ + pᵢ|| / τ ≤ a_max
    a_max = 10.0
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        constraints.append(cp.norm(accel) <= a_max * tau)

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)

    T_optimal = np.sqrt(tau.value)
    return P.value, T_optimal
```

#### Complete Level 1 Implementation

```python
def create_trajectory_level1(path, T=None, N=50):
    """
    Complete QP-based trajectory optimization

    Args:
        path: list of configurations from computepath()
        T: total time (if None, will be optimized)
        N: number of Bezier control points

    Returns:
        q_traj, v_traj, a_traj: trajectory functions
        T: total time
    """
    # Step 1: Optimize control points (and time if T=None)
    if T is None:
        control_points, T = optimize_trajectory_with_time(path, N)
    else:
        control_points, _ = optimize_trajectory_qp_soft(path, T, N)

    # Step 2: Create Bezier trajectory
    q_traj, v_traj, a_traj = control_points_to_trajectory(control_points, T)

    # Step 3: Validate trajectory
    print(f"Trajectory validation:")
    print(f"  Total time: {T:.2f}s")
    print(f"  Start config: {q_traj(0.0)}")
    print(f"  End config: {q_traj(T)}")
    print(f"  Start velocity: {np.linalg.norm(v_traj(0.0))}")
    print(f"  End velocity: {np.linalg.norm(v_traj(T))}")

    return q_traj, v_traj, a_traj, T
```

**Pros of Level 1:**
- Optimal smooth trajectories
- Respects constraints
- Bonus marks (significant)
- Still convex (guaranteed solution)

**Cons of Level 1:**
- More complex to implement
- Need to understand QP
- Debugging can be tricky
- May need parameter tuning

---

### LEVEL 2: Collision-Aware Trajectory Optimization

**Warning:** Only attempt if Level 1 is working and you want exceptional marks (85%+)

**Goal:** Optimize trajectory while explicitly avoiding collisions.

**Challenge:** Collision constraints are non-convex!

**Strategy:** Use non-linear programming (NLP) with collision constraints.

```python
from scipy.optimize import minimize

def optimize_trajectory_nlp(path, T, N=50):
    """
    NLP-based trajectory optimization with collision avoidance

    Non-convex problem: may get stuck in local minima
    Use Level 1 solution as initialization
    """
    nq = len(path[0])

    # Initial guess: use Level 1 solution
    P_init, _ = optimize_trajectory_qp_soft(path, T, N)
    x0 = P_init.flatten()

    def cost(x):
        P = x.reshape((N, nq))

        # Acceleration cost
        cost_accel = 0
        for i in range(N - 2):
            accel = P[i+2] - 2*P[i+1] + P[i]
            cost_accel += np.sum(accel**2)

        # Collision penalty
        cost_collision = 0
        for i in range(N):
            q = P[i]
            if collision(robot, q):
                cost_collision += 1000.0  # large penalty
            else:
                # Distance-based penalty
                d = distanceToObstacle(robot, q)
                if d < 0.05:  # safety margin
                    cost_collision += 100.0 / (d + 0.01)

        return cost_accel + cost_collision

    # Constraints
    constraints = [
        # Boundary conditions
        {'type': 'eq', 'fun': lambda x: x[:nq] - path[0]},  # start
        {'type': 'eq', 'fun': lambda x: x[-nq:] - path[-1]},  # end
        # Zero velocities
        {'type': 'eq', 'fun': lambda x: x[:nq] - x[nq:2*nq]},
        {'type': 'eq', 'fun': lambda x: x[-nq:] - x[-2*nq:-nq]},
    ]

    result = minimize(cost, x0, method='SLSQP', constraints=constraints)

    P_opt = result.x.reshape((N, nq))
    return P_opt, result.fun
```

**Important:** This is advanced and may not be necessary. The instructor suggests that "if you track your path well enough you will avoid collisions" - so Level 1 should be sufficient!

---

### Recommended Progression for Task I

**Week 1:**
1. Implement Level 0 (manual parametrization)
2. Test with control law
3. Verify robot can track trajectory

**Week 2:**
1. Study Tutorial 6 and 7 on QP
2. Implement Level 1 (QP optimization)
3. Compare with Level 0

**Week 3 (Optional):**
1. If time permits and marks are needed
2. Attempt Level 2 (collision-aware)
3. Focus on report instead if Level 1 works well

---

<a name="task-ii-implementation"></a>
## 4. Detailed Implementation Plan: Task II - Control Law

### Goal: Track the trajectory using torque control in PyBullet

**File to Edit:** `control.py`

---

### Understanding the Control Loop

The lab uses a **step-by-step simulation** approach:

```python
# Main control loop (from control.py main)
tcur = 0.0
while tcur < total_time:
    rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
    tcur += DT
```

At each time step:
1. **Get state** from PyBullet: `q, vq = sim.getpybulletstate()`
2. **Compute desired state** from trajectory: `q_des = q_traj(t)`
3. **Compute control torques**: `tau = f(q, vq, q_des, ...)`
4. **Apply torques** to simulator: `sim.step(torques)`

---

### Control Law Options

#### Option 1: PD Control with Gravity Compensation (Recommended)

**Theory:**
```
τ = Kp * (q_des - q) + Kd * (vq_des - vq) + g(q)
```

where:
- `Kp`: proportional gains (stiffness)
- `Kd`: derivative gains (damping)
- `g(q)`: gravity and Coriolis terms

**Why this works:**
- Simple and robust
- Provides stable tracking
- Compensates for gravity
- Good enough for most tasks

**Implementation:**

```python
import pinocchio as pin

def controllaw(sim, robot, trajs, tcurrent, cube):
    """
    PD control with gravity compensation

    Args:
        sim: PyBullet simulation object
        robot: Pinocchio robot model
        trajs: (q_traj, v_traj, a_traj) trajectory functions
        tcurrent: current time
        cube: cube object (for visualization)
    """
    # Unpack trajectories
    q_traj, v_traj, a_traj = trajs

    # Get current state from simulator
    q, vq = sim.getpybulletstate()

    # Get desired state from trajectory
    q_des = q_traj(tcurrent)
    vq_des = v_traj(tcurrent)

    # Compute errors
    error_pos = q_des - q
    error_vel = vq_des - vq

    # PD control gains
    Kp = 300.0
    Kd = 2 * np.sqrt(Kp)  # critical damping

    # Compute gravity compensation term
    # pin.nle() computes non-linear effects: Coriolis + gravity
    pin.computeAllTerms(robot.model, robot.data, q, vq)
    g = pin.nle(robot.model, robot.data, q, vq)

    # Control law
    torques = Kp * error_pos + Kd * error_vel + g

    # Apply torques
    sim.step(torques)
```

**Pros:**
- Simple to implement
- Stable and robust
- Works for most cases

**Cons:**
- May have tracking errors
- Doesn't explicitly handle dynamics
- May struggle with fast motions

---

#### Option 2: Inverse Dynamics Control (Recommended for Better Performance)

**Theory:**
```
τ = M(q) * aq_cmd + h(q, vq)
where:
  aq_cmd = aq_des + Kp*(q_des - q) + Kd*(vq_des - vq)
```

where:
- `M(q)`: mass matrix (inertia)
- `h(q, vq)`: non-linear effects (Coriolis + gravity)
- `aq_des`: desired acceleration from trajectory

**Why this works:**
- Exact feedback linearization
- Better tracking performance
- Handles dynamics explicitly
- Still simple to implement

**Implementation:**

```python
def controllaw_inverse_dynamics(sim, robot, trajs, tcurrent, cube):
    """
    Inverse dynamics control (computed torque control)

    Better tracking than PD, but more computation
    """
    q_traj, v_traj, a_traj = trajs

    # Current state
    q, vq = sim.getpybulletstate()

    # Desired state
    q_des = q_traj(tcurrent)
    vq_des = v_traj(tcurrent)
    aq_des = a_traj(tcurrent)

    # Errors
    error_pos = q_des - q
    error_vel = vq_des - vq

    # Control gains
    Kp = 300.0
    Kd = 2 * np.sqrt(Kp)

    # Commanded acceleration (with feedback)
    aq_cmd = aq_des + Kp * error_pos + Kd * error_vel

    # Compute mass matrix and non-linear effects
    pin.computeAllTerms(robot.model, robot.data, q, vq)
    M = pin.crba(robot.model, robot.data, q)  # mass matrix
    h = pin.nle(robot.model, robot.data, q, vq)  # Coriolis + gravity

    # Inverse dynamics
    torques = M @ aq_cmd + h

    # Apply torques
    sim.step(torques)
```

**Pros:**
- Excellent tracking
- Theoretically optimal
- Handles fast motions well

**Cons:**
- Requires mass matrix computation
- More sensitive to model errors
- Slightly more complex

---

#### Option 3: Inverse Dynamics + Force Control (For Grasping)

**Goal:** Apply additional forces to grasp the cube.

**Theory:**
```
τ = M(q) * aq_cmd + h(q, vq) + J^T * F_grasp
```

where:
- `J`: Jacobian of end-effector
- `F_grasp`: desired grasping force (3D or 6D)

**Why this helps:**
- Maintains contact with cube
- Compensates for positioning errors
- Attracts hands to cube naturally

**Implementation:**

```python
def controllaw_with_grasping(sim, robot, trajs, tcurrent, cube):
    """
    Inverse dynamics + force control for grasping
    """
    q_traj, v_traj, a_traj = trajs

    # Current state
    q, vq = sim.getpybulletstate()

    # Desired state (baseline control)
    q_des = q_traj(tcurrent)
    vq_des = v_traj(tcurrent)
    aq_des = a_traj(tcurrent)

    error_pos = q_des - q
    error_vel = vq_des - vq

    Kp = 300.0
    Kd = 2 * np.sqrt(Kp)
    aq_cmd = aq_des + Kp * error_pos + Kd * error_vel

    # Inverse dynamics baseline
    pin.computeAllTerms(robot.model, robot.data, q, vq)
    M = pin.crba(robot.model, robot.data, q)
    h = pin.nle(robot.model, robot.data, q, vq)

    tau_baseline = M @ aq_cmd + h

    # --- Force control for grasping ---

    # Get current end-effector positions
    pin.framesForwardKinematics(robot.model, robot.data, q)
    IDX_LEFT = robot.model.getFrameId(LEFT_HAND)
    IDX_RIGHT = robot.model.getFrameId(RIGHT_HAND)

    oMleft = robot.data.oMf[IDX_LEFT]
    oMright = robot.data.oMf[IDX_RIGHT]

    # Get cube hook positions (desired positions)
    from config import LEFT_HOOK, RIGHT_HOOK
    from tools import getcubeplacement

    oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)

    # Compute position errors (SE3 log)
    error_left = pin.log(oMleft.inverse() * oMcubeL).vector
    error_right = pin.log(oMright.inverse() * oMcubeR).vector

    # Desired forces (proportional to error)
    Kf = 50.0  # force gain
    F_left = Kf * error_left
    F_right = Kf * error_right

    # Compute Jacobians
    pin.computeJointJacobians(robot.model, robot.data, q)
    J_left = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_LEFT)
    J_right = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_RIGHT)

    # Force control torques
    tau_grasp = J_left.T @ F_left + J_right.T @ F_right

    # Combined control
    torques = tau_baseline + tau_grasp

    # Apply
    sim.step(torques)
```

**Pros:**
- Best for grasping tasks
- Naturally handles positioning errors
- Maintains contact forces

**Cons:**
- Most complex
- May need tuning
- Can be unstable if gains too high

---

### Recommended Control Strategy

**Start with Option 1 (PD + Gravity):**
1. Simplest to implement and debug
2. Good enough for initial testing
3. Validates your trajectory

**Upgrade to Option 2 (Inverse Dynamics):**
1. Better tracking performance
2. Still relatively simple
3. Likely sufficient for good marks

**Add Option 3 (Force Control) if needed:**
1. Only if cube keeps slipping
2. Only if you have time
3. May not be necessary

---

### Implementation Checklist for Task II

#### Phase 1: Basic Control (PD)
- [ ] Implement `controllaw()` with PD gains
- [ ] Add gravity compensation using `pin.nle()`
- [ ] Test with Level 0 trajectory
- [ ] Verify robot doesn't fall/drift
- [ ] Check tracking errors (plot q vs q_des)

#### Phase 2: Improved Control (Inverse Dynamics)
- [ ] Compute mass matrix using `pin.crba()`
- [ ] Implement inverse dynamics control law
- [ ] Test and compare with PD control
- [ ] Measure tracking performance

#### Phase 3: Grasping (Optional)
- [ ] Add force control term
- [ ] Compute Jacobians and forces
- [ ] Tune force gains
- [ ] Verify cube stays grasped

#### Phase 4: Integration
- [ ] Connect trajectory from Task I
- [ ] Run full simulation q0 -> qgoal
- [ ] Verify cube reaches target
- [ ] Record video/screenshots for report

---

<a name="tutorials"></a>
## 5. Relevant Tutorials and Resources

### For Part 2 Task I (Trajectory Optimization)

**Primary Resources:**
1. **Tutorial 6: Convex Trajectory Optimization** (`/home/infernox/aro/tutorials/6_convex_optimisation.ipynb`)
   - ODE integration basics
   - Linear and Quadratic Programming
   - Minimum acceleration trajectories
   - **Key for Level 1 implementation**

2. **Tutorial 7: Advanced QP & Motion Planning** (`/home/infernox/aro/tutorials/7_more_qp.ipynb`)
   - Polynomial trajectory optimization
   - Hermite splines and Bezier curves
   - Joint limit constraints
   - **Key for understanding control points**

**Secondary Resources:**
3. **Bezier Curves Primer:** https://pomax.github.io/bezierinfo/
   - Visual understanding of Bezier curves
   - Control point manipulation
   - Derivative properties

4. **bezier.py** in your lab folder
   - Ready-to-use Bezier class
   - Derivative computation
   - Example usage in `control.py:47`

**Python Libraries:**
- `cvxpy`: Convex optimization (for QP) - `pip install cvxpy`
- `scipy.optimize`: Non-linear optimization (for Level 2)

---

### For Part 2 Task II (Control)

**Primary Resources:**
1. **Tutorial 5: Control Design** (`/home/infernox/aro/tutorials/5_control.ipynb`)
   - PID controller fundamentals
   - Gain tuning strategies
   - Stability analysis
   - **Key for understanding control basics**

2. **Pinocchio Dynamics API:**
   - `pin.nle(model, data, q, vq)`: Non-linear effects (Coriolis + gravity)
   - `pin.crba(model, data, q)`: Composite Rigid Body Algorithm (mass matrix)
   - `pin.computeAllTerms(model, data, q, vq)`: Update all dynamic quantities
   - `pin.rnea(model, data, q, vq, aq)`: Recursive Newton-Euler (inverse dynamics)

**Reference Materials:**
3. **Lab Instructions** (section "II. Implementing a torque control law")
   - PyBullet integration
   - Step-by-step simulation
   - `Simulation` class methods

4. **setup_pybullet.py**
   - Understand PyBullet interface
   - `getpybulletstate()`: Read robot state
   - `step(torques)`: Apply control
   - `setqsim(q)`: Set initial configuration

---

<a name="pseudocode"></a>
## 6. Pseudocode and Algorithms

### Task I: Trajectory Generation

#### Level 0 Pseudocode

```
FUNCTION create_trajectory_level0(path, total_time):
    INPUT:
        path = [q0, q1, ..., qn]  # list of configurations
        total_time = T  # total duration (seconds)

    OUTPUT:
        q_traj(t), v_traj(t), a_traj(t)  # trajectory functions

    STEPS:
    1. n_segments = len(path) - 1
    2. segment_duration = total_time / n_segments

    3. FOR each segment i from 0 to n_segments-1:
        a. q_start = path[i]
        b. q_end = path[i+1]
        c. t_start = i * segment_duration
        d. t_end = (i+1) * segment_duration

        # Create cubic Bezier with zero endpoint velocities
        e. control_points = [q_start, q_start, q_end, q_end]
        f. segments[i] = Bezier(control_points, t_start, t_end)

    4. DEFINE q_traj(t):
        a. Find segment index: i = min(floor(t / segment_duration), n_segments-1)
        b. RETURN segments[i](t)

    5. DEFINE v_traj(t):
        a. Find segment index i
        b. RETURN segments[i].derivative(1)(t)

    6. DEFINE a_traj(t):
        a. Find segment index i
        b. RETURN segments[i].derivative(2)(t)

    7. RETURN q_traj, v_traj, a_traj
END FUNCTION
```

---

#### Level 1 Pseudocode (QP Optimization)

```
FUNCTION create_trajectory_level1(path, total_time, N):
    INPUT:
        path = [q0, q1, ..., qM]  # M waypoints
        total_time = T
        N = number of Bezier control points (N > M)

    OUTPUT:
        q_traj(t), v_traj(t), a_traj(t)

    STEPS:

    1. SETUP OPTIMIZATION PROBLEM:
        a. Decision variables: P = [p0, p1, ..., p_{N-1}]  # each pᵢ ∈ R^nq

        b. Cost function: minimize Σᵢ ||pᵢ₊₂ - 2*pᵢ₊₁ + pᵢ||²
           (This approximates ∫ ||aq(t)||² dt)

        c. Constraints:
           i.   p0 = q0 (start configuration)
           ii.  p_{N-1} = qM (end configuration)
           iii. p0 = p1 (zero initial velocity)
           iv.  p_{N-2} = p_{N-1} (zero final velocity)

           v.   FOR each waypoint qⱼ in path:
                    t_j = j / (M-1)  # normalized time
                    idx = round(t_j * (N-1))
                    p_{idx} ≈ qⱼ  (either hard or soft constraint)

           vi.  (OPTIONAL) Velocity limits:
                    ||pᵢ₊₁ - pᵢ|| ≤ v_max * dt

           vii. (OPTIONAL) Acceleration limits:
                    ||pᵢ₊₂ - 2*pᵢ₊₁ + pᵢ|| ≤ a_max * dt²

    2. SOLVE QP:
        a. Use CVXPY or scipy
        b. Solver: OSQP or ECOS
        c. Check solution status

    3. CREATE BEZIER CURVE:
        a. control_points = solution P
        b. bezier = Bezier(control_points, t_min=0, t_max=T)
        c. v_bezier = bezier.derivative(1)
        d. a_bezier = bezier.derivative(2)

    4. RETURN bezier, v_bezier, a_bezier
END FUNCTION
```

**Alternative: Soft Constraints for Path Following**

```
Modify Step 1.c.v to:

    v. Path-following SOFT constraint (add to cost):
        cost += weight_path * Σⱼ ||p_{idx(j)} - path[j]||²

       where weight_path >> 1 (e.g., 1000)

    Remove hard constraints on intermediate waypoints
```

---

### Task II: Control Law

#### PD Control with Gravity Compensation

```
FUNCTION controllaw_PD(sim, robot, trajs, t_current):
    INPUT:
        sim: PyBullet simulation
        robot: Pinocchio robot model
        trajs: (q_traj, v_traj, a_traj)
        t_current: current time

    OUTPUT:
        None (applies torques to sim)

    STEPS:

    1. GET CURRENT STATE:
        q, vq = sim.getpybulletstate()

    2. GET DESIRED STATE:
        q_des = q_traj(t_current)
        vq_des = v_traj(t_current)

    3. COMPUTE ERRORS:
        e_pos = q_des - q
        e_vel = vq_des - vq

    4. COMPUTE GRAVITY/CORIOLIS:
        pin.computeAllTerms(robot.model, robot.data, q, vq)
        g = pin.nle(robot.model, robot.data, q, vq)

    5. CONTROL LAW:
        tau = Kp * e_pos + Kd * e_vel + g

        where:
            Kp = 300 (or tune per joint)
            Kd = 2 * sqrt(Kp)  (critical damping)

    6. APPLY TORQUES:
        sim.step(tau)
END FUNCTION
```

---

#### Inverse Dynamics Control

```
FUNCTION controllaw_InverseDynamics(sim, robot, trajs, t_current):
    INPUT: same as above
    OUTPUT: same as above

    STEPS:

    1. GET STATES:
        q, vq = sim.getpybulletstate()
        q_des = q_traj(t_current)
        vq_des = v_traj(t_current)
        aq_des = a_traj(t_current)  # use desired acceleration!

    2. COMPUTE ERRORS:
        e_pos = q_des - q
        e_vel = vq_des - vq

    3. COMMANDED ACCELERATION (with feedback):
        aq_cmd = aq_des + Kp * e_pos + Kd * e_vel

    4. COMPUTE DYNAMICS TERMS:
        pin.computeAllTerms(robot.model, robot.data, q, vq)
        M = pin.crba(robot.model, robot.data, q)  # mass matrix
        h = pin.nle(robot.model, robot.data, q, vq)  # Coriolis + gravity

    5. INVERSE DYNAMICS:
        tau = M * aq_cmd + h

    6. APPLY:
        sim.step(tau)
END FUNCTION
```

---

#### Inverse Dynamics + Force Control

```
FUNCTION controllaw_WithGrasping(sim, robot, cube, trajs, t_current):
    INPUT:
        sim, robot, trajs, t_current (as before)
        cube: cube object for grasping

    OUTPUT: applies torques

    STEPS:

    1-5. SAME AS INVERSE DYNAMICS (compute tau_baseline)

    6. COMPUTE GRASPING FORCES:
        a. Get current end-effector poses:
            pin.framesForwardKinematics(robot.model, robot.data, q)
            oM_left = robot.data.oMf[IDX_LEFT]
            oM_right = robot.data.oMf[IDX_RIGHT]

        b. Get desired hook poses on cube:
            oM_cube_left = getcubeplacement(cube, LEFT_HOOK)
            oM_cube_right = getcubeplacement(cube, RIGHT_HOOK)

        c. Compute SE3 errors:
            error_left = log(oM_left^{-1} * oM_cube_left)
            error_right = log(oM_right^{-1} * oM_cube_right)

        d. Desired forces (proportional control):
            F_left = Kf * error_left.vector
            F_right = Kf * error_right.vector

            where Kf = 50 (tune as needed)

    7. COMPUTE FORCE TORQUES:
        a. Get Jacobians:
            J_left = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_LEFT)
            J_right = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_RIGHT)

        b. Map forces to joint torques:
            tau_grasp = J_left^T * F_left + J_right^T * F_right

    8. COMBINED CONTROL:
        tau = tau_baseline + tau_grasp

    9. APPLY:
        sim.step(tau)
END FUNCTION
```

---

### Complete Integration

```
MAIN PROGRAM:

1. SETUP:
    robot, sim, cube = setupwithpybullet()

2. COMPUTE PATH (Part 1):
    q0, success0 = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT)
    qe, successe = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET)

    IF NOT (success0 AND successe):
        ERROR "Invalid initial or end configuration"
        EXIT

    path, path_success = computepath(q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    IF NOT path_success:
        WARNING "Path may be incomplete"

3. CREATE TRAJECTORY (Part 2 Task I):
    # Choose your level:

    # Option A: Level 0
    trajs = create_trajectory_level0(path, total_time=10.0)

    # Option B: Level 1
    trajs = create_trajectory_level1(path, total_time=10.0, N=50)

    T = total_time

4. SET INITIAL CONFIGURATION:
    sim.setqsim(q0)

5. CONTROL LOOP (Part 2 Task II):
    t = 0.0
    WHILE t < T:
        controllaw(sim, robot, trajs, t, cube)
        t = t + DT  # DT from config.py (typically 1e-3)

6. DONE
    PRINT "Task complete!"
    # Robot should have moved cube to target
```

---

<a name="collaboration"></a>
## 7. Team Collaboration Strategy

### How to Work with Your Teammate

#### Division of Labor

**Option 1: Sequential (Recommended for Learning)**
- Person A: Implement Level 0 trajectory
- Person B: Implement basic control law (PD)
- Test together
- Person A: Upgrade to Level 1 trajectory
- Person B: Upgrade to inverse dynamics control
- Test and refine together

**Option 2: Parallel (Faster)**
- Person A: Focus entirely on Task I (trajectory)
  - Implement Level 0
  - Implement Level 1
  - Document and test
- Person B: Focus entirely on Task II (control)
  - Implement PD control
  - Implement inverse dynamics
  - Add force control if needed
- Integration: Meet to combine and test

**Option 3: Feature-based**
- Person A: Core implementation (Level 0 + PD control)
- Person B: Advanced features (Level 1 QP + force control)
- Both: Testing, debugging, report

---

#### Communication Strategy

**Daily Stand-ups (5 minutes):**
- What did you do yesterday?
- What will you do today?
- Any blockers?

**Weekly Deep Dives (1-2 hours):**
- Review each other's code
- Pair programming on difficult sections
- Integration testing
- Plan next steps

**Use Version Control Properly:**
```bash
# Person A working on trajectory
git checkout -b feature/trajectory-optimization
# Make changes, commit
git commit -m "Implement Level 1 QP trajectory optimization"
git push origin feature/trajectory-optimization

# Person B working on control
git checkout -b feature/inverse-dynamics-control
# Make changes, commit
git commit -m "Add inverse dynamics control law"
git push origin feature/inverse-dynamics-control

# Merge when ready
git checkout main
git merge feature/trajectory-optimization
git merge feature/inverse-dynamics-control
```

---

#### Code Review Checklist

When reviewing teammate's code, check:

**Functionality:**
- [ ] Does it run without errors?
- [ ] Does it produce expected results?
- [ ] Are edge cases handled?

**Quality:**
- [ ] Is code readable and well-commented?
- [ ] Are variable names meaningful?
- [ ] Is there duplicated code that could be refactored?

**Integration:**
- [ ] Does it follow the required function signatures?
- [ ] Is it compatible with existing code?
- [ ] Are dependencies documented?

**Testing:**
- [ ] Are there test cases?
- [ ] Does it handle invalid inputs gracefully?
- [ ] Are there any obvious bugs?

---

#### Handling Disagreements

**Technical Disagreements:**
1. Both present your approach with pros/cons
2. Implement both quickly (1-2 hours each)
3. Test and compare objectively
4. Choose best approach or hybrid
5. Document decision in comments/report

**Example:**
```
# Disagreement: Should we use soft or hard constraints for path following?

# Approach A (Person 1): Hard constraints
# Pros: Guarantees path following
# Cons: May be infeasible

# Approach B (Person 2): Soft constraints
# Pros: Always feasible
# Cons: May deviate from path

# Decision: Test both, use soft with high weight
# Rationale: More robust, still tracks path well
```

---

### Discussion Topics with Teammate

#### Initial Planning Meeting

**Agenda:**
1. Review Part 1 implementation together
   - Does it work reliably?
   - Any bugs to fix?
   - Can we improve it?

2. Understand Part 2 requirements
   - Read lab instructions together
   - Clarify any confusing points
   - Discuss which level to target (0, 1, or 2)

3. Decide on division of labor
   - Who works on what?
   - How will we integrate?
   - What's our timeline?

4. Set up development environment
   - Do both have same library versions?
   - Can both run PyBullet?
   - Test basic control loop

5. Agree on coding standards
   - Naming conventions
   - Comment style
   - Git workflow

---

#### Weekly Progress Review

**Week 1 Discussion Points:**
- [ ] Level 0 trajectory working?
- [ ] Basic control law working?
- [ ] Robot can track straight-line trajectory?
- [ ] Identified any issues?
- [ ] Ready to move to Level 1?

**Week 2 Discussion Points:**
- [ ] QP optimization implemented?
- [ ] Trajectories look smooth?
- [ ] Control tracking well?
- [ ] Cube staying grasped?
- [ ] Need force control?

**Week 3 Discussion Points:**
- [ ] Everything working end-to-end?
- [ ] Quantitative results (tracking errors)?
- [ ] What to include in report?
- [ ] Any bonus features to add?

---

<a name="qa"></a>
## 8. Common Questions and Answers

### General Questions

**Q1: Do I have to follow this plan exactly?**

A: No! The lab instructions say:
> "I don't like the approach you have proposed to solve the problem. Can I do my own thing? Yes... and no."

You're free to use different methods as long as:
1. You discuss with TAs/instructor first
2. You implement required functions (`computepath`, etc.)
3. You justify your approach in the report
4. It's not hard-coded

**Q2: Can I use libraries not mentioned?**

A: From instructions:
> "If you want to use non-native python libraries, we must discuss this."

Ask instructor first! But likely okay: numpy, scipy, cvxpy, matplotlib.

**Q3: How will I be graded?**

A: From instructions:
> "I will run code that will use these functions to evaluate quantitatively the methods you proposed."

So: Make sure required function signatures are correct! The instructor will call your functions with test cases.

---

### Task I: Trajectory Questions

**Q4: How many control points (N) should I use for Bezier curve?**

A: Rule of thumb:
- N = 2 * M (where M = number of path waypoints) is a good start
- N = 50-100 for most paths
- Too few (N < 20): jerky motion
- Too many (N > 200): slow optimization, overfitting

Test different values and see what works!

**Q5: What if my QP optimization fails or is infeasible?**

A: Common causes and fixes:

1. **Conflicting constraints:** Path waypoints too close together or inconsistent
   - Solution: Use soft constraints for intermediate waypoints

2. **Numerical issues:** Constraints too tight
   - Solution: Relax velocity/acceleration bounds

3. **Wrong time scaling:** Time too short for path
   - Solution: Increase total_time

4. **Solver issues:** CVXPY sometimes fails
   - Solution: Try different solver (OSQP, ECOS, SCS)

**Debug checklist:**
```python
# Check if problem is feasible
prob.solve()
print(f"Status: {prob.status}")
if prob.status == cp.INFEASIBLE:
    # Try relaxing constraints
    # Check constraint compatibility
```

**Q6: My trajectory doesn't follow the path closely. What's wrong?**

A: Likely causes:
1. Using soft constraints with low weight
   - Fix: Increase `weight_path` (try 100, 1000, 10000)

2. Too few control points
   - Fix: Increase N

3. Path has sharp turns
   - Fix: QP inherently smooths trajectories. This is okay! As long as it's collision-free.

**Q7: Should I optimize trajectory in configuration space or task space?**

A: Recommended: **Configuration space**
- Easier to handle full robot motion
- Direct control of all joints
- Grasping constraints handled implicitly via IK

Task space is possible but more complex (you'd need IK at every time step).

**Q8: Do I need to handle the grasping constraint in trajectory optimization?**

A: From instructions:
> "I believe that if the tracked trajectory is good enough you should not need to handle the grasping constraints as accurately as this will be fixed by the control law."

So: **No, probably not necessary.** Let the force control handle it.

If you really want to (Level 2):
- Add constraints: q(t) must satisfy grasping IK
- This makes problem non-convex!

---

### Task II: Control Questions

**Q9: Which control law should I use?**

A: Recommended progression:
1. **Start with:** PD + gravity compensation
   - Simplest, works reasonably well
   - Good for testing trajectory

2. **Upgrade to:** Inverse dynamics control
   - Better performance, still not too complex
   - Likely sufficient for good marks (75-80%)

3. **Add if needed:** Force control for grasping
   - Only if cube keeps slipping
   - For exceptional marks (85%+)

**Q10: How do I tune the gains Kp and Kd?**

A: Start with instructor's suggestion:
```python
Kp = 300
Kd = 2 * np.sqrt(Kp)  # critical damping
```

If that doesn't work:

**Symptoms of low Kp:**
- Robot too slow to track
- Large position errors
- Sluggish response

**Symptoms of high Kp:**
- Oscillations
- Instability
- Robot vibrates

**Tuning process:**
1. Start with Kp = 100
2. Gradually increase until you see oscillations
3. Back off to 70-80% of that value
4. Set Kd = 2*sqrt(Kp)
5. Fine-tune Kd if needed (decrease if too slow, increase if oscillating)

**Advanced: Per-joint tuning**
```python
# Some joints may need different gains
Kp = np.array([300, 300, 200, 200, 150, 150, ...])  # one per joint
Kd = 2 * np.sqrt(Kp)
```

**Q11: My robot is falling/drifting even with gravity compensation. Why?**

A: Check:
1. Are you using `pin.nle()` correctly?
   ```python
   pin.computeAllTerms(robot.model, robot.data, q, vq)
   g = pin.nle(robot.model, robot.data, q, vq)
   ```

2. Are torques being applied to the correct joints?
   - Check `sim.bulletCtrlJointsInPinOrder`
   - Ensure torque array has correct length

3. Is PyBullet in torque control mode?
   ```python
   sim.setTorqueControlMode()  # should be called in setup
   ```

4. Are you using the correct configuration?
   - Pinocchio and PyBullet may have different joint orders
   - Check `sim.getpybulletstate()` returns correct shape

**Q12: The cube keeps falling or slipping. How do I fix this?**

A: Options:

1. **Check trajectory:** Is robot actually grasping cube?
   - Visualize in MeshCat
   - Check hand-hook distances

2. **Add force control:** Attract hands to cube hooks
   - See Option 3 control law in this document

3. **Increase force gains:** If using force control
   - Try Kf = 10, 50, 100, 200

4. **Check PyBullet friction:** May need to adjust
   ```python
   # In setup_pybullet.py or main
   for joint_id in [left_hand_id, right_hand_id]:
       p.changeDynamics(joint_id, -1,
                        lateralFriction=1.0,
                        spinningFriction=0.1)
   ```

5. **Trajectory too fast:** Slow down
   - Increase total_time

**Q13: My robot becomes unstable and explodes. Help!**

A: Classic robotics problem! Causes:

1. **Gains too high:**
   - Reduce Kp and Kd by factor of 2-10

2. **Trajectory discontinuity:**
   - Check trajectory has zero initial velocity
   - Check no sudden jumps in q(t)

3. **Numerical issues in dynamics:**
   - Check q is not violating joint limits
   - Check for NaN/Inf in torques:
     ```python
     if np.any(np.isnan(torques)) or np.any(np.isinf(torques)):
         print("WARNING: Invalid torques!")
         torques = np.zeros_like(torques)  # safe fallback
     ```

4. **Initial configuration mismatch:**
   - Ensure `sim.setqsim(q0)` matches trajectory start
   - Check: `np.allclose(q0, q_traj(0.0))`

5. **Timestep too large:**
   - DT = 1e-3 should be fine
   - If still unstable, try DT = 5e-4

---

### Integration Questions

**Q14: How do I connect my trajectory to my control law?**

A: In `control.py` main section:

```python
# 1. Compute path (Part 1)
path, success = computepath(q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

# 2. Create trajectory (Part 2 Task I)
total_time = 10.0
trajs = create_trajectory_level1(path, total_time, N=50)
# trajs = (q_traj, v_traj, a_traj)

# 3. Set initial config
sim.setqsim(q0)

# 4. Run control loop (Part 2 Task II)
t = 0.0
while t < total_time:
    controllaw(sim, robot, trajs, t, cube)
    t += DT
```

**Q15: How do I visualize/debug my trajectory?**

A: Several options:

1. **Plot trajectory:**
```python
import matplotlib.pyplot as plt

times = np.linspace(0, total_time, 100)
qs = np.array([q_traj(t) for t in times])

plt.figure(figsize=(10, 6))
for joint_idx in range(qs.shape[1]):
    plt.plot(times, qs[:, joint_idx], label=f'Joint {joint_idx}')
plt.xlabel('Time (s)')
plt.ylabel('Configuration (rad)')
plt.legend()
plt.title('Trajectory in Configuration Space')
plt.show()
```

2. **Visualize in MeshCat:**
```python
robot, cube, viz = setupwithmeshcat()

for t in np.linspace(0, total_time, 100):
    q = q_traj(t)
    viz.display(q)
    time.sleep(0.05)
```

3. **Check velocities/accelerations:**
```python
times = np.linspace(0, total_time, 100)
vqs = np.array([v_traj(t) for t in times])
aqs = np.array([a_traj(t) for t in times])

print(f"Max velocity: {np.max(np.abs(vqs))}")
print(f"Max acceleration: {np.max(np.abs(aqs))}")
```

**Q16: How do I know if my implementation is correct?**

A: Validation checklist:

**Trajectory (Task I):**
- [ ] Trajectory passes through q0 at t=0
- [ ] Trajectory passes through qgoal at t=T
- [ ] Velocity is zero at t=0 and t=T
- [ ] Trajectory roughly follows path waypoints
- [ ] No joint limit violations
- [ ] No NaN/Inf values
- [ ] Smooth (no sudden jumps)

**Control (Task II):**
- [ ] Robot doesn't fall under gravity
- [ ] Robot tracks simple trajectory (straight line)
- [ ] Tracking errors decrease over time
- [ ] No instabilities/explosions
- [ ] Cube moves with robot
- [ ] Cube reaches target region

**Integration:**
- [ ] Full pipeline runs without errors
- [ ] Cube is grasped at start
- [ ] Cube reaches target at end
- [ ] Motion looks reasonable (not jerky/unnatural)

---

## 9. Timeline and Milestones

### Recommended 3-Week Schedule

#### Week 1: Foundation
**Days 1-2: Understand and Plan**
- Review Part 1 implementation
- Read Part 2 instructions thoroughly
- Study Tutorial 5 (Control)
- Study Tutorial 6 (Trajectory Optimization)
- Discuss plan with teammate

**Days 3-4: Level 0 + Basic Control**
- Implement Level 0 trajectory (manual parametrization)
- Implement PD control with gravity compensation
- Test with simple trajectories

**Days 5-7: Integration and Testing**
- Connect trajectory to control
- Test full pipeline
- Fix bugs
- Measure performance

**Milestone 1:** Robot can track Level 0 trajectory and move cube

---

#### Week 2: Optimization
**Days 1-3: Level 1 Trajectory**
- Study QP formulation
- Implement QP-based optimization
- Test and debug
- Compare with Level 0

**Days 4-5: Improved Control**
- Implement inverse dynamics control
- Test and compare with PD
- Tune gains if needed

**Days 6-7: Integration**
- Connect Level 1 trajectory to control
- Full system testing
- Performance evaluation

**Milestone 2:** Robot tracks optimized trajectory smoothly

---

#### Week 3: Refinement and Documentation
**Days 1-2: Optional Enhancements**
- Add force control if needed
- Attempt Level 2 if time permits
- Improve any weak points

**Days 3-4: Testing and Validation**
- Extensive testing with different scenarios
- Record videos/screenshots
- Measure quantitative metrics

**Days 5-7: Report Writing**
- Document approach
- Present results
- Discuss limitations
- Finalize submission

**Milestone 3:** Complete working system + comprehensive report

---

## 10. Success Metrics

### How to Measure Performance

#### Trajectory Quality
```python
def evaluate_trajectory(path, q_traj, total_time):
    """
    Evaluate trajectory quality
    """
    # 1. Path following error
    M = len(path)
    errors = []
    for i, q_waypoint in enumerate(path):
        t = i / (M-1) * total_time
        q_t = q_traj(t)
        error = np.linalg.norm(q_t - q_waypoint)
        errors.append(error)

    max_path_error = max(errors)
    avg_path_error = np.mean(errors)

    # 2. Smoothness (acceleration magnitude)
    times = np.linspace(0, total_time, 100)
    accels = [np.linalg.norm(a_traj(t)) for t in times]
    avg_accel = np.mean(accels)
    max_accel = max(accels)

    # 3. Boundary conditions
    v0 = np.linalg.norm(v_traj(0.0))
    vf = np.linalg.norm(v_traj(total_time))

    print(f"Trajectory Evaluation:")
    print(f"  Max path error: {max_path_error:.4f}")
    print(f"  Avg path error: {avg_path_error:.4f}")
    print(f"  Avg acceleration: {avg_accel:.4f}")
    print(f"  Max acceleration: {max_accel:.4f}")
    print(f"  Initial velocity: {v0:.6f} (should be ~0)")
    print(f"  Final velocity: {vf:.6f} (should be ~0)")

    return {
        'max_path_error': max_path_error,
        'avg_path_error': avg_path_error,
        'avg_accel': avg_accel,
        'max_accel': max_accel,
        'boundary_conditions_ok': (v0 < 1e-3 and vf < 1e-3)
    }
```

#### Control Performance
```python
def evaluate_control(sim, robot, trajs, total_time):
    """
    Evaluate control tracking performance
    """
    q_traj, v_traj, a_traj = trajs

    # Run simulation and record
    sim.setqsim(q_traj(0.0))

    tracking_errors = []
    velocity_errors = []

    t = 0.0
    while t < total_time:
        q_des = q_traj(t)
        vq_des = v_traj(t)

        controllaw(sim, robot, trajs, t, cube)

        q, vq = sim.getpybulletstate()

        tracking_errors.append(np.linalg.norm(q - q_des))
        velocity_errors.append(np.linalg.norm(vq - vq_des))

        t += DT

    print(f"Control Evaluation:")
    print(f"  Max tracking error: {max(tracking_errors):.4f}")
    print(f"  Avg tracking error: {np.mean(tracking_errors):.4f}")
    print(f"  Max velocity error: {max(velocity_errors):.4f}")
    print(f"  Avg velocity error: {np.mean(velocity_errors):.4f}")

    return {
        'max_tracking_error': max(tracking_errors),
        'avg_tracking_error': np.mean(tracking_errors),
        'tracking_errors': tracking_errors  # for plotting
    }
```

#### Task Success
```python
def evaluate_task_success(cube, target_placement, threshold=0.05):
    """
    Check if cube reached target
    """
    final_placement = getcubeplacement(cube)

    # Position error
    pos_error = np.linalg.norm(
        final_placement.translation - target_placement.translation
    )

    # Orientation error
    rot_error = np.linalg.norm(
        pin.log(final_placement.rotation.T @ target_placement.rotation).vector
    )

    success = pos_error < threshold

    print(f"Task Evaluation:")
    print(f"  Position error: {pos_error:.4f}m (threshold: {threshold})")
    print(f"  Rotation error: {rot_error:.4f}rad")
    print(f"  Success: {success}")

    return success
```

---

## Final Recommendations

### Priority List (What to Do First)

1. **Must Have (Essential for passing):**
   - [ ] Implement Level 0 trajectory (1-2 days)
   - [ ] Implement PD control with gravity compensation (1-2 days)
   - [ ] Test and debug basic system (2-3 days)
   - [ ] Write report documenting approach (3-4 days)

2. **Should Have (For good marks 70-80%):**
   - [ ] Implement Level 1 QP trajectory (3-4 days)
   - [ ] Implement inverse dynamics control (1-2 days)
   - [ ] Quantitative evaluation (1 day)

3. **Nice to Have (For excellent marks 80%+):**
   - [ ] Add force control for grasping (1-2 days)
   - [ ] Time optimization in QP (1 day)
   - [ ] Extensive testing and analysis (2 days)

4. **Exceptional (For 85%+):**
   - [ ] Level 2 collision-aware optimization (3-5 days)
   - [ ] Advanced control features (2-3 days)
   - [ ] Comprehensive report with deep analysis (4-5 days)

---

### Key Insights for Success

1. **Start Simple:** Level 0 + PD control gets you far. Don't jump to advanced methods immediately.

2. **Test Incrementally:** Test each component separately before integrating.

3. **Use Tutorials:** Tutorials 5, 6, 7 contain most of what you need.

4. **Ask for Help:** Use Piazza, office hours, teammate discussions.

5. **Document as You Go:** Don't leave report for the end.

6. **Version Control:** Commit often, branch for features.

7. **Focus on Correctness:** A simple working solution beats a complex broken one.

8. **Time Management:** Leave buffer for unexpected issues.

---

## Conclusion

This document provides everything you need to successfully implement Part 2 of the lab. The key is to:

1. **Understand** the problem deeply
2. **Start simple** with Level 0 and PD control
3. **Test thoroughly** at each stage
4. **Upgrade systematically** to Level 1 and inverse dynamics
5. **Document well** for the report

Good luck! Remember: the goal is not just to complete the lab, but to **understand robotics deeply**. Take time to experiment, analyze, and learn from both successes and failures.

---

## Appendix: Quick Reference

### Important Functions

**Pinocchio Dynamics:**
```python
pin.computeAllTerms(model, data, q, vq)  # Update all terms
pin.nle(model, data, q, vq)              # Coriolis + gravity
pin.crba(model, data, q)                  # Mass matrix
pin.rnea(model, data, q, vq, aq)         # Inverse dynamics
```

**Pinocchio Kinematics:**
```python
pin.framesForwardKinematics(model, data, q)
pin.computeFrameJacobian(model, data, q, frame_id)
pin.log(M)  # SE3 -> se3 (6D vector)
```

**PyBullet Interface:**
```python
sim.getpybulletstate()  # Returns (q, vq)
sim.step(torques)        # Apply joint torques
sim.setqsim(q)          # Set configuration
```

**Bezier Trajectory:**
```python
from bezier import Bezier

curve = Bezier(control_points, t_min=0.0, t_max=T)
v_curve = curve.derivative(1)
a_curve = curve.derivative(2)

q = curve(t)   # Evaluate at time t
vq = v_curve(t)
aq = a_curve(t)
```

**Optimization (CVXPY):**
```python
import cvxpy as cp

x = cp.Variable(shape)
cost = cp.sum_squares(x)  # or other cost
constraints = [x >= 0, ...]
prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve(solver=cp.OSQP)
solution = x.value
```

### Configuration Values

```python
# From config.py
DT = 1e-3              # Simulation timestep
EPSILON = 1e-3         # Convergence threshold
LEFT_HAND = 'LARM_EFF'
RIGHT_HAND = 'RARM_EFF'
LEFT_HOOK = 'LARM_HOOK'
RIGHT_HOOK = 'RARM_HOOK'
```

### Typical Parameter Values

```python
# Control gains
Kp = 300.0
Kd = 2 * np.sqrt(Kp)  # ≈ 34.6
Kf = 50.0  # Force control gain

# Trajectory optimization
N = 50  # Number of control points
total_time = 10.0  # seconds
v_max = 2.0  # rad/s
a_max = 10.0  # rad/s²
weight_path = 1000.0  # Path-following weight
```

---

**End of Implementation Plan**

**Next Steps:**
1. Read this document thoroughly
2. Discuss with your teammate
3. Start implementing Level 0
4. Test early and often
5. Good luck!