# Pseudocode and Implementation Algorithms - Part 2

This document contains detailed pseudocode for all the algorithms you need to implement for Part 2 of the lab. Use this as a reference when coding.

---

## Table of Contents

1. [Task I: Trajectory Generation](#trajectory)
   - [Level 0: Manual Parametrization](#level-0)
   - [Level 1: QP Optimization](#level-1)
   - [Level 2: Collision-Aware](#level-2)
2. [Task II: Control Laws](#control)
   - [PD Control](#pd-control)
   - [Inverse Dynamics](#inverse-dynamics)
   - [Force Control](#force-control)
3. [Complete Integration](#integration)
4. [Helper Functions](#helpers)

---

<a name="trajectory"></a>
# Task I: Trajectory Generation Algorithms

<a name="level-0"></a>
## Level 0: Manual Time Parametrization

### Algorithm 1: Simple Piecewise Bezier Trajectory

```
FUNCTION create_trajectory_level0_simple(path, total_time):
    """
    Create trajectory using cubic Bezier curves between waypoints
    Zero velocities at all waypoints for simplicity

    Complexity: O(n) where n = len(path)
    Time to implement: 1-2 hours
    """

    INPUT:
        path: list of configurations [q0, q1, ..., qM]
        total_time: T (seconds)

    OUTPUT:
        (q_traj, v_traj, a_traj): tuple of functions

    ALGORITHM:

    1. INITIALIZATION
        M ← len(path)
        n_segments ← M - 1
        dt ← total_time / n_segments
        segments ← empty list

    2. CREATE BEZIER SEGMENTS
        FOR i FROM 0 TO n_segments - 1:
            q_start ← path[i]
            q_end ← path[i + 1]
            t_start ← i * dt
            t_end ← (i + 1) * dt

            // Cubic Bezier: 4 control points
            // p0 = p1 = q_start gives zero initial velocity
            // p2 = p3 = q_end gives zero final velocity
            control_points ← [q_start, q_start, q_end, q_end]

            bezier ← Bezier(control_points,
                           t_min=t_start,
                           t_max=t_end)

            segments.append(bezier)

    3. DEFINE TRAJECTORY FUNCTIONS
        FUNCTION q_traj(t):
            IF t < 0:
                RETURN path[0]
            IF t >= total_time:
                RETURN path[-1]

            // Find which segment
            segment_idx ← min(floor(t / dt), n_segments - 1)
            RETURN segments[segment_idx](t)

        FUNCTION v_traj(t):
            IF t < 0 OR t >= total_time:
                RETURN zeros_like(path[0])

            segment_idx ← min(floor(t / dt), n_segments - 1)
            RETURN segments[segment_idx].derivative(1)(t)

        FUNCTION a_traj(t):
            IF t < 0 OR t >= total_time:
                RETURN zeros_like(path[0])

            segment_idx ← min(floor(t / dt), n_segments - 1)
            RETURN segments[segment_idx].derivative(2)(t)

    4. RETURN (q_traj, v_traj, a_traj)

END FUNCTION
```

**Python Implementation:**

```python
def create_trajectory_level0_simple(path, total_time):
    from bezier import Bezier
    import numpy as np

    M = len(path)
    n_segments = M - 1
    dt = total_time / n_segments

    segments = []
    for i in range(n_segments):
        q_start = path[i]
        q_end = path[i + 1]
        t_start = i * dt
        t_end = (i + 1) * dt

        # Zero velocity at endpoints
        control_points = [q_start, q_start, q_end, q_end]
        bezier = Bezier(control_points, t_min=t_start, t_max=t_end)
        segments.append(bezier)

    def q_traj(t):
        if t < 0:
            return path[0]
        if t >= total_time:
            return path[-1]
        idx = min(int(t / dt), n_segments - 1)
        return segments[idx](t)

    def v_traj(t):
        if t < 0 or t >= total_time:
            return np.zeros_like(path[0])
        idx = min(int(t / dt), n_segments - 1)
        return segments[idx].derivative(1)(t)

    def a_traj(t):
        if t < 0 or t >= total_time:
            return np.zeros_like(path[0])
        idx = min(int(t / dt), n_segments - 1)
        return segments[idx].derivative(2)(t)

    return q_traj, v_traj, a_traj
```

---

### Algorithm 2: Improved Trajectory with Velocity Continuity

```
FUNCTION create_trajectory_level0_smooth(path, total_time):
    """
    Smoother trajectory with matched velocities at waypoints
    Uses Hermite-to-Bezier conversion

    Complexity: O(n)
    Time to implement: 2-3 hours
    """

    INPUT:
        path: list [q0, q1, ..., qM]
        total_time: T

    OUTPUT:
        (q_traj, v_traj, a_traj)

    ALGORITHM:

    1. COMPUTE WAYPOINT VELOCITIES (finite differences)
        M ← len(path)
        dt ← total_time / (M - 1)

        velocities ← empty list
        velocities[0] ← zeros_like(path[0])  // zero at start

        FOR i FROM 1 TO M - 2:
            // Central difference
            v ← (path[i+1] - path[i-1]) / (2 * dt)
            velocities[i] ← v

        velocities[M-1] ← zeros_like(path[-1])  // zero at end

    2. CREATE CUBIC HERMITE SEGMENTS (converted to Bezier)
        segments ← empty list

        FOR i FROM 0 TO M - 2:
            q0 ← path[i]
            q1 ← path[i+1]
            v0 ← velocities[i]
            v1 ← velocities[i+1]
            t_start ← i * dt
            t_end ← (i+1) * dt

            // Hermite to Bezier control point conversion:
            // For Hermite segment with endpoints q0, q1 and tangents v0, v1
            // Bezier control points are:
            p0 ← q0
            p1 ← q0 + (dt / 3) * v0
            p2 ← q1 - (dt / 3) * v1
            p3 ← q1

            control_points ← [p0, p1, p2, p3]
            bezier ← Bezier(control_points, t_start, t_end)
            segments.append(bezier)

    3. DEFINE TRAJECTORY FUNCTIONS
        (same as Algorithm 1)

    4. RETURN (q_traj, v_traj, a_traj)

END FUNCTION
```

**Python Implementation:**

```python
def create_trajectory_level0_smooth(path, total_time):
    from bezier import Bezier
    import numpy as np

    M = len(path)
    n_segments = M - 1
    dt = total_time / n_segments

    # Compute velocities at waypoints
    velocities = [np.zeros_like(path[0])]  # zero at start
    for i in range(1, M - 1):
        v = (path[i + 1] - path[i - 1]) / (2 * dt)
        velocities.append(v)
    velocities.append(np.zeros_like(path[-1]))  # zero at end

    # Create Bezier segments
    segments = []
    for i in range(n_segments):
        q0 = path[i]
        q1 = path[i + 1]
        v0 = velocities[i]
        v1 = velocities[i + 1]
        t_start = i * dt
        t_end = (i + 1) * dt

        # Hermite to Bezier conversion
        p0 = q0
        p1 = q0 + (dt / 3.0) * v0
        p2 = q1 - (dt / 3.0) * v1
        p3 = q1

        bezier = Bezier([p0, p1, p2, p3], t_min=t_start, t_max=t_end)
        segments.append(bezier)

    # Define trajectory functions (same as before)
    def q_traj(t):
        if t < 0:
            return path[0]
        if t >= total_time:
            return path[-1]
        idx = min(int(t / dt), n_segments - 1)
        return segments[idx](t)

    def v_traj(t):
        if t < 0 or t >= total_time:
            return np.zeros_like(path[0])
        idx = min(int(t / dt), n_segments - 1)
        return segments[idx].derivative(1)(t)

    def a_traj(t):
        if t < 0 or t >= total_time:
            return np.zeros_like(path[0])
        idx = min(int(t / dt), n_segments - 1)
        return segments[idx].derivative(2)(t)

    return q_traj, v_traj, a_traj
```

---

<a name="level-1"></a>
## Level 1: QP-Based Trajectory Optimization

### Algorithm 3: QP with Soft Path Constraints

```
FUNCTION optimize_trajectory_qp_soft(path, total_time, N, weight_path):
    """
    Optimize Bezier control points using Quadratic Programming
    Soft constraints allow small deviations from path for smoothness

    Complexity: O(N^3) for QP solver
    Time to implement: 4-6 hours (including learning QP)
    """

    INPUT:
        path: list [q0, q1, ..., qM]
        total_time: T
        N: number of Bezier control points
        weight_path: weight for path-following cost (e.g., 1000)

    OUTPUT:
        control_points: array of shape (N, nq)
        cost: final optimization cost

    ALGORITHM:

    1. PROBLEM SETUP
        M ← len(path)
        nq ← len(path[0])  // configuration dimension

    2. DECISION VARIABLES
        P ← Variable(shape=(N, nq))
        // P[i] is the i-th control point

    3. COST FUNCTION (minimize acceleration + path deviation)

        // 3a. Acceleration cost (sum of squared second differences)
        cost_accel ← 0
        FOR i FROM 0 TO N - 3:
            accel ← P[i+2] - 2*P[i+1] + P[i]
            cost_accel += sum_squares(accel)

        // 3b. Path-following cost (soft constraint)
        cost_path ← 0
        FOR j FROM 0 TO M - 1:
            // Map waypoint to control point index
            t_normalized ← j / (M - 1)
            idx ← round(t_normalized * (N - 1))
            cost_path += sum_squares(P[idx] - path[j])

        // 3c. Total cost
        cost ← cost_accel + weight_path * cost_path

    4. CONSTRAINTS (hard)

        constraints ← []

        // 4a. Boundary: start at q0
        constraints.append(P[0] == path[0])

        // 4b. Boundary: end at qM
        constraints.append(P[N-1] == path[M-1])

        // 4c. Zero initial velocity
        constraints.append(P[0] == P[1])

        // 4d. Zero final velocity
        constraints.append(P[N-2] == P[N-1])

    5. SOLVE QP
        problem ← Problem(Minimize(cost), constraints)
        problem.solve(solver=OSQP, verbose=True)

        IF problem.status != OPTIMAL:
            PRINT "Warning: solver status =", problem.status

    6. RETURN P.value, problem.value

END FUNCTION
```

**Python Implementation:**

```python
def optimize_trajectory_qp_soft(path, total_time, N=50, weight_path=1000.0):
    import cvxpy as cp
    import numpy as np

    M = len(path)
    nq = len(path[0])

    # Decision variables
    P = cp.Variable((N, nq))

    # Cost: acceleration
    cost_accel = 0
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        cost_accel += cp.sum_squares(accel)

    # Cost: path following (soft)
    cost_path = 0
    for j in range(M):
        t_norm = j / (M - 1)
        idx = int(round(t_norm * (N - 1)))
        idx = min(idx, N - 1)  # boundary safety
        cost_path += cp.sum_squares(P[idx] - path[j])

    # Total cost
    cost = cost_accel + weight_path * cost_path

    # Constraints
    constraints = [
        P[0] == path[0],        # start
        P[N-1] == path[-1],     # end
        P[0] == P[1],           # zero initial velocity
        P[N-2] == P[N-1]        # zero final velocity
    ]

    # Solve
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, verbose=True)

    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        print(f"Warning: Solver status: {prob.status}")

    return P.value, prob.value
```

---

### Algorithm 4: QP with Velocity and Acceleration Bounds

```
FUNCTION optimize_trajectory_qp_bounded(path, total_time, N, v_max, a_max):
    """
    QP optimization with explicit velocity and acceleration limits

    Complexity: O(N^3)
    Time to implement: 5-7 hours
    """

    INPUT:
        path, total_time, N (as before)
        v_max: maximum velocity (rad/s)
        a_max: maximum acceleration (rad/s^2)

    OUTPUT:
        control_points, cost

    ALGORITHM:

    1-3. SETUP (same as Algorithm 3, but use hard path constraints)

    4. CONSTRAINTS (extended)

        constraints ← []
        dt ← total_time / (N - 1)

        // 4a. Boundary conditions
        constraints.append(P[0] == path[0])
        constraints.append(P[N-1] == path[-1])
        constraints.append(P[0] == P[1])
        constraints.append(P[N-2] == P[N-1])

        // 4b. Velocity bounds
        FOR i FROM 0 TO N - 2:
            vel ← (P[i+1] - P[i]) / dt
            constraints.append(norm(vel, 2) <= v_max)

        // 4c. Acceleration bounds
        FOR i FROM 0 TO N - 3:
            accel ← (P[i+2] - 2*P[i+1] + P[i]) / (dt^2)
            constraints.append(norm(accel, 2) <= a_max)

        // 4d. Pass through waypoints (hard constraint)
        FOR j FROM 0 TO M - 1:
            t_norm ← j / (M - 1)
            idx ← round(t_norm * (N - 1))
            constraints.append(P[idx] == path[j])

    5. SOLVE (same as Algorithm 3)

    6. RETURN

END FUNCTION
```

**Note:** The `norm(vel, 2) <= v_max` constraint is actually a second-order cone constraint, not a pure QP constraint. CVXPY can handle this using SOCP solvers like ECOS or SCS.

**Python Implementation:**

```python
def optimize_trajectory_qp_bounded(path, total_time, N=50,
                                   v_max=2.0, a_max=10.0):
    import cvxpy as cp
    import numpy as np

    M = len(path)
    nq = len(path[0])
    dt = total_time / (N - 1)

    P = cp.Variable((N, nq))

    # Cost: minimize acceleration
    cost = 0
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        cost += cp.sum_squares(accel)

    constraints = []

    # Boundary conditions
    constraints.append(P[0] == path[0])
    constraints.append(P[N-1] == path[-1])
    constraints.append(P[0] == P[1])
    constraints.append(P[N-2] == P[N-1])

    # Velocity limits
    for i in range(N - 1):
        vel = (P[i+1] - P[i]) / dt
        constraints.append(cp.norm(vel, 2) <= v_max)

    # Acceleration limits
    for i in range(N - 2):
        accel = (P[i+2] - 2*P[i+1] + P[i]) / (dt**2)
        constraints.append(cp.norm(accel, 2) <= a_max)

    # Pass through waypoints (can make soft if needed)
    for j in range(M):
        t_norm = j / (M - 1)
        idx = int(round(t_norm * (N - 1)))
        idx = min(idx, N - 1)
        constraints.append(P[idx] == path[j])

    # Solve with SOCP solver
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.ECOS, verbose=True)

    return P.value, prob.value
```

---

### Algorithm 5: Joint Time and Trajectory Optimization

```
FUNCTION optimize_trajectory_with_time(path, N, tau_min, tau_max):
    """
    Optimize both trajectory and total time
    Uses tau = T^2 trick to keep problem convex

    Complexity: O(N^3)
    Time to implement: 6-8 hours
    """

    INPUT:
        path: waypoints
        N: number of control points
        tau_min: minimum tau (T_min^2)
        tau_max: maximum tau (T_max^2)

    OUTPUT:
        control_points, T_optimal

    ALGORITHM:

    1. DECISION VARIABLES
        P ← Variable(shape=(N, nq))
        tau ← Variable()  // tau = T^2

    2. COST FUNCTION
        // Trade-off: smooth trajectory vs. fast motion
        cost_accel ← 0
        FOR i FROM 0 TO N - 3:
            accel ← P[i+2] - 2*P[i+1] + P[i]
            // Note: acceleration scales as 1/tau
            cost_accel += sum_squares(accel) / tau

        // Penalize long time
        alpha ← 0.1  // trade-off parameter (tune this)
        cost ← cost_accel + alpha * tau

    3. CONSTRAINTS
        constraints ← []

        // Boundary conditions
        constraints.append(P[0] == path[0])
        constraints.append(P[N-1] == path[-1])
        constraints.append(P[0] == P[1])
        constraints.append(P[N-2] == P[N-1])

        // Time bounds
        constraints.append(tau >= tau_min)
        constraints.append(tau <= tau_max)

        // Optional: acceleration bounds scaled by tau
        a_max ← 10.0
        FOR i FROM 0 TO N - 3:
            accel ← P[i+2] - 2*P[i+1] + P[i]
            // ||accel|| / tau <= a_max
            constraints.append(norm(accel, 2) <= a_max * tau)

    4. SOLVE
        problem ← Problem(Minimize(cost), constraints)
        problem.solve(solver=OSQP)

    5. EXTRACT RESULTS
        P_optimal ← P.value
        T_optimal ← sqrt(tau.value)

    6. RETURN P_optimal, T_optimal

END FUNCTION
```

**Python Implementation:**

```python
def optimize_trajectory_with_time(path, N=50,
                                  T_min=1.0, T_max=20.0,
                                  alpha=0.1, a_max=10.0):
    import cvxpy as cp
    import numpy as np

    M = len(path)
    nq = len(path[0])

    # Decision variables
    P = cp.Variable((N, nq))
    tau = cp.Variable()  # tau = T^2

    # Cost: acceleration / tau + penalty on tau
    cost_accel = 0
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        cost_accel += cp.sum_squares(accel) / tau

    cost = cost_accel + alpha * tau

    # Constraints
    constraints = []

    # Boundary
    constraints.append(P[0] == path[0])
    constraints.append(P[N-1] == path[-1])
    constraints.append(P[0] == P[1])
    constraints.append(P[N-2] == P[N-1])

    # Time bounds
    tau_min = T_min ** 2
    tau_max = T_max ** 2
    constraints.append(tau >= tau_min)
    constraints.append(tau <= tau_max)

    # Acceleration bounds
    for i in range(N - 2):
        accel = P[i+2] - 2*P[i+1] + P[i]
        constraints.append(cp.norm(accel, 2) <= a_max * tau)

    # Soft path following
    weight_path = 1000.0
    cost_path = 0
    for j in range(M):
        t_norm = j / (M - 1)
        idx = int(round(t_norm * (N - 1)))
        idx = min(idx, N - 1)
        cost_path += cp.sum_squares(P[idx] - path[j])

    cost = cost + weight_path * cost_path

    # Solve
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, verbose=True)

    T_optimal = np.sqrt(tau.value)
    return P.value, T_optimal
```

---

### Algorithm 6: Create Trajectory from Control Points

```
FUNCTION control_points_to_trajectory(control_points, total_time):
    """
    Convert optimized control points to trajectory functions

    Complexity: O(1) setup, O(N) evaluation
    Time to implement: 30 minutes
    """

    INPUT:
        control_points: array (N, nq)
        total_time: T

    OUTPUT:
        (q_traj, v_traj, a_traj)

    ALGORITHM:

    1. CREATE BEZIER CURVE
        // Convert array to list of arrays (Bezier class requirement)
        cp_list ← [control_points[i] for i in 0 to N-1]

        bezier ← Bezier(cp_list, t_min=0.0, t_max=total_time)

    2. CREATE DERIVATIVES
        v_bezier ← bezier.derivative(1)
        a_bezier ← bezier.derivative(2)

    3. RETURN (bezier, v_bezier, a_bezier)
        // These are callable: bezier(t) returns q(t)

END FUNCTION
```

**Python Implementation:**

```python
def control_points_to_trajectory(control_points, total_time):
    from bezier import Bezier

    # Convert to list
    cp_list = [control_points[i] for i in range(len(control_points))]

    # Create Bezier curve
    q_traj = Bezier(cp_list, t_min=0.0, t_max=total_time)
    v_traj = q_traj.derivative(1)
    a_traj = q_traj.derivative(2)

    return q_traj, v_traj, a_traj
```

---

### Complete Level 1 Pipeline

```
FUNCTION create_trajectory_level1(path, total_time, N):
    """
    Complete Level 1 implementation

    Complexity: O(N^3) for optimization
    Time to implement: 6-8 hours total
    """

    INPUT:
        path: list of waypoint configurations
        total_time: T (or None to optimize)
        N: number of Bezier control points

    OUTPUT:
        (q_traj, v_traj, a_traj), T

    ALGORITHM:

    1. OPTIMIZE CONTROL POINTS
        IF total_time is None:
            control_points, T ← optimize_trajectory_with_time(path, N)
        ELSE:
            control_points, cost ← optimize_trajectory_qp_soft(path, total_time, N)
            T ← total_time

    2. CREATE TRAJECTORY FUNCTIONS
        q_traj, v_traj, a_traj ← control_points_to_trajectory(control_points, T)

    3. VALIDATE
        PRINT "Trajectory validation:"
        PRINT "  Total time:", T
        PRINT "  Start config:", q_traj(0.0)
        PRINT "  End config:", q_traj(T)
        PRINT "  Start velocity norm:", norm(v_traj(0.0))
        PRINT "  End velocity norm:", norm(v_traj(T))

        ASSERT norm(q_traj(0.0) - path[0]) < 1e-3
        ASSERT norm(q_traj(T) - path[-1]) < 1e-3
        ASSERT norm(v_traj(0.0)) < 1e-3
        ASSERT norm(v_traj(T)) < 1e-3

    4. RETURN (q_traj, v_traj, a_traj), T

END FUNCTION
```

---

<a name="level-2"></a>
## Level 2: Collision-Aware Trajectory Optimization

**Warning:** Advanced topic, only attempt if Level 1 works perfectly.

```
FUNCTION optimize_trajectory_collision_aware(path, total_time, N):
    """
    Non-linear optimization with collision avoidance
    Use Level 1 solution as initialization

    Complexity: O(N^4) or worse (non-convex)
    Time to implement: 10-15 hours
    Risk: May not converge
    """

    INPUT:
        path, total_time, N

    OUTPUT:
        control_points, cost

    ALGORITHM:

    1. INITIALIZATION (use Level 1 solution)
        P_init, _ ← optimize_trajectory_qp_soft(path, total_time, N)
        x0 ← flatten(P_init)  // vectorize for scipy

    2. DEFINE COST FUNCTION
        FUNCTION cost(x):
            P ← reshape(x, (N, nq))

            // Smoothness cost
            cost_accel ← sum over i of ||P[i+2] - 2*P[i+1] + P[i]||^2

            // Collision penalty
            cost_collision ← 0
            FOR i FROM 0 TO N-1:
                q ← P[i]
                IF collision(robot, q):
                    cost_collision += 10000  // large penalty
                ELSE:
                    d ← distanceToObstacle(robot, q)
                    IF d < safety_margin:
                        cost_collision += 1000 / (d + 0.001)

            RETURN cost_accel + cost_collision

    3. DEFINE CONSTRAINTS
        constraints ← []

        // Boundary conditions
        FUNCTION eq_constraint_start(x):
            RETURN x[0:nq] - path[0]

        FUNCTION eq_constraint_end(x):
            RETURN x[-nq:] - path[-1]

        constraints.append({'type': 'eq', 'fun': eq_constraint_start})
        constraints.append({'type': 'eq', 'fun': eq_constraint_end})

        // Zero velocities (similar)

    4. SOLVE NON-LINEAR PROGRAM
        result ← scipy.optimize.minimize(
            cost,
            x0,
            method='SLSQP',
            constraints=constraints,
            options={'maxiter': 1000}
        )

    5. EXTRACT SOLUTION
        P_optimal ← reshape(result.x, (N, nq))

    6. VALIDATE SOLUTION
        success ← result.success
        IF NOT success:
            PRINT "Warning: NLP did not converge"

        // Check collisions
        collision_free ← True
        FOR i FROM 0 TO N-1:
            IF collision(robot, P_optimal[i]):
                collision_free ← False
                BREAK

    7. RETURN P_optimal, result.fun

END FUNCTION
```

**Python Implementation (sketch):**

```python
def optimize_trajectory_collision_aware(path, total_time, N=50):
    from scipy.optimize import minimize
    import numpy as np
    from tools import collision, distanceToObstacle

    # Get good initialization from Level 1
    P_init, _ = optimize_trajectory_qp_soft(path, total_time, N)
    x0 = P_init.flatten()

    nq = len(path[0])

    def cost(x):
        P = x.reshape((N, nq))

        # Smoothness
        cost_accel = 0
        for i in range(N - 2):
            accel = P[i+2] - 2*P[i+1] + P[i]
            cost_accel += np.sum(accel**2)

        # Collision penalty
        cost_collision = 0
        for i in range(N):
            q = P[i]
            if collision(robot, q):
                cost_collision += 10000.0
            else:
                d = distanceToObstacle(robot, q)
                if d < 0.05:  # safety margin
                    cost_collision += 1000.0 / (d + 0.001)

        return cost_accel + cost_collision

    # Constraints
    constraints = [
        {'type': 'eq', 'fun': lambda x: x[:nq] - path[0]},
        {'type': 'eq', 'fun': lambda x: x[-nq:] - path[-1]},
        {'type': 'eq', 'fun': lambda x: x[:nq] - x[nq:2*nq]},
        {'type': 'eq', 'fun': lambda x: x[-nq:] - x[-2*nq:-nq]},
    ]

    # Solve
    result = minimize(cost, x0, method='SLSQP',
                     constraints=constraints,
                     options={'maxiter': 1000, 'disp': True})

    P_optimal = result.x.reshape((N, nq))
    return P_optimal, result.fun
```

---

<a name="control"></a>
# Task II: Control Law Algorithms

<a name="pd-control"></a>
## PD Control with Gravity Compensation

```
FUNCTION controllaw_pd(sim, robot, trajs, t_current, cube):
    """
    PD control with gravity/Coriolis compensation

    Complexity: O(nq^2) for dynamics computation
    Time to implement: 1-2 hours
    """

    INPUT:
        sim: PyBullet simulation object
        robot: Pinocchio robot model
        trajs: (q_traj, v_traj, a_traj)
        t_current: current time (s)
        cube: cube object

    OUTPUT:
        None (applies torques to sim)

    ALGORITHM:

    1. UNPACK TRAJECTORIES
        q_traj, v_traj, a_traj ← trajs

    2. GET CURRENT STATE FROM SIMULATOR
        q, vq ← sim.getpybulletstate()

    3. GET DESIRED STATE FROM TRAJECTORY
        q_des ← q_traj(t_current)
        vq_des ← v_traj(t_current)

    4. COMPUTE ERRORS
        e_pos ← q_des - q
        e_vel ← vq_des - vq

    5. SET CONTROL GAINS
        Kp ← 300.0  // or tune per joint
        Kd ← 2 * sqrt(Kp)  // critical damping ≈ 34.6

    6. COMPUTE GRAVITY AND CORIOLIS TERMS
        pin.computeAllTerms(robot.model, robot.data, q, vq)
        g ← pin.nle(robot.model, robot.data, q, vq)
        // nle = non-linear effects = Coriolis + gravity

    7. COMPUTE CONTROL TORQUES
        tau ← Kp * e_pos + Kd * e_vel + g

    8. APPLY TORQUES TO SIMULATOR
        sim.step(tau)

END FUNCTION
```

**Python Implementation:**

```python
def controllaw_pd(sim, robot, trajs, tcurrent, cube):
    import pinocchio as pin
    import numpy as np

    # Unpack
    q_traj, v_traj, a_traj = trajs

    # Current state
    q, vq = sim.getpybulletstate()

    # Desired state
    q_des = q_traj(tcurrent)
    vq_des = v_traj(tcurrent)

    # Errors
    e_pos = q_des - q
    e_vel = vq_des - vq

    # Gains
    Kp = 300.0
    Kd = 2.0 * np.sqrt(Kp)

    # Gravity compensation
    pin.computeAllTerms(robot.model, robot.data, q, vq)
    g = pin.nle(robot.model, robot.data, q, vq)

    # Control law
    torques = Kp * e_pos + Kd * e_vel + g

    # Apply
    sim.step(torques)
```

---

<a name="inverse-dynamics"></a>
## Inverse Dynamics Control

```
FUNCTION controllaw_inverse_dynamics(sim, robot, trajs, t_current, cube):
    """
    Inverse dynamics control (computed torque control)
    Better tracking than PD control

    Complexity: O(nq^2) for mass matrix
    Time to implement: 2-3 hours
    """

    INPUT: (same as PD control)
    OUTPUT: (same)

    ALGORITHM:

    1. UNPACK AND GET STATES
        q_traj, v_traj, a_traj ← trajs
        q, vq ← sim.getpybulletstate()
        q_des ← q_traj(t_current)
        vq_des ← v_traj(t_current)
        aq_des ← a_traj(t_current)  // USE DESIRED ACCELERATION!

    2. COMPUTE ERRORS
        e_pos ← q_des - q
        e_vel ← vq_des - vq

    3. COMPUTE COMMANDED ACCELERATION (with feedback)
        Kp ← 300.0
        Kd ← 2 * sqrt(Kp)
        aq_cmd ← aq_des + Kp * e_pos + Kd * e_vel
        // This is the key difference: feedforward + feedback

    4. COMPUTE DYNAMICS TERMS
        pin.computeAllTerms(robot.model, robot.data, q, vq)

        // Mass matrix (via Composite Rigid Body Algorithm)
        M ← pin.crba(robot.model, robot.data, q)

        // Non-linear effects (Coriolis + gravity)
        h ← pin.nle(robot.model, robot.data, q, vq)

    5. INVERSE DYNAMICS: tau = M * aq_cmd + h
        tau ← M @ aq_cmd + h

    6. APPLY
        sim.step(tau)

END FUNCTION
```

**Python Implementation:**

```python
def controllaw_inverse_dynamics(sim, robot, trajs, tcurrent, cube):
    import pinocchio as pin
    import numpy as np

    q_traj, v_traj, a_traj = trajs

    # Current state
    q, vq = sim.getpybulletstate()

    # Desired state
    q_des = q_traj(tcurrent)
    vq_des = v_traj(tcurrent)
    aq_des = a_traj(tcurrent)  # Important!

    # Errors
    e_pos = q_des - q
    e_vel = vq_des - vq

    # Gains
    Kp = 300.0
    Kd = 2.0 * np.sqrt(Kp)

    # Commanded acceleration
    aq_cmd = aq_des + Kp * e_pos + Kd * e_vel

    # Compute dynamics
    pin.computeAllTerms(robot.model, robot.data, q, vq)
    M = pin.crba(robot.model, robot.data, q)
    h = pin.nle(robot.model, robot.data, q, vq)

    # Inverse dynamics
    torques = M @ aq_cmd + h

    # Apply
    sim.step(torques)
```

---

<a name="force-control"></a>
## Inverse Dynamics + Force Control for Grasping

```
FUNCTION controllaw_with_grasping(sim, robot, cube, trajs, t_current):
    """
    Inverse dynamics + force control for maintaining grasp
    Best for manipulation tasks

    Complexity: O(nq^2) for dynamics + O(nq) for Jacobians
    Time to implement: 3-5 hours
    """

    INPUT:
        sim, robot, cube, trajs, t_current

    OUTPUT: (applies torques)

    ALGORITHM:

    1. BASELINE CONTROL (same as inverse dynamics)
        q, vq ← sim.getpybulletstate()
        q_des, vq_des, aq_des ← trajs at t_current

        e_pos ← q_des - q
        e_vel ← vq_des - vq

        Kp ← 300.0
        Kd ← 2 * sqrt(Kp)
        aq_cmd ← aq_des + Kp * e_pos + Kd * e_vel

        pin.computeAllTerms(robot.model, robot.data, q, vq)
        M ← pin.crba(robot.model, robot.data, q)
        h ← pin.nle(robot.model, robot.data, q, vq)

        tau_baseline ← M @ aq_cmd + h

    2. COMPUTE GRASPING FORCES

        // 2a. Get current end-effector poses
        pin.framesForwardKinematics(robot.model, robot.data, q)

        IDX_LEFT ← robot.model.getFrameId(LEFT_HAND)
        IDX_RIGHT ← robot.model.getFrameId(RIGHT_HAND)

        oM_left ← robot.data.oMf[IDX_LEFT]
        oM_right ← robot.data.oMf[IDX_RIGHT]

        // 2b. Get desired hook poses on cube
        oM_cube_left ← getcubeplacement(cube, LEFT_HOOK)
        oM_cube_right ← getcubeplacement(cube, RIGHT_HOOK)

        // 2c. Compute SE3 errors
        // log: SE3 -> se3 (6D vector: linear + angular)
        error_left ← pin.log(oM_left.inverse() * oM_cube_left).vector
        error_right ← pin.log(oM_right.inverse() * oM_cube_right).vector

        // 2d. Desired forces (proportional control in task space)
        Kf ← 50.0  // force gain (tune as needed)
        F_left ← Kf * error_left
        F_right ← Kf * error_right

    3. MAP FORCES TO JOINT TORQUES

        // 3a. Compute Jacobians
        pin.computeJointJacobians(robot.model, robot.data, q)

        J_left ← pin.computeFrameJacobian(robot.model, robot.data, q, IDX_LEFT)
        J_right ← pin.computeFrameJacobian(robot.model, robot.data, q, IDX_RIGHT)
        // Shape: (6, nq) each

        // 3b. Map forces to torques: tau = J^T * F
        tau_grasp ← J_left.T @ F_left + J_right.T @ F_right

    4. COMBINED CONTROL
        tau ← tau_baseline + tau_grasp

    5. SAFETY CHECK (optional but recommended)
        IF any(isnan(tau)) OR any(isinf(tau)):
            PRINT "Warning: invalid torques, setting to zero"
            tau ← zeros_like(tau)

        // Torque limits (optional)
        tau_max ← 100.0  // example
        tau ← clip(tau, -tau_max, tau_max)

    6. APPLY
        sim.step(tau)

END FUNCTION
```

**Python Implementation:**

```python
def controllaw_with_grasping(sim, robot, cube, trajs, tcurrent):
    import pinocchio as pin
    import numpy as np
    from config import LEFT_HAND, RIGHT_HAND, LEFT_HOOK, RIGHT_HOOK
    from tools import getcubeplacement

    q_traj, v_traj, a_traj = trajs

    # --- Baseline inverse dynamics control ---
    q, vq = sim.getpybulletstate()
    q_des = q_traj(tcurrent)
    vq_des = v_traj(tcurrent)
    aq_des = a_traj(tcurrent)

    e_pos = q_des - q
    e_vel = vq_des - vq

    Kp = 300.0
    Kd = 2.0 * np.sqrt(Kp)
    aq_cmd = aq_des + Kp * e_pos + Kd * e_vel

    pin.computeAllTerms(robot.model, robot.data, q, vq)
    M = pin.crba(robot.model, robot.data, q)
    h = pin.nle(robot.model, robot.data, q, vq)

    tau_baseline = M @ aq_cmd + h

    # --- Grasping force control ---
    pin.framesForwardKinematics(robot.model, robot.data, q)

    IDX_LEFT = robot.model.getFrameId(LEFT_HAND)
    IDX_RIGHT = robot.model.getFrameId(RIGHT_HAND)

    oM_left = robot.data.oMf[IDX_LEFT]
    oM_right = robot.data.oMf[IDX_RIGHT]

    oM_cube_left = getcubeplacement(cube, LEFT_HOOK)
    oM_cube_right = getcubeplacement(cube, RIGHT_HOOK)

    error_left = pin.log(oM_left.inverse() * oM_cube_left).vector
    error_right = pin.log(oM_right.inverse() * oM_cube_right).vector

    Kf = 50.0
    F_left = Kf * error_left
    F_right = Kf * error_right

    # Jacobians
    pin.computeJointJacobians(robot.model, robot.data, q)
    J_left = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_LEFT)
    J_right = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_RIGHT)

    tau_grasp = J_left.T @ F_left + J_right.T @ F_right

    # Combined
    torques = tau_baseline + tau_grasp

    # Safety
    if np.any(np.isnan(torques)) or np.any(np.isinf(torques)):
        print("Warning: invalid torques detected!")
        torques = np.zeros_like(torques)

    # Apply
    sim.step(torques)
```

---

<a name="integration"></a>
# Complete Integration Algorithm

```
ALGORITHM: Complete Lab Part 2 Pipeline

MAIN PROGRAM:

1. SETUP ENVIRONMENT
    robot, sim, cube ← setupwithpybullet()

2. COMPUTE GRASPING CONFIGURATIONS (Part 1)
    q0, success0 ← computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe, successe ← computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, None)

    IF NOT (success0 AND successe):
        ERROR "Invalid initial or end configuration"
        EXIT

3. COMPUTE PATH (Part 1)
    path, path_success ← computepath(q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    IF NOT path_success:
        WARNING "Path computation may be incomplete"

    PRINT "Path length:", len(path), "waypoints"

4. CREATE TRAJECTORY (Part 2 Task I)
    // Choose implementation level:

    // LEVEL 0 (simple):
    total_time ← 10.0
    trajs ← create_trajectory_level0_smooth(path, total_time)

    // LEVEL 1 (recommended):
    N ← 50
    trajs, total_time ← create_trajectory_level1(path, None, N)

    q_traj, v_traj, a_traj ← trajs

5. VALIDATE TRAJECTORY (optional but recommended)
    ASSERT norm(q_traj(0.0) - q0) < 1e-3, "Trajectory doesn't start at q0"
    ASSERT norm(q_traj(total_time) - qe) < 1e-3, "Trajectory doesn't end at qe"
    ASSERT norm(v_traj(0.0)) < 1e-3, "Non-zero initial velocity"
    ASSERT norm(v_traj(total_time)) < 1e-3, "Non-zero final velocity"
    PRINT "Trajectory validation: PASSED"

6. SET INITIAL CONFIGURATION IN SIMULATOR
    sim.setqsim(q0)
    PRINT "Initial configuration set"

7. CONTROL LOOP (Part 2 Task II)
    t ← 0.0
    DT ← 1e-3  // from config.py

    PRINT "Starting control loop..."

    WHILE t < total_time:
        // Call control law
        // Choose: controllaw_pd, controllaw_inverse_dynamics,
        //         or controllaw_with_grasping
        controllaw_inverse_dynamics(sim, robot, trajs, t, cube)

        t ← t + DT

        // Optional: visualization
        IF t mod 0.1 < DT:  // every 0.1 seconds
            q_current, _ ← sim.getpybulletstate()
            PRINT "Time:", t, "  Tracking error:", norm(q_current - q_traj(t))

8. FINAL VALIDATION
    q_final, _ ← sim.getpybulletstate()
    cube_final ← getcubeplacement(cube)
    error_final ← norm(cube_final.translation - CUBE_PLACEMENT_TARGET.translation)

    PRINT "Task complete!"
    PRINT "Final configuration error:", norm(q_final - qe)
    PRINT "Final cube position error:", error_final

    IF error_final < 0.05:
        PRINT "SUCCESS: Cube reached target"
    ELSE:
        PRINT "PARTIAL SUCCESS: Cube did not reach target precisely"

9. CLEANUP
    // Optional: record video, save results, etc.

END MAIN
```

**Complete Python Implementation:**

```python
def main():
    from tools import setupwithpybullet
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, DT
    from inverse_geometry import computeqgrasppose
    from path import computepath
    import numpy as np

    print("=== Part 2: Dynamics and Control ===")

    # 1. Setup
    print("Setting up environment...")
    robot, sim, cube = setupwithpybullet()

    # 2. Compute grasping configurations
    print("Computing grasping configurations...")
    q0, success0 = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe, successe = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, None)

    if not (success0 and successe):
        print("ERROR: Invalid initial or end configuration")
        return

    # 3. Compute path
    print("Computing path...")
    path, path_success = computepath(q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    print(f"Path computed: {len(path)} waypoints")

    # 4. Create trajectory
    print("Creating trajectory...")

    # LEVEL 0:
    # trajs = create_trajectory_level0_smooth(path, total_time=10.0)
    # total_time = 10.0

    # LEVEL 1:
    trajs, total_time = create_trajectory_level1(path, total_time=None, N=50)

    q_traj, v_traj, a_traj = trajs
    print(f"Trajectory created: T = {total_time:.2f}s")

    # 5. Validate
    assert np.linalg.norm(q_traj(0.0) - q0) < 1e-3
    assert np.linalg.norm(q_traj(total_time) - qe) < 1e-3
    assert np.linalg.norm(v_traj(0.0)) < 1e-3
    assert np.linalg.norm(v_traj(total_time)) < 1e-3
    print("Trajectory validation: PASSED")

    # 6. Set initial configuration
    sim.setqsim(q0)
    print("Initial configuration set")

    # 7. Control loop
    print("Starting control loop...")
    t = 0.0
    while t < total_time:
        # Choose your control law:
        controllaw_inverse_dynamics(sim, robot, trajs, t, cube)
        # Or: controllaw_with_grasping(sim, robot, cube, trajs, t)

        t += DT

        # Progress monitoring
        if t % 1.0 < DT:  # every second
            q_current, _ = sim.getpybulletstate()
            error = np.linalg.norm(q_current - q_traj(t))
            print(f"t = {t:.1f}s, tracking error = {error:.4f}")

    # 8. Final validation
    from tools import getcubeplacement
    q_final, _ = sim.getpybulletstate()
    cube_final = getcubeplacement(cube)
    error_pos = np.linalg.norm(cube_final.translation - CUBE_PLACEMENT_TARGET.translation)

    print("\n=== Task Complete ===")
    print(f"Final cube position error: {error_pos:.4f}m")
    if error_pos < 0.05:
        print("SUCCESS!")
    else:
        print("Partial success")

if __name__ == "__main__":
    main()
```

---

<a name="helpers"></a>
# Helper Functions and Utilities

## Trajectory Evaluation

```python
def evaluate_trajectory(path, q_traj, v_traj, a_traj, total_time):
    """Evaluate trajectory quality metrics"""
    import numpy as np

    M = len(path)
    errors = []

    for i in range(M):
        t = i / (M - 1) * total_time
        q_t = q_traj(t)
        error = np.linalg.norm(q_t - path[i])
        errors.append(error)

    times = np.linspace(0, total_time, 100)
    accels = [np.linalg.norm(a_traj(t)) for t in times]

    return {
        'max_path_error': max(errors),
        'avg_path_error': np.mean(errors),
        'max_accel': max(accels),
        'avg_accel': np.mean(accels),
        'initial_vel': np.linalg.norm(v_traj(0.0)),
        'final_vel': np.linalg.norm(v_traj(total_time))
    }
```

## Visualization

```python
def visualize_trajectory(robot, cube, viz, q_traj, total_time, dt=0.05):
    """Visualize trajectory in MeshCat"""
    import numpy as np
    import time

    times = np.arange(0, total_time, dt)

    for t in times:
        q = q_traj(t)
        viz.display(q)
        time.sleep(dt)

def plot_trajectory(q_traj, v_traj, a_traj, total_time, nq):
    """Plot trajectory in configuration space"""
    import matplotlib.pyplot as plt
    import numpy as np

    times = np.linspace(0, total_time, 200)
    qs = np.array([q_traj(t) for t in times])
    vqs = np.array([v_traj(t) for t in times])
    aqs = np.array([a_traj(t) for t in times])

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Positions
    for j in range(min(nq, 5)):  # plot first 5 joints
        axes[0].plot(times, qs[:, j], label=f'q{j}')
    axes[0].set_ylabel('Position (rad)')
    axes[0].legend()
    axes[0].grid()

    # Velocities
    for j in range(min(nq, 5)):
        axes[1].plot(times, vqs[:, j], label=f'vq{j}')
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].legend()
    axes[1].grid()

    # Accelerations
    for j in range(min(nq, 5)):
        axes[2].plot(times, aqs[:, j], label=f'aq{j}')
    axes[2].set_ylabel('Acceleration (rad/s²)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()
    axes[2].grid()

    plt.tight_layout()
    plt.show()
```

## Control Performance Monitoring

```python
def monitor_control_performance(sim, robot, trajs, total_time, DT):
    """Run control loop and record performance metrics"""
    import numpy as np

    q_traj, v_traj, a_traj = trajs

    tracking_errors = []
    times = []

    t = 0.0
    while t < total_time:
        q, vq = sim.getpybulletstate()
        q_des = q_traj(t)

        error = np.linalg.norm(q - q_des)
        tracking_errors.append(error)
        times.append(t)

        # Apply control
        controllaw_inverse_dynamics(sim, robot, trajs, t, None)

        t += DT

    return np.array(times), np.array(tracking_errors)

def plot_tracking_errors(times, errors):
    """Plot tracking errors over time"""
    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 6))
    plt.plot(times, errors)
    plt.xlabel('Time (s)')
    plt.ylabel('Tracking Error (rad)')
    plt.title('Control Tracking Performance')
    plt.grid()
    plt.show()

    print(f"Max tracking error: {np.max(errors):.4f}")
    print(f"Mean tracking error: {np.mean(errors):.4f}")
    print(f"Final tracking error: {errors[-1]:.4f}")
```

---

## Summary of Implementation Order

1. **Start Here (Day 1-2):**
   - Algorithm 1: Simple piecewise Bezier (Level 0)
   - PD control algorithm

2. **Next (Day 3-5):**
   - Test and debug
   - Algorithm 2: Smooth trajectory (Level 0 improved)
   - Inverse dynamics control

3. **Advanced (Week 2):**
   - Algorithm 3: QP optimization (Level 1)
   - Algorithm 6: Control points to trajectory
   - Integration testing

4. **Optional Enhancements:**
   - Algorithm 4: QP with bounds
   - Algorithm 5: Time optimization
   - Force control for grasping
   - Algorithm 7: Collision-aware (Level 2)

---

**End of Pseudocode Document**

Use this as a reference while implementing. Copy the algorithms that match your chosen level and adapt as needed!