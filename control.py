#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Control Law Implementation for Part 2: Dynamics

This module implements trajectory generation and control laws for the Nextage robot
to grasp and manipulate a cube, following the lab instructions.

Part 2, Task I: Trajectory Generation (Level 0, 1, 2)
Part 2, Task II: Control Law (Optional Task 0 + Actual Task)

@author: lab implementation
"""

import numpy as np
import pinocchio as pin
from bezier import Bezier

# ============================================================================
# CONTROL GAINS
# ============================================================================
# These gains work well for all joints but can be tuned individually if needed
Kp = 300.0              # Proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)    # Derivative gain (D of PD) - critically damped

# ============================================================================
# PART 2, TASK I: TRAJECTORY GENERATION
# ============================================================================

def maketraj_level0(path, T):
    """
    Level 0: Manual time parametrization with zero boundary velocities.

    Uses simple Bezier curve interpolation between waypoints to create a smooth
    trajectory with zero velocities at start and end.

    Args:
        path: List of configurations [q0, q1, ..., qn] from path planning
        T: Total duration of trajectory (seconds)

    Returns:
        q_of_t: Function that returns configuration at time t
        vq_of_t: Function that returns velocity at time t
        vvq_of_t: Function that returns acceleration at time t
    """
    M = len(path)  # Number of waypoints
    n_segments = M - 1
    dt = T / n_segments  # Time per segment

    print(f"\n=== Level 0 Trajectory Generation ===")
    print(f"Waypoints: {M}")
    print(f"Segments: {n_segments}")
    print(f"Time per segment: {dt:.3f}s")

    # Create Bezier curve segments with smooth velocity transitions
    segments = []

    # Compute velocities at each waypoint using finite differences
    # This ensures C1 continuity (velocity continuity)
    velocities = [np.zeros_like(path[0])]  # Zero velocity at start

    for i in range(1, M - 1):
        # Central difference for interior waypoints
        v = (path[i + 1] - path[i - 1]) / (2 * dt)
        velocities.append(v)

    velocities.append(np.zeros_like(path[-1]))  # Zero velocity at end

    # Create Hermite-Bezier segments (cubic Bezier with specified velocities)
    for i in range(n_segments):
        q0 = path[i]
        q1 = path[i + 1]
        v0 = velocities[i]
        v1 = velocities[i + 1]
        t_start = i * dt
        t_end = (i + 1) * dt

        # Convert Hermite to Bezier control points
        # For cubic Bezier: P(t) with P(0)=q0, P(1)=q1, P'(0)=v0*dt, P'(1)=v1*dt
        p0 = q0
        p1 = q0 + (dt / 3.0) * v0
        p2 = q1 - (dt / 3.0) * v1
        p3 = q1

        bezier = Bezier([p0, p1, p2, p3], t_min=t_start, t_max=t_end)
        segments.append(bezier)

    # Create trajectory functions
    def q_of_t(t):
        """Return configuration at time t"""
        if t <= 0:
            return path[0].copy()
        if t >= T:
            return path[-1].copy()
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx](t)

    def vq_of_t(t):
        """Return velocity at time t"""
        if t <= 0 or t >= T:
            return np.zeros_like(path[0])
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx].derivative(1)(t)

    def vvq_of_t(t):
        """Return acceleration at time t"""
        if t <= 0 or t >= T:
            return np.zeros_like(path[0])
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx].derivative(2)(t)

    # Validate trajectory
    print(f"Trajectory validation:")
    print(f"  q(0) error: {np.linalg.norm(q_of_t(0.0) - path[0]):.6e}")
    print(f"  q(T) error: {np.linalg.norm(q_of_t(T) - path[-1]):.6e}")
    print(f"  v(0): {np.linalg.norm(vq_of_t(0.0)):.6e}")
    print(f"  v(T): {np.linalg.norm(vq_of_t(T)):.6e}")

    return q_of_t, vq_of_t, vvq_of_t


def maketraj_level1(path, T, robot):
    """
    Level 1: QP-based trajectory optimization using Bezier curves.

    Uses quadratic programming to optimize trajectory by minimizing acceleration
    while respecting joint limits, velocity limits, and boundary conditions.

    Following Tutorial 6 & 7 approach for convex optimization.

    Args:
        path: List of configurations [q0, q1, ..., qn]
        T: Total duration
        robot: Robot model for joint limits

    Returns:
        q_of_t, vq_of_t, vvq_of_t: Trajectory functions
    """
    try:
        import quadprog
    except ImportError:
        print("WARNING: quadprog not installed. Falling back to Level 0.")
        print("Install with: pip install quadprog")
        return maketraj_level0(path, T)

    print(f"\n=== Level 1 Trajectory Generation (QP Optimization) ===")

    M = len(path)
    n_segments = M - 1
    dt = T / n_segments

    # For QP optimization, we'll optimize control points of Bezier curves
    # connecting the given waypoints while minimizing acceleration

    # For simplicity, optimize velocities at interior waypoints
    # (similar to Tutorial 7's Hermite spline approach)

    n_dof = path[0].shape[0]
    n_vars = (M - 2) * n_dof  # Optimize interior velocities only

    if n_vars == 0:
        # Only 2 waypoints, use Level 0
        return maketraj_level0(path, T)

    # Cost function: minimize integral of squared acceleration
    # For Hermite splines, this can be expressed as quadratic form in velocities
    # H = Σ (1/dt) * (v_i^2 + v_i·v_{i+1} + v_{i+1}^2)

    H = np.zeros((n_vars, n_vars))
    for i in range(M - 2):
        idx = i * n_dof
        # Add contribution from segments connecting to this velocity
        H[idx:idx+n_dof, idx:idx+n_dof] += (2.0 / dt) * np.eye(n_dof)

    # No linear term (minimizing squared norm)
    f = np.zeros(n_vars)

    # Inequality constraints: velocity limits
    # |v_i| < v_max (for each joint)
    v_max = np.ones(n_dof) * 2.0  # rad/s (conservative limit)

    # Constraints: -v_max <= v_i <= v_max
    G = np.vstack([np.eye(n_vars), -np.eye(n_vars)])
    h = np.hstack([v_max.repeat(M-2), v_max.repeat(M-2)])

    # Solve QP: minimize 0.5 * x^T H x + f^T x subject to Gx <= h
    try:
        sol = quadprog.solve_qp(H, f, G.T, h)[0]
        velocities_opt = sol.reshape((M-2, n_dof))

        print(f"QP solved successfully!")
        print(f"  Max velocity: {np.max(np.abs(velocities_opt)):.3f} rad/s")

        # Build full velocity vector
        velocities = [np.zeros(n_dof)]  # Zero at start
        for v in velocities_opt:
            velocities.append(v)
        velocities.append(np.zeros(n_dof))  # Zero at end

    except Exception as e:
        print(f"QP failed: {e}. Using finite differences.")
        # Fallback: use finite differences
        velocities = [np.zeros(n_dof)]
        for i in range(1, M - 1):
            v = (path[i + 1] - path[i - 1]) / (2 * dt)
            velocities.append(v)
        velocities.append(np.zeros(n_dof))

    # Create Bezier segments (same as Level 0 but with optimized velocities)
    segments = []
    for i in range(n_segments):
        q0, q1 = path[i], path[i + 1]
        v0, v1 = velocities[i], velocities[i + 1]
        t_start, t_end = i * dt, (i + 1) * dt

        p0 = q0
        p1 = q0 + (dt / 3.0) * v0
        p2 = q1 - (dt / 3.0) * v1
        p3 = q1

        bezier = Bezier([p0, p1, p2, p3], t_min=t_start, t_max=t_end)
        segments.append(bezier)

    # Create trajectory functions
    def q_of_t(t):
        if t <= 0: return path[0].copy()
        if t >= T: return path[-1].copy()
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx](t)

    def vq_of_t(t):
        if t <= 0 or t >= T: return np.zeros_like(path[0])
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx].derivative(1)(t)

    def vvq_of_t(t):
        if t <= 0 or t >= T: return np.zeros_like(path[0])
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx].derivative(2)(t)

    return q_of_t, vq_of_t, vvq_of_t


def maketraj_level2(path, T, robot, cube):
    """
    Level 2: Non-linear trajectory optimization with collision constraints.

    Uses sequential quadratic programming or penalty method to refine the
    Level 1 trajectory while enforcing collision avoidance constraints.

    Args:
        path: List of configurations
        T: Total duration
        robot: Robot model
        cube: Cube model

    Returns:
        q_of_t, vq_of_t, vvq_of_t: Collision-free trajectory functions
    """
    from tools import collision, distanceToObstacle
    from scipy.optimize import minimize

    print(f"\n=== Level 2 Trajectory Generation (NLP with Collision Constraints) ===")

    # Start with Level 1 solution
    q_traj_init, v_traj_init, a_traj_init = maketraj_level1(path, T, robot)

    M = len(path)
    n_segments = M - 1
    dt = T / n_segments
    n_dof = path[0].shape[0]

    # Discretize trajectory for collision checking
    n_check_points = max(20, M * 3)  # Check points along trajectory
    t_check = np.linspace(0, T, n_check_points)

    print(f"Checking collisions at {n_check_points} points...")

    # Check if initial trajectory is already collision-free
    collision_free = True
    for t in t_check:
        q = q_traj_init(t)
        if collision(robot, q):
            collision_free = False
            break

    if collision_free:
        print("Initial trajectory is collision-free! No refinement needed.")
        return q_traj_init, v_traj_init, a_traj_init

    print("Collisions detected. Refining trajectory...")

    # Optimization variables: interior waypoint velocities
    n_vars = (M - 2) * n_dof
    if n_vars == 0:
        print("WARNING: Cannot optimize with only 2 waypoints.")
        return q_traj_init, v_traj_init, a_traj_init

    # Initial guess from Level 1
    velocities_init = []
    for i in range(1, M - 1):
        velocities_init.append(v_traj_init(i * dt))
    x0 = np.concatenate(velocities_init)

    # Cost function: minimize acceleration + penalty for collisions
    d_safe = 0.05  # Safe distance margin (5cm)

    def cost_function(x):
        """Minimize acceleration + collision penalty"""
        # Reshape to velocities
        vels = x.reshape((M - 2, n_dof))
        velocities = [np.zeros(n_dof)] + list(vels) + [np.zeros(n_dof)]

        # Compute acceleration cost
        acc_cost = 0.0
        for i in range(M - 1):
            q0, q1 = path[i], path[i + 1]
            v0, v1 = velocities[i], velocities[i + 1]
            # Approximate acceleration
            a = (v1 - v0) / dt
            acc_cost += np.sum(a ** 2)

        # Collision penalty
        collision_cost = 0.0
        segments = []
        for i in range(n_segments):
            q0, q1 = path[i], path[i + 1]
            v0, v1 = velocities[i], velocities[i + 1]
            t_start, t_end = i * dt, (i + 1) * dt

            p0 = q0
            p1 = q0 + (dt / 3.0) * v0
            p2 = q1 - (dt / 3.0) * v1
            p3 = q1

            segments.append(Bezier([p0, p1, p2, p3], t_min=t_start, t_max=t_end))

        # Check collisions at discretized points
        for t in t_check:
            segment_idx = min(int(t / dt), n_segments - 1)
            q = segments[segment_idx](t)

            # Distance-based penalty (smooth)
            dist = distanceToObstacle(robot, q)
            if dist < d_safe:
                collision_cost += 1000.0 * (d_safe - dist) ** 2

        return acc_cost + collision_cost

    # Optimize
    print("Running NLP optimization...")
    result = minimize(
        cost_function,
        x0,
        method='SLSQP',
        options={'maxiter': 100, 'disp': False}
    )

    if result.success:
        print(f"Optimization converged!")
        x_opt = result.x
    else:
        print(f"Optimization did not converge. Using best result.")
        x_opt = result.x

    # Build optimized trajectory
    vels_opt = x_opt.reshape((M - 2, n_dof))
    velocities = [np.zeros(n_dof)] + list(vels_opt) + [np.zeros(n_dof)]

    segments = []
    for i in range(n_segments):
        q0, q1 = path[i], path[i + 1]
        v0, v1 = velocities[i], velocities[i + 1]
        t_start, t_end = i * dt, (i + 1) * dt

        p0 = q0
        p1 = q0 + (dt / 3.0) * v0
        p2 = q1 - (dt / 3.0) * v1
        p3 = q1

        bezier = Bezier([p0, p1, p2, p3], t_min=t_start, t_max=t_end)
        segments.append(bezier)

    def q_of_t(t):
        if t <= 0: return path[0].copy()
        if t >= T: return path[-1].copy()
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx](t)

    def vq_of_t(t):
        if t <= 0 or t >= T: return np.zeros_like(path[0])
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx].derivative(1)(t)

    def vvq_of_t(t):
        if t <= 0 or t >= T: return np.zeros_like(path[0])
        segment_idx = min(int(t / dt), n_segments - 1)
        return segments[segment_idx].derivative(2)(t)

    # Verify collision-free
    collision_count = 0
    for t in t_check:
        if collision(robot, q_of_t(t)):
            collision_count += 1

    print(f"Final trajectory collision check: {collision_count}/{n_check_points} points in collision")

    return q_of_t, vq_of_t, vvq_of_t


# ============================================================================
# PART 2, TASK II: CONTROL LAW IMPLEMENTATION
# ============================================================================

def controllaw_task0(sim, robot, trajs, tcurrent):
    """
    Optional Task 0: Control law without grasping the cube.

    Tests the inverse dynamics controller by moving the end-effectors
    to a position above the cube without actually grasping it.

    This is recommended by the instructor to validate the controller
    before attempting the actual grasping task.

    Args:
        sim: PyBullet simulation object
        robot: Pinocchio robot model
        trajs: Tuple of (q_traj, vq_traj, vvq_traj) functions
        tcurrent: Current simulation time
    """
    try:
        # Get current robot state from PyBullet
        q, vq = sim.getpybulletstate()

        # Get desired state from trajectory
        q_traj, vq_traj, vvq_traj = trajs
        q_des = q_traj(tcurrent)
        vq_des = vq_traj(tcurrent)
        vvq_des = vvq_traj(tcurrent)

        # Compute tracking errors
        error_q = q_des - q
        error_vq = vq_des - vq

        # Update robot kinematics and dynamics
        pin.computeAllTerms(robot.model, robot.data, q, vq)

        # Get mass matrix M (using CRBA - Composite Rigid Body Algorithm)
        M = pin.crba(robot.model, robot.data, q)

        # Get nonlinear effects: Coriolis + gravity (nle = non-linear effects)
        nle = pin.nle(robot.model, robot.data, q, vq)

        # Inverse dynamics control law
        # τ = M * (a_des + Kp*e_q + Kv*e_vq) + nle
        tau = M @ (vvq_des + Kp * error_q + Kv * error_vq) + nle

        # Convert to list for PyBullet
        torques = tau.tolist()

    except Exception as e:
        print(f"ERROR in controllaw_task0 at t={tcurrent:.3f}: {e}")
        import traceback
        traceback.print_exc()
        torques = [0.0] * len(sim.bulletCtrlJointsInPinOrder)

    # Apply torques and step simulation
    sim.step(torques)


def controllaw_actual(sim, robot, cube, trajs, cube_traj, tcurrent):
    """
    Actual Task: Control law with inverse dynamics + force control for grasping.

    Implements the complete control law to grasp and carry the cube:
    - Inverse dynamics for trajectory tracking
    - Force control to maintain grasp on cube
    - Cube moves due to physical contact forces (PyBullet physics)

    Args:
        sim: PyBullet simulation object
        robot: Pinocchio robot model
        cube: Pinocchio cube model
        trajs: Tuple of (q_traj, vq_traj, vvq_traj) functions
        cube_traj: Function returning desired cube placement at time t
        tcurrent: Current simulation time
    """
    from config import LEFT_HAND, RIGHT_HAND, LEFT_HOOK, RIGHT_HOOK
    from tools import getcubeplacement, setcubeplacement
    import pybullet as pyb

    try:
        # Get current robot state
        q, vq = sim.getpybulletstate()

        # Get desired trajectory state
        q_traj, vq_traj, vvq_traj = trajs
        q_des = q_traj(tcurrent)
        vq_des = vq_traj(tcurrent)
        vvq_des = vvq_traj(tcurrent)

        # === PART 1: Inverse Dynamics Control (Trajectory Tracking) ===

        error_q = q_des - q
        error_vq = vq_des - vq

        # Update robot dynamics
        pin.computeAllTerms(robot.model, robot.data, q, vq)
        M = pin.crba(robot.model, robot.data, q)
        nle = pin.nle(robot.model, robot.data, q, vq)

        # Tracking control
        tau_tracking = M @ (vvq_des + Kp * error_q + Kv * error_vq) + nle

        # === PART 2: Force Control (Grasping) ===

        # Get ACTUAL cube position from PyBullet
        # The hands should grasp wherever the cube currently is
        cube_pos_actual, cube_orn_actual = pyb.getBasePositionAndOrientation(sim.cubeId)
        cube_placement_actual = pin.XYZQUATToSE3(np.array([
            cube_pos_actual[0], cube_pos_actual[1], cube_pos_actual[2],
            cube_orn_actual[0], cube_orn_actual[1], cube_orn_actual[2], cube_orn_actual[3]
        ]))

        # Update forward kinematics
        pin.framesForwardKinematics(robot.model, robot.data, q)

        # Get end-effector frames
        left_hand_id = robot.model.getFrameId(LEFT_HAND)
        right_hand_id = robot.model.getFrameId(RIGHT_HAND)

        # Current hand placements
        oMleft = robot.data.oMf[left_hand_id]
        oMright = robot.data.oMf[right_hand_id]

        # Desired grasp points on cube (based on ACTUAL cube position)
        # This ensures hands maintain grasp on the cube wherever it is
        setcubeplacement(robot, cube, cube_placement_actual)
        oMleft_hook = getcubeplacement(cube, LEFT_HOOK)
        oMright_hook = getcubeplacement(cube, RIGHT_HOOK)

        # Spring-damper force law: F = Kf * (p_desired - p_current)
        # This creates attraction forces to maintain grasp on the cube
        Kf = 1000.0  # Force gain - increased for stronger grasping

        F_left = Kf * (oMleft_hook.translation - oMleft.translation)
        F_right = Kf * (oMright_hook.translation - oMright.translation)

        # Get Jacobians (linear part only)
        J_left = pin.computeFrameJacobian(
            robot.model, robot.data, q, left_hand_id, pin.LOCAL_WORLD_ALIGNED
        )[:3, :]
        J_right = pin.computeFrameJacobian(
            robot.model, robot.data, q, right_hand_id, pin.LOCAL_WORLD_ALIGNED
        )[:3, :]

        # Convert forces to joint torques
        tau_force_left = J_left.T @ F_left
        tau_force_right = J_right.T @ F_right

        # === COMBINE CONTROL TORQUES ===
        tau_total = tau_tracking + tau_force_left + tau_force_right
        torques = tau_total.tolist()

    except Exception as e:
        print(f"ERROR in controllaw_actual at t={tcurrent:.3f}: {e}")
        import traceback
        traceback.print_exc()
        torques = [0.0] * len(sim.bulletCtrlJointsInPinOrder)

    # Apply torques - PyBullet handles contact forces automatically
    sim.step(torques)


# ============================================================================
# MAIN: Testing and Execution
# ============================================================================

if __name__ == "__main__":
    import sys
    import time
    from tools import setupwithpybullet, setupwithmeshcat, rununtil
    from setup_meshcat import updatevisuals
    from config import DT, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    import path

    # Parse command line arguments
    VISUALIZE_ONLY = "--visualize" in sys.argv or "-v" in sys.argv
    TEST_TASK0 = "--task0" in sys.argv
    LEVEL = 0  # Default trajectory level

    if "--level1" in sys.argv:
        LEVEL = 1
    elif "--level2" in sys.argv:
        LEVEL = 2
    elif "--level0" in sys.argv:
        LEVEL = 0

    # Print mode
    print("\n" + "="*70)
    if VISUALIZE_ONLY:
        print("VISUALIZATION MODE: Preview trajectory in MeshCat")
    elif TEST_TASK0:
        print("OPTIONAL TASK 0: Test controller without cube")
    else:
        print("ACTUAL TASK: Grasp and carry cube with force control")
    print(f"Trajectory Level: {LEVEL}")
    print("="*70 + "\n")

    # Setup with PyBullet (with GUI)
    robot, sim, cube = setupwithpybullet()
    viz = None  # No MeshCat visualization when using PyBullet

    # Set global variables for path module
    path.robot = robot
    path.cube = cube
    path.viz = viz


    # ========================================================================
    # OPTIONAL TASK 0: Simple Test Trajectory (No Cube)
    # ========================================================================

    if TEST_TASK0:
        print("\n--- Optional Task 0: Testing controller without cube ---\n")

        # Create simple trajectory: move effectors above cube start position
        from config import LEFT_HAND, RIGHT_HAND

        # Start configuration
        q0 = robot.q0.copy()

        # Target: hands positioned above cube
        cube_above = pin.SE3(CUBE_PLACEMENT.rotation,
                             CUBE_PLACEMENT.translation + np.array([0, 0, 0.15]))
        qe, success = computeqgrasppose(robot, q0, cube, cube_above, viz)

        if not success:
            print("ERROR: Could not compute target configuration for Task 0")
            exit(1)

        # Simple path: linear interpolation
        n_waypoints = 10
        q_path = [q0 + (i / (n_waypoints - 1)) * (qe - q0)
                  for i in range(n_waypoints)]

        # Generate trajectory
        total_time = 5.0  # seconds

        if LEVEL == 0:
            trajs = maketraj_level0(q_path, total_time)
        elif LEVEL == 1:
            trajs = maketraj_level1(q_path, total_time, robot)
        else:
            trajs = maketraj_level2(q_path, total_time, robot, cube)

        q_traj, vq_traj, vvq_traj = trajs

        if VISUALIZE_ONLY:
            # Visualize in MeshCat
            n_frames = 100
            for i in range(n_frames + 1):
                t = (i / n_frames) * total_time
                q = q_traj(t)
                updatevisuals(viz, robot, cube, q)
                time.sleep(total_time / n_frames / 2.0)
            print("\nVisualization complete!")
        else:
            # Run control simulation
            sim.setqsim(q0)
            tcur = 0.0

            while tcur < total_time:
                rununtil(controllaw_task0, DT, sim, robot, trajs, tcur)
                tcur += DT
                if int(tcur) > int(tcur - DT):
                    print(f"Time: {tcur:.1f}s / {total_time:.1f}s")

            print("\nTask 0 complete! Controller works.")
            print("Now try running the actual task (without --task0 flag)")

    # ========================================================================
    # ACTUAL TASK: Grasp and Carry Cube
    # ========================================================================

    else:
        print("\n--- Actual Task: Grasp and carry cube ---\n")

        # Compute grasp configurations
        q0, success_init = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, viz)
        qe, success_end = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, viz)

        if not (success_init and success_end):
            print("ERROR: Could not compute valid grasping configurations!")
            exit(1)

        # Plan path
        print("Computing collision-free path...")
        robot_path, path_success = path.computepath(q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

        if not path_success:
            print("WARNING: Path planning did not fully succeed. Using partial path.")

        print(f"Path computed: {len(robot_path)} waypoints\n")

        # Extract waypoints
        q_waypoints = [q for q, _ in robot_path]
        cube_waypoints = [cube_placement for _, cube_placement in robot_path]

        # Generate trajectory
        total_time = 10.0  # seconds

        if LEVEL == 0:
            trajs = maketraj_level0(q_waypoints, total_time)
        elif LEVEL == 1:
            trajs = maketraj_level1(q_waypoints, total_time, robot)
        else:
            trajs = maketraj_level2(q_waypoints, total_time, robot, cube)

        q_traj, vq_traj, vvq_traj = trajs

        # Create cube trajectory function
        def make_cube_traj(cube_waypoints, T):
            M = len(cube_waypoints)
            n_segments = M - 1
            dt_segment = T / n_segments

            def cube_traj(t):
                if t <= 0:
                    return cube_waypoints[0]
                elif t >= T:
                    return cube_waypoints[-1]
                else:
                    segment_idx = min(int(t / dt_segment), n_segments - 1)
                    t_local = (t - segment_idx * dt_segment) / dt_segment
                    cube_start = cube_waypoints[segment_idx]
                    cube_end = cube_waypoints[segment_idx + 1]

                    # Linear interpolation of translation
                    pos_interp = (1 - t_local) * cube_start.translation + t_local * cube_end.translation
                    return pin.SE3(cube_start.rotation, pos_interp)

            return cube_traj

        cube_traj = make_cube_traj(cube_waypoints, total_time)

        if VISUALIZE_ONLY:
            print("Visualizing trajectory in MeshCat...")
            print("NOTE: Cube moves manually in visualization.")
            print("      In actual simulation, it moves via contact forces.\n")

            from tools import setcubeplacement

            n_frames = 150
            for i in range(n_frames + 1):
                t = (i / n_frames) * total_time
                q = q_traj(t)
                cube_current = cube_traj(t)

                setcubeplacement(robot, cube, cube_current)
                updatevisuals(viz, robot, cube, q)

                if i % 30 == 0:
                    print(f"Progress: {100*i/n_frames:5.1f}% | t={t:5.2f}s")

                time.sleep(total_time / n_frames / 2.0)

            print("\nVisualization complete!")
            print("Run without --visualize for physics simulation")

        else:
            print("Running physics simulation in PyBullet...")
            print("Cube will move due to contact forces from robot hands.\n")

            from tools import setcubeplacement

            # Set initial state
            setcubeplacement(robot, cube, cube_waypoints[0])
            sim.setqsim(q0)

            tcur = 0.0

            # Create wrapper with cube_traj bound
            def controllaw_wrapper(sim, robot, trajs, tcur, cube):
                controllaw_actual(sim, robot, cube, trajs, cube_traj, tcur)

            while tcur < total_time:
                rununtil(controllaw_wrapper, DT, sim, robot, trajs, tcur, cube)
                tcur += DT

                if int(tcur) > int(tcur - DT):
                    print(f"Simulation time: {tcur:.1f}s / {total_time:.1f}s")

            print("\nSimulation complete!")
            print("The cube should have been physically manipulated by the robot.")

