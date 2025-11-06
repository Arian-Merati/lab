#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import pinocchio as pin

from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 300.               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    """
    Control law with inverse dynamics + force control for grasping.

    This implements Part 2, Task II:
    - Inverse dynamics control to track trajectory
    - Force control to hold the cube with both hands
    """
    import pybullet as pyb
    from config import LEFT_HAND, RIGHT_HAND, LEFT_HOOK, RIGHT_HOOK
    from tools import getcubeplacement, setcubeplacement

    try:
        # Get current robot state from PyBullet
        q, vq = sim.getpybulletstate()

        # Get desired state from trajectory
        q_traj, v_traj, a_traj = trajs
        q_des = q_traj(tcurrent)
        vq_des = v_traj(tcurrent)
        vvq_des = a_traj(tcurrent)

        # === PART 1: Inverse Dynamics Control (Trajectory Tracking) ===

        # Compute tracking errors
        error_q = q_des - q
        error_vq = vq_des - vq

        # Update robot kinematics and dynamics
        pin.computeAllTerms(robot.model, robot.data, q, vq)

        # Get mass matrix M (using CRBA - Composite Rigid Body Algorithm)
        M = pin.crba(robot.model, robot.data, q)

        # Get nonlinear effects: Coriolis + gravity (nle = non-linear effects)
        nle = pin.nle(robot.model, robot.data, q, vq)

        # Inverse dynamics control law: tau = M * (a_des + Kp*e_q + Kv*e_vq) + nle
        tau_tracking = M @ (vvq_des + Kp * error_q + Kv * error_vq) + nle

        # === PART 2: Force Control (Grasping) ===

        # Get cube state from PyBullet (actual position, not desired)
        cube_pos_pybullet, cube_orn_pybullet = pyb.getBasePositionAndOrientation(sim.cubeId)

        # Convert PyBullet quaternion [x,y,z,w] to Pinocchio SE3
        cube_placement_actual = pin.XYZQUATToSE3(np.array([
            cube_pos_pybullet[0], cube_pos_pybullet[1], cube_pos_pybullet[2],
            cube_orn_pybullet[0], cube_orn_pybullet[1], cube_orn_pybullet[2], cube_orn_pybullet[3]
        ]))

        # Update forward kinematics to get end-effector positions
        pin.framesForwardKinematics(robot.model, robot.data, q)

        # Get end-effector frame IDs
        left_hand_id = robot.model.getFrameId(LEFT_HAND)
        right_hand_id = robot.model.getFrameId(RIGHT_HAND)

        # Get current end-effector placements
        oMleft = robot.data.oMf[left_hand_id]
        oMright = robot.data.oMf[right_hand_id]

        # Get desired hook positions on the cube
        # Update cube geometry with actual placement
        setcubeplacement(robot, cube, cube_placement_actual)

        # Get hook positions in world frame
        oMleft_hook = getcubeplacement(cube, LEFT_HOOK)
        oMright_hook = getcubeplacement(cube, RIGHT_HOOK)

        # Compute attraction forces (spring-damper law)
        # Force = Kf * (p_desired - p_current)
        Kf = 100.0  # Force gain (tune this)

        # Position errors for each hand
        left_error = oMleft_hook.translation - oMleft.translation
        right_error = oMright_hook.translation - oMright.translation

        # Force vectors (only linear forces, no torques)
        F_left = Kf * left_error
        F_right = Kf * right_error

        # Get Jacobians for end-effectors (linear part only)
        J_left = pin.computeFrameJacobian(robot.model, robot.data, q, left_hand_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        J_right = pin.computeFrameJacobian(robot.model, robot.data, q, right_hand_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]

        # Convert forces to joint torques using Jacobian transpose
        tau_force_left = J_left.T @ F_left
        tau_force_right = J_right.T @ F_right

        # === COMBINE CONTROL TORQUES ===
        tau_total = tau_tracking + tau_force_left + tau_force_right

        # Convert to list for PyBullet
        torques = tau_total.tolist()

    except Exception as e:
        print(f"ERROR in controllaw at t={tcurrent:.3f}: {e}")
        import traceback
        traceback.print_exc()
        torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]

    # Apply torques and step simulation
    sim.step(torques)

if __name__ == "__main__":
    import sys
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT

    # Run with: python control.py --visualize (to test trajectory in MeshCat)
    # Run with: python control.py (for PyBullet control simulation)
    
    VISUALIZE_ONLY = "--visualize" in sys.argv or "-v" in sys.argv

    if VISUALIZE_ONLY:
        print("\n" + "="*70)
        print("VISUALIZATION MODE: Testing trajectory in MeshCat")
        print("="*70)
        print("Make sure meshcat-server is running!")
        print("Browser: http://127.0.0.1:7000/static/")
        print("="*70 + "\n")
        from tools import setupwithmeshcat
        from setup_meshcat import updatevisuals
        import time
        robot, cube, viz = setupwithmeshcat()
        sim = None
    else:
        print("\n" + "="*70)
        print("CONTROL MODE: Running dynamics simulation in PyBullet")
        print("="*70 + "\n")
        robot, sim, cube = setupwithpybullet()
        viz = None
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    import path

    # Set global variables in path module (needed for computepath)
    path.robot = robot
    path.cube = cube
    path.viz = viz

    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  viz)

    if not (successinit and successend):
        print("ERROR: Could not compute valid grasping configurations!")
        exit(1)

    robot_path, pathsuccess = path.computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    print(f"Path computed: {len(robot_path)} waypoints, success={pathsuccess}")

    # Unpack path into separate lists for robot configs and cube placements
    # (path.computepath returns list of (q_config, cube_placement) tuples)
    from tools import setcubeplacement
    q_waypoints = [q for q, _ in robot_path]
    cube_waypoints = [cube_placement for _, cube_placement in robot_path]

    # Reset cube to initial position after path computation
    setcubeplacement(robot, cube, cube_waypoints[0])

    #setting initial configuration
    if not VISUALIZE_ONLY:
        sim.setqsim(q0)
    else:
        # Update visualization with cube at initial position
        updatevisuals(viz, robot, cube, q0)
        time.sleep(1.0)

    
    # PART 2 TASK I: Trajectory Generation - Following the Path
    

    def maketraj(path, T, method='level0_smooth'):
        """
        Create trajectory from path waypoints.

        Args:
            path: list of configurations from computepath() [q0, q1, ..., qn]
            T: total time duration (seconds)

        Returns:
            q_of_t: position trajectory function
            vq_of_t: velocity trajectory function
            vvq_of_t: acceleration trajectory function
        """
        M = len(path)
        print(f"\nCreating trajectory from {M} waypoints using method: {method}")

        if method == 'level0_simple':
            #zero velocities at each waypoint
            n_segments = M - 1
            dt = T / n_segments
            segments = []

            for i in range(n_segments):
                q_start = path[i]
                q_end = path[i + 1]
                t_start = i * dt
                t_end = (i + 1) * dt

                #Cubic Bezier with zero velocities at endpoints
                control_points = [q_start, q_start, q_end, q_end]
                bezier = Bezier(control_points, t_min=t_start, t_max=t_end)
                segments.append(bezier)

            # trajectory functions
            def q_of_t(t):
                if t <= 0:
                    return path[0].copy()
                if t >= T:
                    return path[-1].copy()
                segment_idx = min(int(t / dt), n_segments - 1)
                return segments[segment_idx](t)

            def vq_of_t(t):
                if t <= 0 or t >= T:
                    return np.zeros_like(path[0])
                segment_idx = min(int(t / dt), n_segments - 1)
                return segments[segment_idx].derivative(1)(t)

            def vvq_of_t(t):
                if t <= 0 or t >= T:
                    return np.zeros_like(path[0])
                segment_idx = min(int(t / dt), n_segments - 1)
                return segments[segment_idx].derivative(2)(t)

        elif method == 'level0_smooth':
            # better verions - continuous velocities at waypoints
            n_segments = M - 1
            dt = T / n_segments

            # Compute velocities at waypoints using finite differences
            velocities = [np.zeros_like(path[0])]  # zero at start

            for i in range(1, M - 1):
                v = (path[i + 1] - path[i - 1]) / (2 * dt)
                velocities.append(v)

            velocities.append(np.zeros_like(path[-1]))  # zero at end

            # Create Hermite-Bezier segments
            segments = []
            for i in range(n_segments):
                q0 = path[i]
                q1 = path[i + 1]
                v0 = velocities[i]
                v1 = velocities[i + 1]
                t_start = i * dt
                t_end = (i + 1) * dt

                # Hermite to Bezier control points
                p0 = q0
                p1 = q0 + (dt / 3.0) * v0
                p2 = q1 - (dt / 3.0) * v1
                p3 = q1

                bezier = Bezier([p0, p1, p2, p3], t_min=t_start, t_max=t_end)
                segments.append(bezier)

            # trajectory functions
            def q_of_t(t):
                if t <= 0:
                    return path[0].copy()
                if t >= T:
                    return path[-1].copy()
                segment_idx = min(int(t / dt), n_segments - 1)
                return segments[segment_idx](t)

            def vq_of_t(t):
                if t <= 0 or t >= T:
                    return np.zeros_like(path[0])
                segment_idx = min(int(t / dt), n_segments - 1)
                return segments[segment_idx].derivative(1)(t)

            def vvq_of_t(t):
                if t <= 0 or t >= T:
                    return np.zeros_like(path[0])
                segment_idx = min(int(t / dt), n_segments - 1)
                return segments[segment_idx].derivative(2)(t)

        else:
            raise ValueError(f"Unknown method: {method}")

        #validating trajectory
        print(f"Trajectory created: T={T:.2f}s")
        print(f"  Start config error: {np.linalg.norm(q_of_t(0.0) - path[0]):.6f}")
        print(f"  End config error: {np.linalg.norm(q_of_t(T) - path[-1]):.6f}")
        print(f"  Initial velocity: {np.linalg.norm(vq_of_t(0.0)):.6f}")
        print(f"  Final velocity: {np.linalg.norm(vq_of_t(T)):.6f}")

        return q_of_t, vq_of_t, vvq_of_t


    # trajectory from full path (not just q0 and qe)
    total_time = 10.0  # seconds - adjust as needed
    trajs = maketraj(q_waypoints, total_time, method='level0_smooth')
    q_traj, v_traj, a_traj = trajs

    if VISUALIZE_ONLY:
        print("\n" + "="*70)
        print("Visualizing trajectory in MeshCat...")
        print("Open browser to: http://127.0.0.1:7000/static/")
        print("="*70 + "\n")

        input("Press ENTER to start animation...")

        n_frames = 150
        playback_speed = 2.0  # 2x faster
        dt_display = (total_time / n_frames) / playback_speed

        print(f"Animating {n_frames} frames at {playback_speed}x speed...\n")

        for i in range(n_frames + 1):
            t = (i / n_frames) * total_time
            q = q_traj(t)

            # Interpolate cube position from planned waypoints
            # This uses the collision-free cube trajectory from path planning
            M = len(cube_waypoints)
            n_segments = M - 1
            dt_segment = total_time / n_segments

            if t <= 0:
                cube_current = cube_waypoints[0]
            elif t >= total_time:
                cube_current = cube_waypoints[-1]
            else:
                segment_idx = min(int(t / dt_segment), n_segments - 1)
                # Linear interpolation (LERP) between waypoints
                t_local = (t - segment_idx * dt_segment) / dt_segment
                cube_start = cube_waypoints[segment_idx]
                cube_end = cube_waypoints[segment_idx + 1]

                # Interpolate translation
                pos_interp = (1 - t_local) * cube_start.translation + t_local * cube_end.translation
                # Keep rotation constant (cube doesn't rotate)
                cube_current = pin.SE3(cube_start.rotation, pos_interp)

            setcubeplacement(robot, cube, cube_current)

            # Update visualization
            updatevisuals(viz, robot, cube, q)

            if i % 30 == 0:
                progress = (i / n_frames) * 100
                print(f"Progress: {progress:5.1f}% | t={t:5.2f}s/{total_time:.2f}s")

            time.sleep(dt_display)

        print("\nTrajectory visualization complete!")
        print("If trajectory looks good, run without --visualize for control simulation")

    else:
        # dynamics simulation
        print("\n" + "="*70)
        print("Running control simulation...")
        print("This will take a while (real-time simulation)...")
        print("="*70 + "\n")

        tcur = 0.

        while tcur < total_time:
            rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
            tcur += DT

            # Print progress every second
            if int(tcur) > int(tcur - DT):
                print(f"Simulation time: {tcur:.1f}s / {total_time:.1f}s")

        print("\n Simulation complete!")



""" type in this command to see the output and the animation: "cd /home/infernox/aro/cw2/lab
   /home/infernox/aro/lab/venv/bin/python control.py --visualize" """