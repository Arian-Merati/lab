#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np

from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 300.               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    #TODO
     
    torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
    sim.step(torques)

if __name__ == "__main__":
    import sys
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT

    # ========================================================================
    # VISUALIZATION MODE vs CONTROL MODE
    # Run with: python control.py --visualize (to test trajectory in MeshCat)
    # Run with: python control.py (for PyBullet control simulation)
    # ========================================================================
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
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)

    if not (successinit and successend):
        print("ERROR: Could not compute valid grasping configurations!")
        exit(1)

    path, pathsuccess = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    print(f"Path computed: {len(path)} waypoints, success={pathsuccess}")


    #setting initial configuration
    if not VISUALIZE_ONLY:
        sim.setqsim(q0)
    else:
        updatevisuals(viz, robot, cube, q0)
        time.sleep(1.0)

    # ========================================================================
    # PART 2 TASK I: Trajectory Generation - Following the Path
    # ========================================================================

    def maketraj(path, T, method='level0_smooth'):
        """
        Create trajectory from path waypoints.

        Args:
            path: list of configurations from computepath() [q0, q1, ..., qn]
            T: total time duration (seconds)
            method: 'level0_simple' or 'level0_smooth' (recommended)

        Returns:
            q_of_t: position trajectory function
            vq_of_t: velocity trajectory function
            vvq_of_t: acceleration trajectory function
        """
        M = len(path)
        print(f"\nCreating trajectory from {M} waypoints using method: {method}")

        if method == 'level0_simple':
            # Simple version: zero velocities at each waypoint
            n_segments = M - 1
            dt = T / n_segments
            segments = []

            for i in range(n_segments):
                q_start = path[i]
                q_end = path[i + 1]
                t_start = i * dt
                t_end = (i + 1) * dt

                # Cubic Bezier with zero velocities at endpoints
                control_points = [q_start, q_start, q_end, q_end]
                bezier = Bezier(control_points, t_min=t_start, t_max=t_end)
                segments.append(bezier)

            # Create trajectory functions
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
            # Smooth version: continuous velocities at waypoints
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

            # Create trajectory functions
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

        # Validate trajectory
        print(f"Trajectory created: T={T:.2f}s")
        print(f"  Start config error: {np.linalg.norm(q_of_t(0.0) - path[0]):.6f}")
        print(f"  End config error: {np.linalg.norm(q_of_t(T) - path[-1]):.6f}")
        print(f"  Initial velocity: {np.linalg.norm(vq_of_t(0.0)):.6f}")
        print(f"  Final velocity: {np.linalg.norm(vq_of_t(T)):.6f}")

        return q_of_t, vq_of_t, vvq_of_t


    # Create trajectory from the full path (not just q0 and qe!)
    total_time = 10.0  # seconds - adjust as needed
    trajs = maketraj(path, total_time, method='level0_smooth')
    q_traj, v_traj, a_traj = trajs

    if VISUALIZE_ONLY:
        # ====================================================================
        # VISUALIZATION MODE: Show trajectory in MeshCat
        # ====================================================================
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
            updatevisuals(viz, robot, cube, q)

            if i % 30 == 0:
                progress = (i / n_frames) * 100
                print(f"Progress: {progress:5.1f}% | t={t:5.2f}s/{total_time:.2f}s")

            time.sleep(dt_display)

        print("\n✓ Trajectory visualization complete!")
        print("If trajectory looks good, run without --visualize for control simulation")

    else:
        # ====================================================================
        # CONTROL MODE: Run dynamics simulation
        # ====================================================================
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

        print("\n✓ Simulation complete!")
