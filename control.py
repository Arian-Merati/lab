# 
    
    
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import pinocchio as pin

import scipy.interpolate as spi
from scipy.interpolate import CubicSpline
import pybullet as p #NEW IMPORT

from bezier import Bezier
from config import DT, LEFT_HAND, RIGHT_HAND, LEFT_HOOK, RIGHT_HOOK
from tools import getcubeplacement, setcubeplacement, collision
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 4000               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)     # derivative gain (D of PD)
Ki = 200                 # integral gain (I of PID)
# integral_error = None  
fc = 400               # contact force

OBSTACLE_AVOIDANCE_EPSILON = 0.05  # Distance threshold for obstacle avoidance

def getCubePose(sim):
    pos, orn = p.getBasePositionAndOrientation(
            sim.cubeId,  # The PyBullet ID for the cube
            physicsClientId=sim.physicsClient # The sim's client ID
        )
        
    # Convert this to a pin.SE3 object
    oMf_cube = pin.SE3(pin.Quaternion(orn[0], orn[1], orn[2], orn[3]), 
                        np.array(pos))
    
    return oMf_cube

def controllaw(sim, robot, trajs, tcurrent, cube):
    # global integral_error
    # if integral_error is None:
    #     integral_error = np.zeros_like(robot.q0)

    q, vq = sim.getpybulletstate()
    
    IDX_RIGHT = robot.model.getFrameId(RIGHT_HAND)
    IDX_LEFT = robot.model.getFrameId(LEFT_HAND)

    oMf_cube = getCubePose(sim) 

    setcubeplacement(robot, cube, oMf_cube)

    pin.forwardKinematics(robot.model, robot.data, q, vq)
    pin.computeJointJacobians(robot.model, robot.data, q)
    
    oMleft = robot.data.oMf[IDX_LEFT]
    oMright = robot.data.oMf[IDX_RIGHT]
    
    
    #########################################
    obs_geom_id = robot.collision_model.getGeometryId('obstaclebase_0')
    obs_geom = robot.collision_model.geometryObjects[obs_geom_id]
    oMf_obs = obs_geom.placement # Obstacle's pose
    
    # 2. Get obstacle position and half-dimensions
    obs_pos = oMf_obs.translation
    obs_half_sides = obs_geom.geometry.halfSide # [hx, hy, hz]
    
    # 3. Get hand positions (you already have these)
    left_hand_pos = oMleft.translation
    right_hand_pos = oMright.translation

    # 4. Calculate distance for LEFT hand
    # Find the closest point on the obstacle box to the hand
    closest_point_on_box_L = np.zeros(3)
    closest_point_on_box_L[0] = max(obs_pos[0] - obs_half_sides[0], min(left_hand_pos[0], obs_pos[0] + obs_half_sides[0]))
    closest_point_on_box_L[1] = max(obs_pos[1] - obs_half_sides[1], min(left_hand_pos[1], obs_pos[1] + obs_half_sides[1]))
    closest_point_on_box_L[2] = max(obs_pos[2] - obs_half_sides[2], min(left_hand_pos[2], obs_pos[2] + obs_half_sides[2]))
    
    # Calculate the distance between the hand and that closest point
    dist_left_hand_to_obs = np.linalg.norm(left_hand_pos - closest_point_on_box_L)

    # 5. Calculate distance for RIGHT hand
    closest_point_on_box_R = np.zeros(3)
    closest_point_on_box_R[0] = max(obs_pos[0] - obs_half_sides[0], min(right_hand_pos[0], obs_pos[0] + obs_half_sides[0]))
    closest_point_on_box_R[1] = max(obs_pos[1] - obs_half_sides[1], min(right_hand_pos[1], obs_pos[1] + obs_half_sides[1]))
    closest_point_on_box_R[2] = max(obs_pos[2] - obs_half_sides[2], min(right_hand_pos[2], obs_pos[2] + obs_half_sides[2]))
    
    dist_right_hand_to_obs = np.linalg.norm(right_hand_pos - closest_point_on_box_R)

    q_curr = trajs[0](tcurrent)
    v_curr = trajs[1](tcurrent)
    a_curr = trajs[2](tcurrent)

    q_err = q- q_curr
    v_err = vq - v_curr
    # integral_error += q_err * DT
    
    a_total = a_curr + (-Kp * q_err) + (-Kv * v_err)
    
    data = pin.Data(robot.model)
    #tau_motion = pin.rnea(robot.model, robot.data, q_curr, v_curr, a_total) + robot.data.nle
    tau_motion = pin.rnea(robot.model, data, q_curr, v_curr, a_total) + robot.data.nle

    # data = pin.Data(robot.model)
    # tau_motion = pin.crba(robot.model, data, q_curr) @ a_total  + robot.data.nle # Store the motion torque

    
    tau_force = np.zeros_like(robot.q0)
    
    # --- Left Hand Force ---
    
    try:
        cube_mass = cube.model.inertias[0].mass
    except IndexError:
        print("Warning: Could not get cube mass. Assuming 0.")
        cube_mass = 0.0
        
    # Get the gravity vector from the robot's (world) model
    g_vector = robot.model.gravity.linear   # This is typically [0, 0, -9.81]
    
    # Calculate the upward force vector needed to counteract gravity
    force_antigravity = -cube_mass * g_vector
 
    HandLid = robot.model.getFrameId(LEFT_HAND)

    oMHandL = robot.data.oMf[HandLid]

    oMcubeL = getcubeplacement(cube, LEFT_HOOK)

    err_pos_L = oMcubeL.translation - oMHandL.translation
  
    force_L = fc * err_pos_L

    force_L_total = force_L + 1 * force_antigravity
    
    dist_L_x = left_hand_pos[0] - closest_point_on_box_L[0]
    if abs(dist_L_x) < OBSTACLE_AVOIDANCE_EPSILON:
        # If force and distance vector have opposite signs, force points towards obstacle
        if (dist_L_x * force_L_total[0]) < 0: 
            force_L_total[0] = 0.0 # Zero out X-force component

    # Check Z-direction (Up/Down)
    dist_L_z = left_hand_pos[2] - closest_point_on_box_L[2]
    if abs(dist_L_z) < OBSTACLE_AVOIDANCE_EPSILON:
        # If force and distance vector have opposite signs, force points towards obstacle
        if (dist_L_z * force_L_total[2]) < 0:
            force_L_total[2] = 0.0 # Zero out Z-force component

    wrench_L = pin.Motion(force_L_total, np.zeros(3))

    J_L = pin.computeFrameJacobian(robot.model, robot.data, q, HandLid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    # Map force to torques (tau = J^T * F)
    tau_force += J_L.T @ wrench_L.vector

    # --- Right Hand Force ---
 
    HandRid = robot.model.getFrameId(RIGHT_HAND)

    oMHandR = robot.data.oMf[HandRid]
  
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)

    err_pos_R = oMcubeR.translation - oMHandR.translation

    force_R = fc * err_pos_R

    force_R_total = force_R + 1 * force_antigravity
    
    dist_R_x = right_hand_pos[0] - closest_point_on_box_R[0]
    if abs(dist_R_x) < OBSTACLE_AVOIDANCE_EPSILON:
        if (dist_R_x * force_R_total[0]) < 0: 
            force_R_total[0] = 0.0 # Zero out X-force component

    # Check Z-direction (Up/Down)
    dist_R_z = right_hand_pos[2] - closest_point_on_box_R[2]
    if abs(dist_R_z) < OBSTACLE_AVOIDANCE_EPSILON:
        if (dist_R_z * force_R_total[2]) < 0:
            force_R_total[2] = 0.0 # Zero out Z-force component

    wrench_R = pin.Motion(force_R_total, np.zeros(3))
    # Get Jacobian
    J_R = pin.computeFrameJacobian(robot.model, robot.data, q, HandRid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    # Map force to torques
    tau_force += J_R.T @ wrench_R.vector


    # 5. SEND FINAL COMBINED TORQUE
    tau_final = tau_motion + tau_force  #+ robot.data.nle
    
    sim.step(tau_final)


    # def maketraj(path:list, T:float):
    #     path = [path[0].copy()] + [path[0].copy()] + [path[0].copy()] + path + [path[-1].copy()] + [path[-1].copy()] + [path[-1].copy()]
    #     new_path = np.array(path)
    #     Bezier_traj = Bezier(new_path, t_min=0.0, t_max=T)
    #     vq_traj = Bezier_traj.derivative(order=1)
    #     aq_traj = Bezier_traj.derivative(order=2)
    #     return Bezier_traj, vq_traj, aq_traj


# def maketraj(path:list, T:float):
#     """Create piecewise Bezier trajectory through all waypoints"""
#     path = [path[0].copy()] + [path[0].copy()]+ path \
#            + [path[-1].copy()] + [path[-1].copy()]
#     n_segments = len(path) - 1
#     T_segment = T / n_segments

#     # ((t_start, t_end), (q_segment, vq_segment, aq_segment))
#     trajectories:list[tuple[tuple, tuple]] = []
#     for i in range(n_segments):
#         segment_points = [path[i], path[i], path[i+1], path[i+1]]
        
#         t_start = i * T_segment
#         t_end = (i + 1) * T_segment
        
#         # time should not be constant for each segment but instead proportional to the ditsnace
        
#         q_segment = Bezier(segment_points, t_min=t_start, t_max=t_end)
#         vq_segment = q_segment.derivative(order=1)
#         aq_segment = q_segment.derivative(order=2)

#         trajectories.append(((t_start, t_end), (q_segment, vq_segment, aq_segment)))

#     # ((t_start, t_end), (q_segment, vq_segment, aq_segment))

#     def q_of_t(t):
#         for (t_start, t_end), (q_segment, _, _) in trajectories:
#             if (t >= t_start) and (t <= t_end):
#                 return q_segment(t)

#     def vq_of_t(t):
#         for (t_start, t_end), (_, vq_segment, _) in trajectories:
#             if (t >= t_start) and (t <= t_end):
#                 return vq_segment(t)

#     def aq_of_t(t):
#         for (t_start, t_end), (_, _, aq_segment) in trajectories:
#             if (t >= t_start) and (t <= t_end):
#                 return aq_segment(t)
    
#     return q_of_t, vq_of_t, aq_of_t


import numpy as np # Make sure numpy is imported
from bezier import Bezier # Already in your imports

def maketraj(path:list, T:float):
    """Create piecewise Bezier trajectory through all waypoints,
       with segment time proportional to C-space distance."""
    
    # 1. Pad the path
    path = [path[0].copy()] + [path[0].copy()]+ path \
           + [path[-1].copy()] + [path[-1].copy()]
    n_segments = len(path) - 1

    # 2. Calculate segment distances
    distances = []
    for i in range(n_segments):
        dist = np.linalg.norm(path[i+1] - path[i]) 
        distances.append(dist)

    # 3. Calculate total distance
    total_distance = sum(distances)

    # 4. Calculate time duration for each segment
    segment_durations = []
    if total_distance < 1e-9: 
        print("Warning: Total path distance is near zero. Assigning equal time segments (may fail).")
        segment_durations = [0.0] * n_segments
    else:
        for dist in distances:
            proportion = dist / total_distance
            T_segment_i = proportion * T 
            segment_durations.append(T_segment_i)

    # 5. Build trajectories using variable time segments
    trajectories:list[tuple[tuple, tuple]] = []
    t_current = 0.0 # This will be our running t_start

    for i in range(n_segments):
        T_segment_i = segment_durations[i]
        
        # Skip any segment that has zero (or near-zero) duration.
        # This handles the padded segments [p0, p0, p0] etc.
        if T_segment_i < 1e-9:
            continue

        segment_points = [path[i], path[i], path[i+1], path[i+1]]
        
        t_start = t_current
        t_end = t_current + T_segment_i
        
        t_current = t_end
        
        q_segment = Bezier(segment_points, t_min=t_start, t_max=t_end)
        vq_segment = q_segment.derivative(order=1)
        aq_segment = q_segment.derivative(order=2)

        trajectories.append(((t_start, t_end), (q_segment, vq_segment, aq_segment)))
        
    if not trajectories:
        print("Error: No valid path segments found (total distance is likely zero).")
        # Create a single, static trajectory to avoid crashing
        t_start, t_end = 0.0, T
        q_segment = Bezier([path[0], path[0], path[0], path[0]], t_min=t_start, t_max=t_end)
        vq_segment = q_segment.derivative(order=1)
        aq_segment = q_segment.derivative(order=2)
        trajectories.append(((t_start, t_end), (q_segment, vq_segment, aq_segment)))


    
    t_min_global = trajectories[0][0][0]
    t_max_global = trajectories[-1][0][1] # This should be equal to T

    def q_of_t(t):
        t = np.clip(t, t_min_global, t_max_global)

        if t == t_max_global:
            return trajectories[-1][1][0](t) # Use the last segment's function

        for (t_start, t_end), (q_segment, _, _) in trajectories:
            if t >= t_start and t < t_end:
                return q_segment(t)
        
        # Fallback (should only be t_min_global)
        return trajectories[0][1][0](t_min_global)


    def vq_of_t(t):
        t = np.clip(t, t_min_global, t_max_global)
        
        if t == t_max_global:
            return trajectories[-1][1][1](t)

        for (t_start, t_end), (_, vq_segment, _) in trajectories:
            if t >= t_start and t < t_end:
                return vq_segment(t)

        return trajectories[0][1][1](t_min_global)

    def aq_of_t(t):
        t = np.clip(t, t_min_global, t_max_global)

        if t == t_max_global:
            return trajectories[-1][1][2](t)

        for (t_start, t_end), (_, _, aq_segment) in trajectories:
            if t >= t_start and t < t_end:
                return aq_segment(t)
        
        return trajectories[0][1][2](t_min_global)
    
    return q_of_t, vq_of_t, aq_of_t


if __name__ == "__main__":   
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    
    robot, sim, cube = setupwithpybullet()
    TOTAL_TIME = 10.0  # total simulation time in seconds
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path, success = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    if (not successinit) or (not successend) or (not success):
        print("Could not compute a valid path")
        exit(0)

    #setting initial configuration
    sim.setqsim(path[0])

    trajs = maketraj(path, TOTAL_TIME)
    
    tcur = 0.0
    integral_error = np.zeros_like(robot.q0)
    while tcur < TOTAL_TIME:
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT