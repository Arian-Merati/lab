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
Ki = 100.0 ##################

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    
    q_curr = trajs[0](tcurrent)
    v_curr = trajs[1](tcurrent)
    a_curr = trajs[2](tcurrent)

    q_err = q_curr - q
    v_err = v_curr - vq
    
    robot.i_err += q_err * DT##

    acceleration = a_curr + (Kp * q_err) + (Kv * v_err) + (Ki * robot.i_err) ##########'#

    torque = pin.rnea(robot.model, robot.data, q, vq, acceleration)

    sim.step(torque)
                      
# def maketraj(path, T):

#     new_path = np.array([path[0].copy()] + [path[0].copy()] + path + [path[-1].copy()] + [path[-1].copy()])
#     q_of_t = Bezier(new_path, t_max=T)
#     vq_of_t = q_of_t.derivative(1)
#     aq_of_t = vq_of_t.derivative(1)

#     return q_of_t, vq_of_t, aq_of_t

def maketraj(path:list, T:float):
    """Create piecewise Bezier trajectory through all waypoints"""
    path = [path[0].copy()] + [path[0].copy()] + [path[0].copy()] + path + [path[-1].copy()] + [path[-1].copy()] + [path[-1].copy()]
    n_segments = len(path) - 1
    T_segment = T / n_segments

    # ((t_start, t_end), (q_segment, vq_segment, aq_segment))
    trajectories:list[tuple[tuple, tuple]] = []
    for i in range(n_segments):
        segment_points = [path[i], path[i], path[i+1], path[i+1]]
        
        t_start = i * T_segment
        t_end = (i + 1) * T_segment
        
        q_segment = Bezier(segment_points, t_min=t_start, t_max=t_end)
        vq_segment = q_segment.derivative(order=1)
        aq_segment = q_segment.derivative(order=2)

        trajectories.append(((t_start, t_end), (q_segment, vq_segment, aq_segment)))

    # ((t_start, t_end), (q_segment, vq_segment, aq_segment))

    def q_of_t(t):
        for (t_start, t_end), (q_segment, _, _) in trajectories:
            if (t >= t_start) and (t <= t_end):
                return q_segment(t)

    def vq_of_t(t):
        for (t_start, t_end), (_, vq_segment, _) in trajectories:
            if (t >= t_start) and (t <= t_end):
                return vq_segment(t)

    def aq_of_t(t):
        for (t_start, t_end), (_, _, aq_segment) in trajectories:
            if (t >= t_start) and (t <= t_end):
                return aq_segment(t)
    
    return q_of_t, vq_of_t, aq_of_t


if __name__ == "__main__":   
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    TOTAL_TIME = 4.0  # total simulation time in seconds
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path, success = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    robot.i_err = np.zeros(robot.model.nv)
    if (not successinit) or (not successend) or (not success):
        print("Could not compute a valid path")
        exit(0)

    #setting initial configuration
    sim.setqsim(path[0])

    #TODO this is just a random trajectory, you need to do this yourself
    trajs = maketraj(path, TOTAL_TIME)
    
    tcur = 0.0
    while tcur < TOTAL_TIME:
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
    
    
    