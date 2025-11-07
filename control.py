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
Ki = 50.

# def controllaw(sim, robot, trajs, tcurrent, cube):
#     q, vq = sim.getpybulletstate()
#     #TODO - inverse dynamics controller
#     torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
#     sim.step(torques)


def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    
    # q_curr = trajs[0](tcurrent)
    # v_curr = trajs[1](tcurrent)
    # a_curr = trajs[2](tcurrent)

    # q_err = q_curr - q
    # v_err = v_curr - vq

    # pin.rnea(robot.model, robot.data, q, vq, a_curr)
    
    # torque = (Kp * q_err) + (Kv * v_err) + robot.data.nle

    # sim.step(torque)
    q_curr = trajs[0](tcurrent)
    v_curr = trajs[1](tcurrent)
    a_curr = trajs[2](tcurrent)

    q_err = q_curr - q
    v_err = v_curr - vq
    
    acceleration = a_curr + (Kp * q_err) + (Kv * v_err) + (Ki * robot.i_err)

    torque = pin.rnea(robot.model, robot.data, q, vq, acceleration)

    sim.step(torque)

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
    #setting initial configuration
    sim.setqsim(q0)
    
    robot.i_err = np.zeros(robot.model.nv) # Added this line
    
    
    #TODO this is just an example, you are free to do as you please.
    #In any case this trajectory does not follow the path 
    #0 init and end velocities
    # def maketraj(q0,q1,T): #TODO compute a real trajectory !
    #     q_of_t = Bezier([q0,q0,q1,q1],t_max=T)
    #     vq_of_t = q_of_t.derivative(1)
    #     vvq_of_t = vq_of_t.derivative(1)
    #     return q_of_t, vq_of_t, vvq_of_t
    
    def maketraj(path, T):
        q_of_t = Bezier(path, t_min=0., t_max=T)
        vq_of_t = q_of_t.derivative(1)
        aq_of_t = vq_of_t.derivative(1)

        return q_of_t, vq_of_t, aq_of_t
    
    
    #TODO this is just a random trajectory, you need to do this yourself
    total_time=10.0
    chunk_size = 4
    path = path[0]
    path = [path[0].copy()] + [path[0].copy()] + path + [path[-1].copy()] + [path[-1].copy()]
    trajs = []
    
    chunked_path = [path[i:i + chunk_size] for i in range(0, len(path), chunk_size)]
    time = total_time / len(chunked_path)
    for path_segment in chunked_path:
        trajs.append(maketraj(path_segment, time))
    #trajs = maketraj(q0, qe, total_time)   
    # trajs = maketraj(path, total_time)
    
    tcur = 0.
    
    
    # while tcur < total_time:
    #     rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
    #     tcur += DT
    

    for traj in trajs:
        tcur = 0.
        while tcur < time:
            rununtil(controllaw, DT, sim, robot, traj, tcur, cube)
            tcur += DT
    
    
    