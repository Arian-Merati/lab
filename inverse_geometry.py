#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, jointlimitsviolated
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET

from tools import setcubeplacement

MAX_ITERATIONS = 1000


def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)
    #TODO implement
    oMleftgoal = getcubeplacement(cube, LEFT_HOOK)
    oMrightgoal = getcubeplacement(cube, RIGHT_HOOK)
    
    IDX_RIGHT = robot.model.getFrameId(RIGHT_HAND)
    IDX_LEFT = robot.model.getFrameId(LEFT_HAND)
    
    DT = 1e-2
    iteration = 0

    for i in range(MAX_ITERATIONS):
        #compute current end-effector placements
        
        pin.framesForwardKinematics(robot.model, robot.data, qcurrent)
        pin.computeJointJacobians(robot.model, robot.data, qcurrent)
        
        oMleft = robot.data.oMf[IDX_LEFT]
        oMright = robot.data.oMf[IDX_RIGHT]

        error_left = -pin.log(oMleft.inverse() * oMleftgoal).vector
        error_right = -pin.log(oMright.inverse() * oMrightgoal).vector

        o_Jright = pin.computeFrameJacobian(robot.model, robot.data, qcurrent, IDX_RIGHT) ## In terms of the world instead
        o_Jleft = pin.computeFrameJacobian(robot.model, robot.data, qcurrent, IDX_LEFT) ## In terms of the world instead
        
        vq = -pinv(o_Jright) @ error_right
        P = np.eye(robot.nv)-pinv(o_Jright) @ o_Jright
        vq += pinv(o_Jleft @ P) @ ( -error_left - o_Jleft @ vq)
        

        # vq_right = pinv(o_Jright)@error_right
        # vq_left = pinv(o_Jleft)@error_left
        # print(vq_right)

        # vq = vq_right + vq_left   ####### I think the error is here - how do we update each arm separately?

        qcurrent = pin.integrate(robot.model, qcurrent, DT*vq)
        
        #qcurrent = projecttojointlimits(robot, qcurrent)
        #viz.display(qcurrent)
        
        # pin.framesForwardKinematics(robot.model, robot.data, qcurrent)
        # pin.computeJointJacobians(robot.model, robot.data, qcurrent)

        # oMleft = robot.data.oMf[IDX_LEFT]
        # oMright = robot.data.oMf[IDX_RIGHT]

        # error_left = pin.log(oMleft.inverse() * oMleftgoal).vector
        # error_right = pin.log(oMright.inverse() * oMrightgoal).vector
        
        if (norm(error_left) < EPSILON and norm(error_right) < EPSILON):
            if not collision(robot, qcurrent) and not jointlimitsviolated(robot, qcurrent):
                return qcurrent, True
            else:
                return qcurrent, False
    return qcurrent, False

def cost(q):
    M = robot.placement(q, 6)
    return norm(pin.log(M.inverse()))
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, q0)
    
    
    
