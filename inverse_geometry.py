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


# ARIAN - FIXED VERSION
def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)

    oMleftgoal = getcubeplacement(cube, LEFT_HOOK)
    oMrightgoal = getcubeplacement(cube, RIGHT_HOOK)

    IDX_RIGHT = robot.model.getFrameId(RIGHT_HAND)
    IDX_LEFT = robot.model.getFrameId(LEFT_HAND)

    DT = 1e-2

    for i in range(MAX_ITERATIONS):
        # Compute current end-effector placements
        pin.framesForwardKinematics(robot.model, robot.data, qcurrent)
        pin.computeJointJacobians(robot.model, robot.data, qcurrent)

        oMleft = robot.data.oMf[IDX_LEFT]
        oMright = robot.data.oMf[IDX_RIGHT]

        error_left = -pin.log(oMleft.inverse() * oMleftgoal).vector
        error_right = -pin.log(oMright.inverse() * oMrightgoal).vector

        # checking early convergence
        if norm(error_left) < EPSILON and norm(error_right) < EPSILON:
            if not collision(robot, qcurrent) and not jointlimitsviolated(robot, qcurrent):
                return qcurrent, True
            else:
                # Converged but in collision or violates limits
                return qcurrent, False

        # Compute Jacobians
        o_Jright = pin.computeFrameJacobian(robot.model, robot.data, qcurrent, IDX_RIGHT)
        o_Jleft = pin.computeFrameJacobian(robot.model, robot.data, qcurrent, IDX_LEFT)

        # Null-space control: right hand primary, left hand secondary
        vq = -pinv(o_Jright) @ error_right
        P = np.eye(robot.nv) - pinv(o_Jright) @ o_Jright
        vq += pinv(o_Jleft @ P) @ (-error_left - o_Jleft @ vq)

        # Integrate
        qcurrent = pin.integrate(robot.model, qcurrent, DT*vq)

        # *Project to joint limits during IK
        qcurrent = projecttojointlimits(robot, qcurrent)

    
    return qcurrent, False



# SHREEYA

# def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
#       '''Return a collision free configuration grasping a cube at a specific location'''
#       # Set cube to target placement
#       setcubeplacement(robot, cube, cubetarget)

#       # Get goal placements for both hands
#       oMleftgoal = getcubeplacement(cube, LEFT_HOOK)
#       oMrightgoal = getcubeplacement(cube, RIGHT_HOOK)

#       # Get frame indices
#       IDX_RIGHT = robot.model.getFrameId(RIGHT_HAND)
#       IDX_LEFT = robot.model.getFrameId(LEFT_HAND)

#       # Control parameters
#       DT = 1e-2
#       lambda_gain = 1.0  # Can tune for faster convergence
#       lambda_postural = 0.1  # Postural task weight

#       # Postural reference (comfortable home position)
#       q_postural = robot.q0.copy()

#       for iteration in range(MAX_ITERATIONS):
#           # === Forward kinematics ===
#           pin.framesForwardKinematics(robot.model, robot.data, qcurrent)
#           pin.computeJointJacobians(robot.model, robot.data, qcurrent)

#           # === Get current hand placements ===
#           oMleft = robot.data.oMf[IDX_LEFT]
#           oMright = robot.data.oMf[IDX_RIGHT]

#           # === Compute 6D errors in LOCAL frames ===
#           # Error from current right hand to goal (in right hand frame)
#           rightMrightgoal = oMright.inverse() * oMrightgoal
#           error_right = pin.log(rightMrightgoal).vector

#           # Error from current left hand to goal (in left hand frame)
#           leftMleftgoal = oMleft.inverse() * oMleftgoal
#           error_left = pin.log(leftMleftgoal).vector

#           # === Check convergence ===
#           if norm(error_right) < EPSILON and norm(error_left) < EPSILON:
#               # Success! Check collision and joint limits
#               if not collision(robot, qcurrent) and not jointlimitsviolated(robot, qcurrent):
#                   if viz: viz.display(qcurrent)
#                   return qcurrent, True
#               else:
#                   # Converged but in collision/violates limits
#                   break

#           # === Compute Jacobians in LOCAL frames ===
#           right_Jright = pin.computeFrameJacobian(robot.model, robot.data, qcurrent,
#                                                   IDX_RIGHT, pin.LOCAL)
#           left_Jleft = pin.computeFrameJacobian(robot.model, robot.data, qcurrent,
#                                                 IDX_LEFT, pin.LOCAL)

#           # === Task 1: Right hand placement (PRIMARY) ===
#           vq = pinv(right_Jright) @ error_right

#           # === Task 2: Left hand placement (SECONDARY - in null space of Task 1) ===
#           P1 = np.eye(robot.nv) - pinv(right_Jright) @ right_Jright
#           vq += pinv(left_Jleft @ P1) @ (error_left - left_Jleft @ vq)

#           # === Task 3: Postural task (TERTIARY - in null space of Tasks 1 & 2) ===
#           P2 = P1 - pinv(left_Jleft @ P1) @ (left_Jleft @ P1)
#           postural_error = q_postural - qcurrent
#           vq += lambda_postural * P2 @ postural_error

#           # === Integrate with gain ===
#           qcurrent = pin.integrate(robot.model, qcurrent, lambda_gain * DT * vq)

#           # === Project to joint limits ===
#           qcurrent = projecttojointlimits(robot, qcurrent)

#           # === Visualize ===
#           if viz and iteration % 10 == 0:  # Visualize every 10 iterations
#               viz.display(qcurrent)

#       # Failed to converge
#       return qcurrent, False



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
    
    
    
