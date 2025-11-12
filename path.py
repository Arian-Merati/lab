#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv

from config import LEFT_HAND, RIGHT_HAND
import time
from tools import collision, jointlimitsviolated, setcubeplacement
from inverse_geometry import computeqgrasppose
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET # Added import statement
from tools import setupwithmeshcat
from inverse_geometry import computeqgrasppose

    
#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
    # def RAND_CONF_CUBE(cube):
    #     '''
    #     Return a random configuration for the cube
    #     '''
    #     while True:
    #         q = pin.randomConfiguration(robot.model)  # sample between -3.2 and +3.2.
    #         if (not collision(robot, q)) and (not jointlimitsviolated(robot, q)):
    #             return q
        
def RAND_CONF(robot, cube, viz, G):
    '''
    Return a random configuration for the cube
    '''
    nearest_to_target_idx = NEAREST_VERTEX(G, CUBE_PLACEMENT_TARGET.translation)
    base_placement = G[nearest_to_target_idx][1]
    # sample_range_lower = np.array([0.05, 0.05, 0]) 
    # sample_range_upper = np.array([0.15, 0.3, 0.3])

    sample_range_lower = np.array([0.12, 0.3, 0.05]) 
    sample_range_upper = np.array([0.12, 0.3, 0.3])
    lower_bound = base_placement - sample_range_lower
    upper_bound = base_placement + sample_range_upper

    random_position = np.random.uniform(lower_bound, upper_bound)
    cube_matrix = pin.SE3(CUBE_PLACEMENT.rotation, random_position)
   # setcubeplacement(robot, cube, cube_matrix)
    q, successFlag = computeqgrasppose(robot, robot.q0, cube, cube_matrix, viz)
    print("RAND CONF SUCCESS FLAG:", successFlag)
    if successFlag:
        return q, cube_matrix
    else:
        return RAND_CONF(robot, cube, viz, G)
    

def distance(q1,q2):    
    '''Return the euclidian distance between two configurations'''
    # print("DISTANCE CALLED")
    # print("Q1:", q1)
    # print("Q2:", q2)
    return np.linalg.norm(q2-q1)
        
def NEAREST_VERTEX(G,cube_rand):
    '''returns the index of the Node of G with the configuration closest to cube_rand  '''
    min_dist = 10e4
    idx=-1
    for (i,node) in enumerate(G):
        dist = distance(node[1],cube_rand) 
        if dist < min_dist:
            min_dist = dist
            idx = i
    return idx

def lerp(q0,q1,t):    
    return q0 * (1 - t) + q1 * t

def NEW_CONF(robot, cube, viz, cube_near,cube_rand,discretisationsteps, delta_q = None):
    '''Return the closest configuration q_new such that the path q_near => q_new is the longest
    along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q'''
    cube_end = cube_rand.copy()
    dist = distance(cube_near, cube_rand)
    if delta_q is not None and dist > delta_q:
        #compute the configuration that corresponds to a path of length delta_q
        cube_end = lerp(cube_near,cube_rand,delta_q/dist)
        # now dist == delta_q
    dt = 1 / discretisationsteps
    g_grasp = robot.q0.copy()
    for i in range(1,discretisationsteps):
        cube_interp = lerp(cube_near,cube_end,dt*i)
        cube_placement = pin.SE3(CUBE_PLACEMENT.rotation, cube_interp)
        q_grasp, successFlag = computeqgrasppose(robot, g_grasp, cube, cube_placement, viz)
        if not successFlag:
            return lerp(cube_near,cube_end,dt*(i-1))
    return cube_end

def ADD_EDGE_AND_VERTEX(G,parent,cubeplacement):
    G += [(parent,cubeplacement)]

def VALID_EDGE(robot, cube, viz,q_new, q_goal, discretisationsteps):
    return np.linalg.norm(q_goal -NEW_CONF(robot, cube, viz, q_new, q_goal,discretisationsteps)) < 1e-3

def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal):
    robot, cube, viz = setupwithmeshcat() # Added this line might change when debug
    discretisationsteps_newconf = 10 #To tweak later on
    discretisationsteps_validedge = 10 #To tweak later on
    k = 1000  #To tweak later on
    delta_q = 0.05 #To tweak later on
    cube_goal = cubeplacementqgoal.translation

    G = [(None, cubeplacementq0.translation)] 
    for _ in range(k):
        q_rand, sampled_cube = RAND_CONF(robot, cube, viz, G)   
        sampled_cube_position = sampled_cube.translation
        cube_near_index = NEAREST_VERTEX(G,sampled_cube_position)
        cube_near = G[cube_near_index][1]        
        cube_new = NEW_CONF(robot, cube, viz, cube_near, sampled_cube_position, discretisationsteps_newconf, delta_q = delta_q)    
        ADD_EDGE_AND_VERTEX(G,cube_near_index,cube_new)
        if VALID_EDGE(robot, cube, viz, cube_new,cube_goal,discretisationsteps_validedge):
            print ("Path found!")
            ADD_EDGE_AND_VERTEX(G,len(G)-1,cube_goal)
            path = getpath(robot, cube, viz, G)
            return path, True
        
    print("path not found")
    return getpath(robot, cube, viz, G), False

# def constructpath(G):
#     path = []
#     q_grasp = robot.q0.copy()
#     for i in range (len(G)):
#         cube_matrix = pin.SE3(CUBE_PLACEMENT.rotation, G[i][1])
#         q_grasp, successFlag = computeqgrasppose(robot, q_grasp, cube, cube_matrix, viz)
#         path += [q_grasp]
#     return path

def getpath(robot, cube, viz, G):
    cube_positions = []
    path = []
    node = G[-1]
    while node[0] is not None:
        cube_positions = [node[1]] + cube_positions
        node = G[node[0]]
    cube_positions = [G[0][1]] + cube_positions
    for position in cube_positions:
        cube_matrix = pin.SE3(CUBE_PLACEMENT.rotation, position)
        setcubeplacement(robot, cube, cube_matrix)
        q_grasp, successFlag = computeqgrasppose(robot, robot.q0, cube, cube_matrix, viz)
        path.append(q_grasp)
    return path
        

def displaypath(robot,path,dt,viz):
    for q in path:
        viz.display(q)
        time.sleep(dt)


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path, success = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,path,dt=0.5,viz=viz) #you ll probably want to lower dt


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# """
# Created on Thu Sep 21 11:44:32 2023

# @author: stonneau
# """
# from inverse_geometry import computeqgrasppose

# import pinocchio as pin
# import numpy as np
# from numpy.linalg import pinv

# from config import LEFT_HAND, RIGHT_HAND, EPSILON, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
# import time
# from tools import collision, jointlimitsviolated, setcubeplacement, setupwithmeshcat
# from config import CUBE_PLACEMENT
# from setup_pinocchio import loadrobot, loadobject, finalisecollisionsetup

# from pinocchio.utils import rotate

    
# #returns a collision free path from qinit to qgoal under grasping constraints
# #the path is expressed as a list of configurations
# def RAND_CONF(cube_placement, cube_goal, robot=None, cube=None):
#     '''
#     Return a random configuration, not is collision
#     '''
#     GOAL_BIAS = 0.0  # probability to sample the goal directly
#     if np.random.rand() < GOAL_BIAS:
#         cube_translation = cube_goal.translation
#     else:
#         cube_translation = cube_placement.translation

#     min_ranges = np.array([0.1,0.35,0.05])  # define the range for x, y, z around the cube
#     max_ranges = np.array([0.1,0.35,0.3])  # define the range for x, y, z around the cube
#     min_vals =  cube_translation - min_ranges 
#     max_vals =  cube_translation + max_ranges

#     while True:
#         sample = np.random.uniform(min_vals, max_vals)
#         sample_se3 = pin.SE3(cube_placement.rotation, sample) 
#         setcubeplacement(robot,cube, sample_se3)
        
#         robot_q = computeqgrasppose(robot, robot.q0, cube, sample_se3)
#         if robot_q[1]:
#             return robot_q, sample_se3
#         else:
#             print("sampled configuration in collision, resampling...")        

# def distance(q1,q2):    
#     '''Return the euclidian distance between two configurations'''
#     return np.linalg.norm(q2.translation-q1.translation)
        
# def NEAREST_VERTEX(G, q_cube):
#     '''returns the index of the Node of G with the configuration closest to q_rand  '''
#     min_dist = 10e4
#     idx=-1
#     for (i,node) in enumerate(G):
#         dist = distance(node[1],q_cube) 
#         if dist < min_dist:
#             min_dist = dist
#             idx = i
#     return idx

# def lerp(cube_start, cube_end, t):    
#     return pin.SE3(cube_start.rotation, cube_start.translation * (1 - t) + cube_end.translation * t) 

# def NEW_CONF(robot, cube, cube_near, cube_goal, discretisationsteps, delta_q):
#     '''Return the closest configuration q_new such that the path q_near => q_new is the longest
#     along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q'''
#     cube_end = cube_goal.copy()
#     dist = distance(cube_near, cube_goal)
#     if delta_q is not None and dist > delta_q:
#         #compute the configuration that corresponds to a path of length delta_q
#         cube_end = lerp(cube_near, cube_end, delta_q/dist)

#     dt = 1 / discretisationsteps
#     prev_config = robot.q0.copy()
#     # dividing the path into smaller pieces to check for collisions
#     for i in range(1, discretisationsteps):
#         cubeLerp = lerp(cube_near, cube_end, dt*i)
#         q = computeqgrasppose(robot, prev_config, cube, cubeLerp)[0]
#         prev_config = q
#         if (collision(robot, q)) or (jointlimitsviolated(robot, q)): 
#             return lerp(cube_near, cube_end, dt*(i-1))
#     return cube_end

# def ADD_EDGE_AND_VERTEX(G,parent,cube_placement):
#     print("\nadding a new vertex to the graph\n")
#     G += [(parent,cube_placement)]

# def VALID_EDGE(robot, cube, cube_new, cube_goal, discretisationsteps, delta_q=None, cubeM=None):
#     # Find the farthest valid pose we can reach from cube_new
#     farthest_pose = NEW_CONF(robot, cube, cube_new, cube_goal, discretisationsteps, delta_q)

#     # Check if the pose we reached is the same as the goal
#     # (This will use your new, correct distance function)
#     diff = distance(farthest_pose, cube_goal)

#     return diff < EPSILON

# discretisationsteps_newconf = 5 #To tweak later on
# discretisationsteps_validedge = 5 #To tweak later on
# k = 1000  #To tweak later on
# delta_q = 0.1 #To tweak later on

# def computepath(q_init, q_goal, cubeplacementq0, cubeplacementqgoal):
#     robot, cube, viz = setupwithmeshcat(url="tcp://127.0.0.1:6000")

#     G = [(None, cubeplacementq0)]  #Graph of the RRT as a list of (parent_index, configuration, cube_placement)
#     for _ in range(k):
#         cube_closest_goal = NEAREST_VERTEX(G, cubeplacementqgoal)
#         q_rand, sampledC = RAND_CONF(G[cube_closest_goal][1], cubeplacementqgoal, robot=robot, cube=cube)

#         cube_near_index = NEAREST_VERTEX(G, sampledC)
#         cube_near = G[cube_near_index][1]
        
#         cube_new = NEW_CONF(robot, cube, cube_near, sampledC, discretisationsteps_newconf, delta_q)
#         ADD_EDGE_AND_VERTEX(G, cube_near_index, cube_new)
#         if VALID_EDGE(robot, cube, cube_new, cubeplacementqgoal, discretisationsteps_validedge, delta_q):
#             print("Path found!")
#             ADD_EDGE_AND_VERTEX(G, len(G)-1, cubeplacementqgoal)
#             return extractpath(G, robot=robot, cube=cube), True
        
#     print("path not found")
#     return extractpath(G, robot=robot, cube=cube), False

# def extractpath(G, robot=None, cube=None):
#     displayed_path = []
#     idx = len(G)-1
#     while idx is not None:
#         cube_placement = G[idx][1]
#         q_grasp, success_grasp = computeqgrasppose(robot, robot.q0, cube, cube_placement)
#         displayed_path = [q_grasp] + displayed_path
#         idx = G[idx][0]

#     return displayed_path

# def displaypath(robot,path,dt,viz):
#     for q in path:
#         viz.display(q)
#         time.sleep(dt)


# if __name__ == "__main__":
#     from tools import setupwithmeshcat
#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
#     from inverse_geometry import computeqgrasppose

#     robot, cube, viz = setupwithmeshcat(url="tcp://127.0.0.1:6000")

#     q = robot.q0.copy()
#     q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
#     qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)

#     if not(successinit and successend):
#         print ("error: invalid initial or end configuration")
        
#     else:
#         path, success = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
#         displaypath(robot,path,dt=0.5,viz=viz) #you ll probably want to lower dt
    
   
