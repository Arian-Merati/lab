#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv

from config import LEFT_HAND, RIGHT_HAND, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
import time
from tools import collision, jointlimitsviolated, setcubeplacement
from inverse_geometry import computeqgrasppose

# Global variables for robot, cube, and viz (set by calling code)
robot = None
cube = None
viz = None

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

def cube_in_collision(robot, cube, cube_placement):
      """
      Check if cube collides with environment (table, obstacle).
      
      
      Args:
          robot: Robot model
          cube: Cube model  
          cube_placement: SE3 placement to check
          
      Returns:
          True if cube collides, False otherwise
      """
      
      setcubeplacement(robot, cube, cube_placement)

      # Update Pinocchio's collision detector 
      pin.updateGeometryPlacements(
          cube.model, cube.data,
          cube.collision_model, cube.collision_data,
          cube.q0
      )

      # Check if cube collides with anything (table, obstacle, etc.)
      is_collision = pin.computeCollisions(
          cube.collision_model,
          cube.collision_data,
          False  # Check all collisions, don't stop at first
      )

      return is_collision


def RAND_CONF(G):
    '''
    Return a random configuration for the cube
    '''
    nearest_to_target_idx = NEAREST_VERTEX(G, CUBE_PLACEMENT_TARGET.translation)
    base_placement = G[nearest_to_target_idx][1]
    sample_range_lower = np.array([0.05, 0.05, 0]) 
    sample_range_upper = np.array([0.15, 0.3, 0.3])
    lower_bound = base_placement - sample_range_lower
    upper_bound = base_placement + sample_range_upper

    random_position = np.random.uniform(lower_bound, upper_bound)
    cube_matrix = pin.SE3(CUBE_PLACEMENT.rotation, random_position)

    # checks if cube collide
    if cube_in_collision(robot, cube, cube_matrix):
          return RAND_CONF(G)  # Try different sample

    setcubeplacement(robot, cube, cube_matrix)
    q, successFlag = computeqgrasppose(robot, robot.q0, cube, cube_matrix, viz)
    print("RAND CONF SUCCESS FLAG:", successFlag)
    if successFlag:
        return q, cube_matrix
    else:
        return RAND_CONF(G)
    

def distance(q1,q2):    
    '''Return the euclidian distance between two configurations'''
    print("DISTANCE CALLED")
    print("Q1:", q1)
    print("Q2:", q2)
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

def NEW_CONF(cube_near,cube_rand,discretisationsteps, delta_q = None):
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

        # Check if cube would collide during motion
        if cube_in_collision(robot, cube, cube_placement):
              return lerp(cube_near, cube_end, dt*(i-1))

        q_grasp, successFlag = computeqgrasppose(robot, g_grasp, cube, cube_placement, viz)
        if not successFlag:
            return lerp(cube_near,cube_end,dt*(i-1))
    return cube_end

def ADD_EDGE_AND_VERTEX(G,parent,cubeplacement):
    G += [(parent,cubeplacement)]

def VALID_EDGE(q_new,q_goal,discretisationsteps):
    return np.linalg.norm(q_goal -NEW_CONF(q_new, q_goal,discretisationsteps)) < 1e-3

def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal):
    discretisationsteps_newconf = 5 #To tweak later on
    discretisationsteps_validedge = 5 #To tweak later on
    k = 1000  #To tweak later on
    delta_q = 0.5 #To tweak later on
    cube_goal = cubeplacementqgoal.translation

    G = [(None, cubeplacementq0.translation)] 
    for _ in range(k):
        q_rand, sampled_cube = RAND_CONF(G)   
        sampled_cube_position = sampled_cube.translation
        cube_near_index = NEAREST_VERTEX(G,sampled_cube_position)
        cube_near = G[cube_near_index][1]        
        cube_new = NEW_CONF(cube_near, sampled_cube_position, discretisationsteps_newconf, delta_q = delta_q)    
        ADD_EDGE_AND_VERTEX(G,cube_near_index,cube_new)
        if VALID_EDGE(cube_new,cube_goal,discretisationsteps_validedge):
            print ("Path found!")
            ADD_EDGE_AND_VERTEX(G,len(G)-1,cube_goal)
            path = getpath(G)
            return path, True
        
    print("path not found")
    return getpath(G), False

# def constructpath(G):
#     path = []
#     q_grasp = robot.q0.copy()
#     for i in range (len(G)):
#         cube_matrix = pin.SE3(CUBE_PLACEMENT.rotation, G[i][1])
#         q_grasp, successFlag = computeqgrasppose(robot, q_grasp, cube, cube_matrix, viz)
#         path += [q_grasp]
#     return path

def getpath(G):
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
    print(path)
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
   