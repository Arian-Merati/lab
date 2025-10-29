#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau

CORRECTED IMPLEMENTATION FOR MOTION PLANNING (Part 1, Task II)
===============================================================

This file implements an RRT (Rapidly-exploring Random Tree) motion planner
that finds collision-free paths while maintaining grasping constraints.

KEY CONCEPTS:
-------------
1. We plan in CUBE SPACE (3D) but store ROBOT CONFIGURATIONS (15D)
2. Each node stores BOTH the cube placement AND the corresponding robot config
3. Path projection ensures grasp constraints are maintained during interpolation

CRITICAL FIX from original implementation:
------------------------------------------
- Original: Stored only cube placements, threw away robot configurations
- Fixed: Stores BOTH cube placements AND robot configurations
- Result: Can now return a valid path of robot configurations
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv, norm

from config import LEFT_HAND, RIGHT_HAND, EPSILON
import time
from tools import collision, jointlimitsviolated, setcubeplacement
from inverse_geometry import computeqgrasppose


# ============================================================================
# SECTION 1: SAMPLING - Generate random valid configurations
# ============================================================================

def sample_random_cube_configuration(robot, cube, cubeplacementq0, viz, max_retries=10):
    """
    Sample a random cube placement and compute corresponding robot configuration.

    STRATEGY:
    - Sample random 3D position for cube (much easier than sampling 15D config!)
    - Use IK (computeqgrasppose) to find robot configuration that grasps cube
    - This automatically satisfies grasp constraints

    Args:
        robot: Robot model
        cube: Cube model
        cubeplacementq0: Initial cube placement (SE3) - used to define sampling bounds
        viz: Visualizer
        max_retries: Maximum number of sampling attempts

    Returns:
        (cube_placement, q_config) if successful
        (None, None) if all attempts failed

    WHY THIS WORKS:
    - Sampling in 3D cube space is MUCH easier than 15D configuration space
    - IK solver automatically ensures hands align with hooks (grasp constraint)
    - Collision checking happens inside computeqgrasppose
    """
    base_position = cubeplacementq0.translation

    # Define sampling bounds - adjust these based on workspace
    # Need to cover the path from start [0.33, -0.3, 0.93] to goal [0.4, 0.11, 0.93]
    # Y-axis range: -0.3 to 0.11 = 0.41m difference!
    # Sampling bounds need to be ASYMMETRIC to cover this
    sample_range_lower = np.array([0.0, 0.35, 0.0])   # Back, left, down
    sample_range_upper = np.array([0.15, 0.45, 0.1])  # Forward, right, up

    for attempt in range(max_retries):
        # Generate random 3D position within bounds
        random_position = np.random.uniform(
            base_position - sample_range_lower,
            base_position + sample_range_upper
        )

        # Create cube placement (keep same rotation as initial placement)
        # NOTE: We only randomize position, not orientation - makes problem easier
        cube_placement = pin.SE3(cubeplacementq0.rotation, random_position)

        # Update cube position in scene
        setcubeplacement(robot, cube, cube_placement)

        # Solve IK to get robot configuration that grasps this cube placement
        # Starting from robot.q0 provides consistent initialization
        q_config, success = computeqgrasppose(robot, robot.q0, cube, cube_placement, viz)

        if success:
            # Successfully found valid grasping configuration!
            # q_config is collision-free, respects joint limits, and grasps the cube
            return cube_placement, q_config

    # All attempts failed - this can happen if sampling in cluttered regions
    return None, None


# ============================================================================
# SECTION 2: PATH PROJECTION - Interpolate while maintaining grasp
# ============================================================================

def project_path_with_grasp_constraint(robot, cube, viz,
                                       q_start, cube_start,
                                       cube_end,
                                       discretisation_steps):
    """
    Interpolate from q_start toward cube_end while maintaining grasp constraint.

    THIS IS THE KEY FUNCTION that ensures grasp constraints are maintained!

    CRITICAL INSIGHT:
    - Linear interpolation in configuration space (q_start -> q_end) does NOT
      maintain grasp constraint! The hands would drift away from the hooks.
    - Solution: Interpolate the CUBE position, then use IK to find robot configs
    - This ensures every intermediate configuration maintains the grasp

    ALGORITHM:
    1. Linearly interpolate cube position from cube_start to cube_end
    2. For each interpolated cube position:
       a. Solve IK to get robot configuration
       b. Check collision and joint limits
       c. If valid, add to path; if not, return partial path

    Args:
        robot: Robot model
        cube: Cube model
        viz: Visualizer
        q_start: Starting robot configuration
        cube_start: Starting cube placement (SE3)
        cube_end: Target cube placement (SE3)
        discretisation_steps: Number of intermediate points

    Returns:
        (path, success) where:
        - path: List of tuples [(q_0, cube_0), (q_1, cube_1), ...]
        - success: True if reached cube_end, False if stopped early

    WHY THIS IS CORRECT:
    - Cube interpolation is smooth and predictable
    - IK ensures hands stay on hooks at each step
    - Collision checking ensures safety
    - Returns partial path on failure (useful for debugging)
    """
    # Initialize path with starting configuration
    path = [(q_start, cube_start)]

    cube_start_pos = cube_start.translation
    cube_end_pos = cube_end.translation

    # Use previous config for warm-starting IK (makes it faster!)
    q_previous = q_start

    for i in range(1, discretisation_steps + 1):
        # Interpolation parameter: 0 at start, 1 at end
        t = i / discretisation_steps

        # Linear interpolation of cube position
        cube_interp_pos = (1 - t) * cube_start_pos + t * cube_end_pos

        # Create cube placement (keep rotation from start)
        cube_interp = pin.SE3(cube_start.rotation, cube_interp_pos)

        # Update cube in scene
        setcubeplacement(robot, cube, cube_interp)

        # Solve IK for this cube placement
        # OPTIMIZATION: Use q_previous as initialization (warm start)
        # This is MUCH faster than starting from robot.q0 every time!
        q_interp, success = computeqgrasppose(robot, q_previous, cube, cube_interp, viz)

        # Validation: Check if this configuration is valid
        if not success:
            # IK failed - can't maintain grasp at this cube position
            return path, False

        if collision(robot, q_interp):
            # Configuration is in collision
            return path, False

        if jointlimitsviolated(robot, q_interp):
            # Joint limits violated
            return path, False

        # Valid configuration! Add to path
        path.append((q_interp, cube_interp))
        q_previous = q_interp  # Update for next warm start

    # Successfully interpolated all the way to cube_end
    return path, True


# ============================================================================
# SECTION 3: RRT PRIMITIVES - Building blocks of the algorithm
# ============================================================================

def find_nearest_node(tree, cube_target):
    """
    Find node in tree with cube placement closest to cube_target.

    NOTE: We measure distance in CUBE SPACE (3D), not configuration space (15D)
    This is much more intuitive and efficient.

    Args:
        tree: List of tree nodes (each node is a dict)
        cube_target: Target cube placement (SE3)

    Returns:
        Index of nearest node
    """
    min_dist = float('inf')
    nearest_idx = 0

    cube_target_pos = cube_target.translation

    for i, node in enumerate(tree):
        cube_pos = node['cube_placement'].translation
        dist = norm(cube_target_pos - cube_pos)

        if dist < min_dist:
            min_dist = dist
            nearest_idx = i

    return nearest_idx


def steer_toward_target(robot, cube, viz,
                       q_near, cube_near,
                       cube_rand,
                       delta_q, discretisation_steps):
    """
    Attempt to extend tree from q_near toward cube_rand.

    STRATEGY:
    - Limit step size to delta_q (prevents too-large jumps)
    - Use path projection to maintain grasp constraint

    Args:
        robot: Robot model
        cube: Cube model
        viz: Visualizer
        q_near: Nearest robot configuration
        cube_near: Nearest cube placement
        cube_rand: Random target cube placement
        delta_q: Maximum step size in cube space
        discretisation_steps: Number of interpolation steps

    Returns:
        (cube_new, q_new, success)
    """
    cube_near_pos = cube_near.translation
    cube_rand_pos = cube_rand.translation

    # Calculate distance in cube space
    dist = norm(cube_rand_pos - cube_near_pos)

    # Limit step size to delta_q
    if dist > delta_q:
        # Take partial step toward target
        t = delta_q / dist
        cube_target_pos = cube_near_pos + t * (cube_rand_pos - cube_near_pos)
        cube_target = pin.SE3(cube_near.rotation, cube_target_pos)
    else:
        # Can reach target in one step
        cube_target = cube_rand

    # Try to connect with grasp constraint
    path, success = project_path_with_grasp_constraint(
        robot, cube, viz,
        q_near, cube_near,
        cube_target,
        discretisation_steps
    )

    if success and len(path) > 1:
        # Successfully extended tree
        q_new, cube_new = path[-1]  # Take last configuration
        return cube_new, q_new, True
    else:
        # Failed to extend
        return None, None, False


def can_connect_to_goal(robot, cube, viz,
                       q_current, cube_current,
                       q_goal, cube_goal,
                       discretisation_steps):
    """
    Check if we can connect current node to goal.

    Args:
        robot: Robot model
        cube: Cube model
        viz: Visualizer
        q_current: Current robot configuration
        cube_current: Current cube placement
        q_goal: Goal robot configuration
        cube_goal: Goal cube placement
        discretisation_steps: Number of interpolation steps

    Returns:
        True if connection is valid, False otherwise
    """
    path, success = project_path_with_grasp_constraint(
        robot, cube, viz,
        q_current, cube_current,
        cube_goal,
        discretisation_steps
    )

    if not success:
        return False

    # Verify we actually reached the goal
    q_final, cube_final = path[-1]
    goal_distance = norm(cube_final.translation - cube_goal.translation)

    return goal_distance < EPSILON


def reconstruct_path(tree):
    """
    Reconstruct path from start to goal using parent pointers.

    IMPORTANT: Returns list of ROBOT CONFIGURATIONS, not cube placements!
    This is what the instructions require.

    Args:
        tree: List of tree nodes

    Returns:
        List of robot configurations from start to goal
    """
    path = []
    current_idx = len(tree) - 1  # Start from goal (last node added)

    while current_idx is not None:
        node = tree[current_idx]
        path.append(node['q'])  # Extract robot configuration
        current_idx = node['parent']

    path.reverse()  # Reverse to get start -> goal order
    return path


# ============================================================================
# SECTION 4: MAIN RRT ALGORITHM
# ============================================================================

def computepath(robot, cube, viz, qinit, qgoal, cubeplacementq0, cubeplacementqgoal):
    """
    Compute collision-free path from qinit to qgoal maintaining grasp constraints.

    ALGORITHM: RRT (Rapidly-exploring Random Tree)
    -----------------------------------------------
    1. Initialize tree with start configuration
    2. Loop k times:
       a. Sample random cube placement
       b. Find nearest node in tree
       c. Extend tree toward sample
       d. Check if goal is reachable
    3. If goal reached, reconstruct path

    WHY RRT?
    --------
    - Probabilistically complete: will find path if one exists (given enough time)
    - Works well in high-dimensional spaces
    - Naturally explores the space

    DUAL REPRESENTATION STRATEGY:
    -----------------------------
    - Sample and measure distances in CUBE SPACE (3D - easy!)
    - Store robot configurations in tree (15D - necessary for execution)
    - Use IK to convert between the two representations

    Args:
        robot: Robot model
        cube: Cube model
        viz: Visualizer
        qinit: Initial robot configuration
        qgoal: Goal robot configuration
        cubeplacementq0: Initial cube placement
        cubeplacementqgoal: Goal cube placement

    Returns:
        (path, success) where:
        - path: List of robot configurations [q0, q1, ..., qgoal]
        - success: True if path found, False otherwise

    PERFORMANCE TUNING:
    -------------------
    - discretisation_steps: Higher = more accurate but slower (10-20 is good)
    - k: More iterations = higher chance of success but slower (500-2000)
    - delta_q: Smaller = smoother path but more nodes (0.3-0.5 is good)
    """

    # ===== PARAMETERS =====
    # These can be tuned for better performance
    k = 1000                          # Maximum iterations
    delta_q = 0.4                     # Maximum step size in cube space (meters)
    discretisation_steps = 10         # Interpolation steps (reduced from 200!)

    # PERFORMANCE NOTE: Your friend had discretisation_steps = 200
    # This meant 200 IK solves per edge = extremely slow!
    # 10-20 steps is usually sufficient and MUCH faster

    # ===== INITIALIZATION =====
    # Tree stores nodes with BOTH cube placement AND robot configuration
    tree = [{
        'q': qinit,                    # Robot configuration (15D)
        'cube_placement': cubeplacementq0,  # Cube placement (SE3)
        'parent': None                 # No parent for root node
    }]

    cube_goal = cubeplacementqgoal

    print(f"Starting RRT motion planning...")
    print(f"Parameters: k={k}, delta_q={delta_q}, discretisation_steps={discretisation_steps}")

    # ===== MAIN RRT LOOP =====
    for iteration in range(k):
        if iteration % 50 == 0:
            print(f"Iteration {iteration}/{k}, tree size: {len(tree)}")

        # ----- STEP 1: SAMPLE -----
        # Generate random cube placement and corresponding robot configuration
        # GOAL BIASING: Sample goal 20% of the time for faster convergence
        if np.random.random() < 0.2:
            cube_rand = cube_goal
            q_rand = qgoal
        else:
            # Increased retries to reduce failures
            cube_rand, q_rand = sample_random_cube_configuration(
                robot, cube, cubeplacementq0, viz, max_retries=30
            )

            if cube_rand is None:
                # Sampling failed, try again
                continue

        # ----- STEP 2: NEAREST -----
        # Find closest node in tree (measured in cube space)
        nearest_idx = find_nearest_node(tree, cube_rand)
        nearest_node = tree[nearest_idx]
        cube_near = nearest_node['cube_placement']
        q_near = nearest_node['q']

        # ----- STEP 3: STEER -----
        # Try to extend tree toward random sample
        cube_new, q_new, success = steer_toward_target(
            robot, cube, viz,
            q_near, cube_near,
            cube_rand,
            delta_q, discretisation_steps
        )

        if not success:
            # Couldn't extend toward this sample (collision or IK failure)
            continue

        # ----- STEP 4: ADD TO TREE -----
        # Successfully extended! Add new node
        new_node = {
            'q': q_new,                      # CRITICAL: Store robot config!
            'cube_placement': cube_new,       # Also store cube placement
            'parent': nearest_idx             # Link to parent for path reconstruction
        }
        tree.append(new_node)

        # ----- STEP 5: CHECK GOAL -----
        # Can we connect to the goal from this new node?
        if can_connect_to_goal(robot, cube, viz,
                              q_new, cube_new,
                              qgoal, cube_goal,
                              discretisation_steps):
            print(f"Path found! Tree size: {len(tree)}, iterations: {iteration}")

            # Add goal node to tree
            goal_node = {
                'q': qgoal,
                'cube_placement': cubeplacementqgoal,
                'parent': len(tree) - 1
            }
            tree.append(goal_node)

            # Reconstruct path from start to goal
            path = reconstruct_path(tree)

            print(f"Path length: {len(path)} configurations")
            return path, True

    # Failed to find path within k iterations
    print(f"Path not found after {k} iterations")
    return None, False


# ============================================================================
# SECTION 5: VISUALIZATION
# ============================================================================

def displaypath(robot, path, dt, viz):
    """
    Display the computed path in the visualizer.

    FIXED: Now correctly handles path as list of robot configurations
    (Original version tried to display cube placements - would crash!)

    Args:
        robot: Robot model
        path: List of robot configurations
        dt: Time step between visualizations (seconds)
        viz: Visualizer
    """
    print(f"Displaying path with {len(path)} configurations...")
    for i, q in enumerate(path):
        viz.display(q)
        time.sleep(dt)
        if i % 10 == 0:
            print(f"  Displaying configuration {i}/{len(path)}")


# ============================================================================
# SECTION 6: TESTING
# ============================================================================

if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose

    print("="*70)
    print("MOTION PLANNING TEST - Part 1, Task II")
    print("="*70)

    # Setup
    robot, cube, viz = setupwithmeshcat()

    # Compute initial and goal configurations
    print("\nComputing initial grasping configuration...")
    q = robot.q0.copy()
    q0, success_init = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)

    if not success_init:
        print("ERROR: Failed to compute initial configuration!")
        exit(1)
    print("✓ Initial configuration found")

    print("\nComputing goal grasping configuration...")
    qe, success_end = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET, viz)

    if not success_end:
        print("ERROR: Failed to compute goal configuration!")
        exit(1)
    print("✓ Goal configuration found")

    # Plan path
    print("\n" + "="*70)
    print("Starting motion planning...")
    print("="*70)

    path, success = computepath(robot, cube, viz, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    if success:
        print("\n" + "="*70)
        print("SUCCESS! Path found!")
        print("="*70)
        print(f"Path contains {len(path)} configurations")

        # Display path
        print("\nDisplaying path (you can adjust dt for speed)...")
        displaypath(robot, path, dt=0.1, viz=viz)

        print("\n✓ Motion planning complete!")
    else:
        print("\n" + "="*70)
        print("FAILED: Could not find path")
        print("="*70)
        print("Try increasing k (max iterations) or adjusting sampling bounds")
