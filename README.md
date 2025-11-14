Part 1 – Inverse Geometry (inverse_geometry.py)

We implemented a hierarchical null-space inverse kinematics scheme with task prioritization for dual-arm grasping control.
- Primary task (right end-effector positioning) solved using pseudo-inverse Jacobian, with left end-effector positioned in the null-space of the primary task. This ensures asymmetric poses are resolved with explicit prioritization.
- Fixed step size (DT = 1e-2) used for configuration integration via pin.integrate(). Convergence achieved within MAX_ITERATIONS=1000 by solving both 6D end-effector errors simultaneously.
- Joint-limit projection (project to joint limits) applied after every iteration to maintain feasibility. This prevents oscillations near joint boundaries and ensures returned configurations are always within valid limits.
- Algorithm converges when both end-effector position/orientation errors fall below EPSILON, followed by validation of collision-free status and joint limit satisfaction.


Part 2 – Motion Planning (path.py)

RRT-based motion planning operating in 3D cube configuration space with kinematics-aware validation. Key design decisions:

- Instead of sampling in the robot's full 14-DOF joint space, we sample directly in 3D cube translation space (keeping orientation fixed). This massively reduces branching factor and makes the search space more interpretable.
- Every sampled cube placement is validated by solving the grasping IK problem. Only configurations with valid IK solutions (reachable by both arms) are added to the graph. This ensures the planner is kinematics-aware rather than purely geometric.

Multi-level validation:
  - Robot-environment collision detection for each IK solution (via collision() function)
  - Cube-environment collision detection for sampled placements (via cube_in_collision())
  - Interpolation checks during path extension: configurations are sampled along segments connecting RRT nodes to verify safety between waypoints

- Sampling region parameters (sample_range_lower/upper) define a bounded region around the cube's target location. IK validation ensures that sampled cube positions lead to valid grasping configurations for both arms.

- Path Extraction: Complete path is reconstructed by backtracking from goal to root through the RRT graph, computing the corresponding joint configurations via inverse kinematics at each cube placement waypoint.


 
Part 3 – Control and Execution (control.py)

Two-phase control architecture combining trajectory tracking with force-based grasp maintenance.

- Raw RRT waypoints smoothed using cubic Bézier interpolation. Each segment connects two waypoints with control points positioned to ensure zero boundary velocities. Derivatives automatically computed for velocity and acceleration references.

- Core control law uses inverse dynamics (τ = M·a_des + nle) with PD feedback on configuration errors:
  - a_des = a_traj + Kp·(q_des - q_current) + Kv·(v_des - v_current)
  - Gains tuned for stable tracking (Kp = 2000, Kv ≈ 90)
  - Compensates for gravity and Coriolis effects via computed non-linear terms

- Jaconian transpose method maintains grasping contact on the cube:
  - F = fc·(p_desired - p_current) pulls each hand toward hook points on cube
  - Force gains (fc ≈ 500-1000) are kept high to maintain secure grasp even during dynamic motions
  - Forces converted to joint torques via Jacobian transpose: τ_force = J^T · F

- Final control torques combine motion tracking and force control:
  - τ_total = τ_tracking + τ_force_left + τ_force_right
