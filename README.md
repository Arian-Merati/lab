inverse_geometry.py

We went for an inverse kinematics approach, moving one hand at a time. One hand has priority, and before moving the second hand, we project into the null space to ensure the other hand is not effected.

Path.py

We used the RRT connect algorithm, building one tree from the starting position and another tree from the goal position. Once these trees are within delta_q range, we connect them. This algorithm nearly halved our runtime from the standard RRT, allowing us to have more room to increase the discretisation steps to ensure more checks along our path, making it safer.

Control.py

- We used an inverse dynamics approach, using pin.crba + robot.data.nle to find the required torque
- We also implemented force control to ensure the cube was being properly grasped - making each hand apply a linear force to each side of the cube,  controlled with a paramater fc. We then converted this to be 6D using pin.Motion, ensuring no change to the orientation.
- We also accounted for the mass of the cube and found the corresponding gravity and making each hand apply the necessary force to counterract it.
- We used bezier curves to give us the necessary functions for position, velocity and acceleration at each timestep. To ensure accurate functions, we only used two points per segment, and these were padded out to make acceleration and velocity 0 at the start.
- To stop our robot from taking the same amount of time for each segment regardless of the distance that needs to be travelled, we made the time for each segment be proportional to the distance that needs to be travelled. This stopped the robot from losing grip by making eratic movements to cover large distances in a short time.
