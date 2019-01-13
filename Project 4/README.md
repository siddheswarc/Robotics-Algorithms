# Grid Localization using Bayes Filter

The goal of this project is to implement Grid Localization which is a variant of discrete Bayes Localization.

The map is a three-dimensional occupancy grid. At each time step, the algorithm finds out the probabilities of the robot presence at every grid cell. The grid cells with maximum probabilities at each step, characterize the robot's trajectory.
Grid Localization runs in two iterative steps - Motion Model and Observation Model.

After each movement, we compute if that movement can move the robot between grid cells. For each observation, we find the most probable cells where that measurement could have occurred.
