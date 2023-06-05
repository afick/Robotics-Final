# Robotics Final Project
### Aneesh, Carter, Kevin, Alex

**Project Objective**: Program a robot that will explore and map (with path planning) a dynamic environment. This environment may have a static component (which may be unknown) with dynamic elements that modify the robot's motion. Then the robot will visit target destinations, doging moving obstacles along its path.

**Motivation**: Autonomous Vehicles (Pre-Computation and Revision of Optimal Path, Discovery/Update of Original Map)

**Files**:
* uber_eats_exploration.py - the exploration module
* output.csv - the .csv form of the fully mapped occupancy grid generated from uber_eats_exploration.py
* uber_eats_TSP.py - the TSP module
* newmaze.png - the testing environment
* newmaze.world - the world file for running exploration
* newmazetsp.world - the world file for running tsp
* All other files are auxiliary, can be ignored
  
**Usage**:
Run the yaml, world, and appropriate script, depending on whether you want to run the exploration or the tsp.
