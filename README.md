# Robotics Final Project
### Aneesh, Carter, Kevin, Alex

**Project Objective**: Program a robot that will explore and map (with path planning) a dynamic environment. This environment may have a static component (which may be unknown) with dynamic elements that modify the robot's motion. Then the robot will visit target destinations, doging moving obstacles along its path.

**Motivation**: Autonomous Vehicles (Pre-Computation and Revision of Optimal Path, Discovery/Update of Original Map)

**Files**:
* uber_eats_exploration.py - the exploration module
* output.csv - the .csv form of the fully mapped occupancy grid generated from our test of uber_eats_exploration.py
* uber_eats_TSP.py - the TSP module
* newmaze.png - the testing environment
* newmaze.world - the world file

# Robotics Final Project
### Aneesh, Carter, Kevin, Alex

**Project Objective**: Program a robot that will explore and map (with path planning) a dynamic environment. This environment may have a static component (which may be unknown) with dynamic elements that modify the robot's motion. Then the robot will visit target destinations, doging moving obstacles along its path.

**Motivation**: Autonomous Vehicles (Pre-Computation and Revision of Optimal Path, Discovery/Update of Original Map)

**Files**:
* uber_eats_exploration.py - the exploration module
* output.csv - the .csv form of the fully mapped occupancy grid generated from our test run of uber_eats_exploration.py
* uber_eats_TSP.py - the TSP module
* newmaze.png - the testing environment
* newmaze.world - the world file

### Execution

Assuming that we are running this program on Apple Silicon, if not done already, clone this repository via the command
```bash
git clone https://github.com/jaismith/ros-apple-silicon
```

Then, enter in the cloned repository folder
``` bash
cd ros-apple-silicon
```

Then, run Docker. Install Docker if it hasn't been installed yet. 
```bash
docker-compose up --build
``` 

Next, while the previous command is running, open up another terminal and cd into the ros-apple-silicon directory again. Now, run the following command:
```bash
docker-compose exec ros bash
```

Now, enter the following command:
```bash
roscore
```

Once again, while both other processes are running, open another terminal window and again cd into the ros-apple-silicon directory again. Run again the following command:
```bash
docker-compose exec ros bash
```

Now, start loading up the map:
```bash
rosrun stage_ros stageros newmaze.world
```

Note that for ease of testing, we usedw newmazetsp.world to test the TSP module (but it should work regardless).
Again, open another terminal and run the following:
```bash
rosrun map_server map_server newmaze.yml
```

In a similar fashion, open up another window, run docker-compose exec ros bash
and then run: 
```bash
rosrun tf static_transform_publisher 2.8 2.2 0 0 0 0 map robot_0/odom 100
```

Again, load up another terminal, cd into ros-apple-silicon, and run:
```bash
docker-compose exec ros bash
```

To visualize the simulation in rviz, use the following command:
```bash
rosrun rviz rviz
```

Open up another terminal window for the last time and cd into the ros-apple-silicon directory again. Again run this command:
```bash
docker-compose exec ros bash
```

Now, if necessary, cd into the directory in which the script is located. Once this is done, run the script with the following command:
```bash
python uber_eats_exploration.py
```

Or, you could run this script afterwards.
```bash
python uber_eats_TSP.py
```

Finally, in order to view the robot simulation, open up a web browser, go to the link http://localhost:8080/vnc.html, and then connect to the simulation. Use the ctrl + c command to terminate the processes run by the script whenever ready. 
