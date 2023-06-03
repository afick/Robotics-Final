#!usr/bin/env python

# Authors: Kevin Cao, Carter Kruse, Alex Fick, Aneesh Patnaik
# Date: June 3, 2023

# Import Relevant Libraries
import rospy # Module - ROS APIs
import tf

# ROS APIs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

# Import Python Libraries / Modules
import math
import time
import numpy as np
from enum import Enum

# CSV Files & Plotting
from numpy import genfromtxt
# from matplotlib import pyplot as plt

# Import Libraries
from PD_Class import PD
import BFS_Class

# Running Multiple Robots Concurrently
import threading

# import the frontier and TSP modules 
# import frontier
# import TSP

# Constants
FREQUENCY = 10 #Hz
LINEAR_VELOCITY = 0.2 #m/s
ANGULAR_VELOCITY = math.pi/4 #rad/s
ORIGIN = (5, 5) #(m, m)

RESOLUTION = 0.05
HEIGHT = 400
WIDTH = 400

THRESHOLD = 0.75 # threshold probability to declare a cell with an obstacle
THRESHOLD_SAMPLE_SIZE = 200 # threshold sample side where observations are finalized

MIN_OBSTACLE_CHECK_RAD = -45.0 / 180 * math.pi
MAX_OBSTACLE_CHECK_RAD = 45.0 / 180 * math.pi
GOAL_DISTANCE = 0.6
AVOID_LINEAR_VELOCITY = 0.1 #m/s
AVOID_ANGULAR_VELOCITY = 90 #rad/s

# Topic names
DEFAULT_CMD_VEL_TOPIC = "robot_0/cmd_vel"
DEFAULT_ODOM_TOPIC = "robot_0/odom"
DEFAULT_SCAN_TOPIC = "robot_0/base_scan" # use scan for actual robot
DEFAULT_MAP_TOPIC = "map"

CUSTOMERS = [(5, 5), (6, 6), (1, 1)]

FINALLY_DONE = False

### CARTER ###
# READ CSV

# # Import Relevant Libraries (Python Modules)
# import csv

# results = []

# # Read the CSV file into an array.
# with open("data.csv") as file:
#     reader = csv.reader(file, quoting = csv.QUOTE_NONNUMERIC) # Constants - Floats
#     for row in reader:
#         results.append(row)

# # Redefine the elements of the array.
# for i in range(len(results)):
#     for j in range(len(results[0])):
#         if results[i][j] == -1:
#             results[i][j] = -100

# # Print the results.
# print(results)
### CARTER ###

### CARTER ###

#     # Create adjacency matrix.
#     # adjacency_matrix = [[len(bfs(data, (points[i][0], points[i][1]), (points[j][0], points[j][1]))) - 1 for j in range(len(points))] for i in range(len(points))]

#     # print(adjacency_matrix)

# if __name__ == "__main__":
#     """Run the main function."""
#     main()
### CARTER ###

class NonstaticObstacle:

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY):

        # Setting up velocity command publishers for an obstacle
        self._cmd_pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=1)

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def translate(self, distance):
        """Helper method that moves the nonstatic obstacle over a specified distance"""
        rate = rospy.Rate(FREQUENCY)

        time = distance/self.linear_velocity

        # Translate for the desired amount of time
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time <= rospy.Duration(time):
            self.move(self.linear_velocity, 0)
            
            rate.sleep()

        self.stop()

    def rotate(self, theta):
        """Helper method that rotates the nonstatic obstacle theta radians"""
    
        rate = rospy.Rate(FREQUENCY)

        time = abs(theta/self.angular_velocity)

        # Rotate for the desired amount of time
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time <= rospy.Duration(time):
            if theta >= 0:
                self.move(0, self.angular_velocity)
            else:
                self.move(0, -self.angular_velocity)

            rate.sleep()

        self.stop()

    def obstacle_motion(self):
        
        end_time = time.time() + 15

        self.rotate(-1 * math.pi)
        while time.time() < end_time:
            self.translate(1)
            self.rotate(math.pi)
            self.translate(1)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg) 


class fsm(Enum):
    
    # SPAWN = 0
    EXPLORE_FRONTIER = 1
    # MOVE = 2
    # RECALCULATE = 3
    AVOID = 4
    # COMPLETE = 5
    TSP = 6
    # add more states here 

class UberEatsCar:

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, resolution = RESOLUTION, width = WIDTH, height = HEIGHT, origin = ORIGIN, threshold = THRESHOLD, sample_size = THRESHOLD_SAMPLE_SIZE, customers = CUSTOMERS,  obstacle_check_angle = [MIN_OBSTACLE_CHECK_RAD, MAX_OBSTACLE_CHECK_RAD], goal_distance = GOAL_DISTANCE, avoid_linear_velocity=AVOID_LINEAR_VELOCITY, avoid_angular_velocity=AVOID_ANGULAR_VELOCITY):

        # Setting up publishers and subscribers for the robot
        self.map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=1)
        self.odomSub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        ######################### Parameters #######################

        # Parameters
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.threshold = threshold
        self.sample_size = sample_size

        # Robot location
        self.xpos = 0
        self.ypos = 0
        self.orientation = 0

        # Robot state
        self.fsm = fsm.EXPLORE_FRONTIER

        # Store customer locations
        self.customers = customers

        #################### Occupancy Grid Mapping #########################

        # Initialize the map parameters
        self.resolution = resolution
        self.height = height
        self.width = width

        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.resolution = self.resolution
        self.map.info.height = self.height
        self.map.info.width = self.width

        self.map.info.origin.position.x = origin[0]
        self.map.info.origin.position.y = origin[1]

        self.map.data = [-1] * self.width * self.height

        # Initial publish
        self.map_pub.publish(self.map)

        # Keep track of cell reading history in order to discriminate between occupied/free
        # Each cell has a 2 element array (unoccupied, occupied) counter
        self.cell_history = [[0, 0] for _ in range(self.width * self.height)]

        #################### PD Controller Variables ######################

        self.obstacle_check_angle = obstacle_check_angle
        self.right_side_obstacle = True    # Determines whether robot should turn left or right to avoid
        self.error = 0     # Scales the urgency at which the robot should turn
        self.goal_distance = goal_distance     # checks
        self.avoid_linear_velocity = avoid_linear_velocity
        self.avoid_angular_velocity = avoid_angular_velocity


        """

        self.kp = 0.5  # Proportional gain
        self.kd = 0.1  # Derivative gain
        self.target_distance = 1.0  # Desired distance from obstacles
        self.prev_error = 0.0  # Previous error value for derivative term
        self.obstacle_direction = 0.0  # Direction of the obstacle relative to the robot
        self.last_t = rospy.get_rostime()   # Stores previous timestamp  

        """ 

    ######################### Storing the Odometry ############################
    def _odom_callback(self, msg):
        """Stores the odometry of the robot"""

        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        explicit_quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quaternion)
        self.orientation = yaw


    ######################### Static/Nonstatic Mapping #########################

    def update_history(self, ray, endpoint):
        """Updates the cell histories based on the laser sensor readings"""

        for x, y in ray:
            index = y * self.width + x
            self.cell_history[index][0] += 1
        
        if 0 <= endpoint[0] < self.width and 0 <= endpoint[1] < self.height:
            index = endpoint[1] * self.width + endpoint[0]
            self.cell_history[index][1] += 1

    def update_grid(self):
        """Updates the occupancy grid based on the updated cell histories"""

        for i in range(len(self.cell_history)):
            if self.cell_history[i] != [0, 0]:
                occupied_probability = self.cell_history[i][1] / (self.cell_history[i][0] + self.cell_history[i][1]) 
                if occupied_probability >= self.threshold:
                    self.map.data[i] = 100
                else:
                    sample_size = self.cell_history[i][0] + self.cell_history[i][1]
                    if self.map.data[i] == 100:
                        if sample_size < self.sample_size:
                            self.map.data[i] = 0
                    else:
                        self.map.data[i] = 0


    def bresenham(self, x0, y0, x1, y1):
        """Raytracing algorithm"""

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        # calculations should be based on the relative locations of the start and end points
        if dx > dy:
            if x0 > x1:
                ray = self.unsteepTrace(x1, y1, x0, y0)
            else:
                ray = self.unsteepTrace(x0, y0, x1, y1)
        else:
            if y0 > y1:
                ray = self.steepTrace(x1, y1, x0, y0)
            else:
                ray = self.steepTrace(x0, y0, x1, y1)

        return ray

    def steepTrace(self, x0, y0, x1, y1):
        """Helper method for the Bresenham algorithm if the ray is steep"""

        points = list()

        dx = x1 - x0
        dy = y1 - y0

        if dx >= 0:
            x_increment = 1
        else:
            x_increment = -1
            dx = x0 - x1
        
        # helps determine whether next cell should be adjacent or diagonal
        d = 2 * dx - dy

        x = x0

        # loop through the y coordinates of the ray going from x0 to x1
        for y in range(y0, y1):

            if 0 <= x < self.width and 0 <= y < self.height:
                points.append((x, y))

            # if d exceeds 0, then increase x coordinate of next cell
            if d > 0:
                x += x_increment
                d += 2 * (dx - dy)
            else:
                d += 2 * dx

        return points

    def unsteepTrace(self, x0, y0, x1, y1):
        """Helper method for the Bresenham algorithm if the ray is not steep"""

        points = list()

        dx = x1 - x0
        dy = y1 - y0
        
        if dy >= 0:
            y_increment = 1
        else:
            y_increment = -1
            dy = y0 - y1

        # helps determine whether next cell should be adjacent or diagonal
        d = 2 * dy - dx
        
        y = y0

        # loop through the x coordinates of the ray going from x0 to x1
        for x in range(x0, x1):

            if 0 <= x < self.width and 0 <= y < self.height:
                points.append((x, y))

            # if d exceeds 0, then increase y coordinate of next cell
            if d > 0:
                y += y_increment
                d += 2 * (dy - dx)
            else:
                d += 2 * dy

        return points


    #################### Processing Laser Scan Data #####################

    def laser_callback(self, msg):
        """Processing of laser message."""
        ### CARTER ###
        
        min_index = max(int((self.obstacle_check_angle[0] - msg.angle_min) / msg.angle_increment), 0)
        max_index = min(int((self.obstacle_check_angle[1] - msg.angle_min) / msg.angle_increment), len(msg.ranges) - 1)

        min_distance = msg.range_max
        i = min_index 

        # Loop through the distance data to see if any valid distances are less than the current minimum distance
        for index in range(min_index, max_index + 1):

            # If the distance is both valid and less than the current minimum distance, then replace the min with this new value
            if msg.ranges[index] < min_distance and msg.range_min < msg.ranges[index] < msg.range_max:
                    
                min_distance = msg.ranges[index]
                i = index

        if i > (min_index + max_index) / 2:
            self.right_side_obstacle = False
        else:
            self.right_side_obstacle = True

        # Calculate the error, which is the difference between the current distance and the goal distance.
        if min_distance < self.goal_distance:
            print("OBSTACLE DETECTED")
            self.error = min_distance - self.goal_distance
            self.fsm = fsm.AVOID
        else:
            self.error = 0
            if FINALLY_DONE:
                self.fsm = fsm.TSP
            else:
                self.fsm = fsm.EXPLORE_FRONTIER
            #     self.fsm = fsm.RECALCULATE

            # return to path function

        # Set the finite state machine to 'MOVE'.
        # self.fsm = fsm.MOVE
        
        ### CARTER ###
        
        ###### mapping/nonstatic detection ######

        for i, distance in enumerate(msg.ranges):

            # first check to see if the range reading is valid
            if msg.range_min <= distance <= msg.range_max:

                angle = self.orientation + msg.angle_min + i * msg.angle_increment

                # get the obstacle location in odom reference frame
                x_obstacle = self.xpos + distance * math.cos(angle)
                y_obstacle = self.ypos + distance * math.sin(angle)

                # convert the obstacle location to grid coordinates
                x_obstacle = int((x_obstacle + self.map.info.origin.position.x) / self.resolution)
                y_obstacle = int((y_obstacle + self.map.info.origin.position.y) / self.resolution)

                # convert the current location to grid coordinates
                x_loc = int((self.xpos + self.map.info.origin.position.x) / self.resolution)
                y_loc = int((self.ypos + self.map.info.origin.position.y) / self.resolution)

                points = self.bresenham(x_loc, y_loc, x_obstacle, y_obstacle)

                self.update_history(points, (x_obstacle, y_obstacle))

        self.update_grid()

        self.map_pub.publish(self.map)

    ######################### Movement Commands ############################

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def translate(self, distance):
        """Helper method that moves the robot over a specified distance"""
        rate = rospy.Rate(FREQUENCY)

        time = distance/self.linear_velocity

        # Translate for the desired amount of time
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time <= rospy.Duration(time):
            self.move(self.linear_velocity, 0)
            
            rate.sleep()

        self.stop()

    def rotate(self, theta):
        """Helper method that rotates the robot theta radians"""
    
        rate = rospy.Rate(FREQUENCY)

        time = abs(theta/self.angular_velocity)

        # Rotate for the desired amount of time
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time <= rospy.Duration(time):
            if theta >= 0:
                self.move(0, self.angular_velocity)
            else:
                self.move(0, -self.angular_velocity)

            rate.sleep()

        self.stop() 

    def move_to(self, x, y):
        """Helper method that translates the robot from its current position
        to a specified point (x,y) in a straight line"""

        xdiff = x - self.xpos
        ydiff = y - self.ypos

        # First, rotate to the proper direction facing (x,y)
        angle = math.atan2(ydiff, xdiff)
        angle -= self.orientation
        self.rotate(angle)

        # Then, translate the proper distance 
        distance = math.sqrt(math.pow(xdiff, 2) + math.pow(ydiff, 2))
        self.translate(distance)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)
    
    def frontier_exploration(self, map):

        """
        Input: 2D array form of the map
        Output: List of frontier points to be visited
        """

        # Region-Growing (BFS)
        regions = []
        to_visit = set()

        # Add to the 'to_visit' set.
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j] != 0:
                    to_visit.add((j, i))
        
        # Algorithm
        while (len(to_visit) != 0):
            queue = []
            region = []

            start_point = to_visit.pop()
            queue.append(start_point)

            neighbors_offsets = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            # Continue to loop through points in the queue (until it is empty).
            while (len(queue) != 0):
                current_point = queue.pop(0)
                region.append(current_point)

                # Neighboring Points
                for offset in neighbors_offsets:
                    neighbor_point = (current_point[0] + offset[0], current_point[1] + offset[1])

                    # Check the threshold condition (in the points to visit).
                    if neighbor_point in to_visit:
                        # Add the point to the queue and remove it from 'to_visit'.
                        queue.append(neighbor_point)
                        to_visit.remove(neighbor_point)
            
            # Minimum Region Size
            if len(region) > 5:
                regions.append(region)
        
        # print(regions)
        # print(len(regions))

        # # # # #

        points = []

        for region in regions:
            points.append([sum(ele) / len(region) for ele in zip(*region)])
        
        print(points)

        return points

    def spin(self):
        
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz

        # if the robot just spawned, stay still for 3 seconds and just map surrounding environment
        rospy.sleep(3)

        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C     

            if self.fsm == fsm.EXPLORE_FRONTIER:

                # restructure the map into a 2D array
                reshaped_map = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))

                # completeness check 
                ### NOTE: insert completeness check here ###
                # NOTE: CHANGE NAME
                FINALLY_DONE = True

                for point in CUSTOMERS:
                    if reshaped_map[int(point[0] / self.resolution)][int(point[1] / self.resolution)] != 0:
                        FINALLY_DONE = False

                print(FINALLY_DONE)
                
                # NOTE: CARTER
                # # Read the occupancy grid and create a numpy array.
                # occupancy_grid = np.array(grid_msg.data).reshape((height, width))

                # # Create a copy of the original occupancy grid.
                # new_grid = np.copy(occupancy_grid)

                # # Iterate over each cell in the occupancy grid.
                # for i in range(occupancy_grid.shape[0]):
                #     for j in range(occupancy_grid.shape[1]):
                #         # If the cell is 0 and has a neighboring cell (representing and obstacle)
                #         # within a set distance, set it to 100 (obstacle).
                #         if occupancy_grid[i][j] == 0:
                #             for k in range(-distance, distance + 1):
                #                 for l in range(-distance, distance + 1):
                #                     # Check the bounds of the occupancy grid.
                #                     if i + k >= 0 and i + k < occupancy_grid.shape[0] and j + l >= 0 and j + l < occupancy_grid.shape[1]:
                #                         if occupancy_grid[i + k][j + l] == 100:
                #                             new_grid[i][j] = 100
                #                             break
                # NOTE: CARTER

                # Pad the grid walls
                
                # FIXME
                distance = 6

                # Create a copy of the original occupancy grid.
                new_grid = np.copy(reshaped_map)

                # Iterate over each cell in the occupancy grid.
                for i in range(reshaped_map.shape[0]):
                    for j in range(reshaped_map.shape[1]):
                        # If the cell is 0 and has a neighboring cell (representing and obstacle)
                        # within a set distance, set it to 100 (obstacle).
                        if reshaped_map[i][j] == 0:
                            for k in range(-distance, distance + 1):
                                for l in range(-distance, distance + 1):
                                    # Check the bounds of the occupancy grid.
                                    if i + k >= 0 and i + k < reshaped_map.shape[0] and j + l >= 0 and j + l < reshaped_map.shape[1]:
                                        if reshaped_map[i + k][j + l] == 100:
                                            new_grid[i][j] = 100
                                            break
                
                # if boolean_done:
                #     self.fsm = fsm.COMPLETE
                
                if not FINALLY_DONE:
                    # find the optimal frontier exploration points
                    
                    frontier_points = self.frontier_exploration(new_grid)
                    
                    # frontier.frontier_exploration(new_grid) # FIXME

                    # pick the closest point 
                    # Cast to integer values.
                    points = [[round(i), round(j)] for [i, j] in frontier_points]

                    # allocation of path memory 
                    shortest_path = [[0 for _ in range(1000)] for _ in range(1000)]

                    for i in range(len(points)):
                        for j in range(len(points)):
                            if i != j:
                                # FIXME
                                new_path = BFS_Class.BFS(new_grid, (points[i][0], points[i][1]), (points[j][0], points[j][1]))

                                if len(new_path) < len(shortest_path):
                                    shortest_path = new_path
                    
                    # print("Shortest Path: ")
                    # print(shortest_path)
                    # print("Length: " + str(len(shortest_path) - 1))

                    # move to the best frontier point
                    for point in shortest_path:
                        self.move_to(point[0] * self.resolution, point[1] * self.resolution)
                else:
                    self.fsm = fsm.TSP
            
            elif self.fsm == fsm.AVOID:
                
                if self.right_side_obstacle:
                    self.move(self.avoid_linear_velocity, self.avoid_angular_velocity)
                else:
                    self.move(self.avoid_linear_velocity, -self.avoid_angular_velocity)

            elif self.fsm == fsm.TSP:
                print("TSM NOW")
                # # restructure the map into a 2D array
                # reshaped_map = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))

                # # Pad the grid walls

                # # FIXME
                # distance = 6

                # # Create a copy of the original occupancy grid.
                # new_grid = np.copy(reshaped_map)

                # # Iterate over each cell in the occupancy grid.
                # for i in range(reshaped_map.shape[0]):
                #     for j in range(reshaped_map.shape[1]):
                #         # If the cell is 0 and has a neighboring cell (representing and obstacle)
                #         # within a set distance, set it to 100 (obstacle).
                #         if reshaped_map[i][j] == 0:
                #             for k in range(-distance, distance + 1):
                #                 for l in range(-distance, distance + 1):
                #                     # Check the bounds of the occupancy grid.
                #                     if i + k >= 0 and i + k < reshaped_map.shape[0] and j + l >= 0 and j + l < reshaped_map.shape[1]:
                #                         if reshaped_map[i + k][j + l] == 100:
                #                             new_grid[i][j] = 100
                #                             break

                # # solve traveling salesman problem  
                # _, _, path = TSP.determine_sequence(self.customers, new_grid)
                # for point in path:
                #     self.move_to(point[0] * self.resolution, point[1] * self.resolution)

                #     if point in self.customers:
                #         self.customers.remove(point)

                # # the robot is done once it reaches the final customer
                # break

            rate.sleep()

 
def main():
    """Main Function."""

    # 1st. initialization of node.
    rospy.init_node("uber_eats_car")

    # Initialization of the class for the uber eats car
    uber_eats_car = UberEatsCar()
    nonstatic_obstacle1 = NonstaticObstacle()

    # Sleep for a few seconds to wait for the registration
    rospy.sleep(6)

    # If interrupted, send a stop command before interrupting
    rospy.on_shutdown(uber_eats_car.stop)
    rospy.on_shutdown(nonstatic_obstacle1.stop)

    try:

        uber_eats_car = threading.Thread(target=uber_eats_car.spin)

        ### Load with more obstacles as desired
        obstacle1 = threading.Thread(target=nonstatic_obstacle1.obstacle_motion)

        uber_eats_car.start()
        obstacle1.start()

        uber_eats_car.join()
        obstacle1.join()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    """Run the main function."""
    main()

        



