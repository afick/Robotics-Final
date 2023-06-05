#!usr/bin/env python

# Authors: Kevin Cao, Carter Kruse, Alex Fick, Aneesh Patnaik
# Date: June 3, 2023

# Import Relevant Libraries
import rospy # Module - ROS APIs
import tf
import itertools

# ROS APIs
from geometry_msgs.msg import PoseArray, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Import Python Libraries / Modules
import math
import time
import numpy as np
from enum import Enum
import Queue
import random

import csv

# CSV Files & Plotting
from numpy import genfromtxt
# from matplotlib import pyplot as plt

# Import Libraries
# from PD_Class import PD
# import BFS_Class

# Running Multiple Robots Concurrently
import threading

# import the frontier and TSP modules 
# import frontier
# import TSP

# Constants
FREQUENCY = 10 #Hz
LINEAR_VELOCITY = 0.1 #m/s
ANGULAR_VELOCITY = math.pi/8 #rad/s
ORIGIN = (-5, -5) #(m, m)

ROBOT = 0.2 * (2 ** (1/2))
RESOLUTION = 0.05
HEIGHT = 400
WIDTH = 400
SLACK = 4

THRESHOLD = 3.6 # threshold probability/odds to declare a cell with an obstacle
THRESHOLD_SAMPLE_SIZE = 600 # threshold sample size where observations are finalized

MIN_OBSTACLE_CHECK_RAD = -20.0 / 180 * math.pi
MAX_OBSTACLE_CHECK_RAD = 20.0 / 180 * math.pi
GOAL_DISTANCE = 0.4
AVOID_LINEAR_VELOCITY = 0.03 #m/s
AVOID_ANGULAR_VELOCITY = math.pi / 2 #rad/s

MIN_THRESHOLD_DISTANCE = 0.75

PADDING_DISTANCE = 5

# Topic names
DEFAULT_CMD_VEL_TOPIC = "robot_0/cmd_vel"
DEFAULT_ODOM_TOPIC = "robot_0/odom"
DEFAULT_SCAN_TOPIC = "robot_0/base_scan" # use scan for actual robot
DEFAULT_MAP_TOPIC = "map"

# (int((2 + 3) / RESOLUTION), int((4 + 3) / RESOLUTION)), (int((1 + 3) / RESOLUTION), int((1 + 3) / RESOLUTION)), 
CUSTOMERS = [(int((1 + 3) / RESOLUTION), int((1 + 3) / RESOLUTION))]
MIN_CHECKPOINT = 3

def create_poses(path, res):
    '''
    Function to assemble the pose array and the step sequence that the robot will take
    '''
    # Initalize 3 lists of different types of poses
    poses = [] # List of pose objects for publisher
    steps = [] # Holds just (x,y) that will be used as polyline path
    prev_angle = 0 # track angle of previous 
    count = 0 # of non-added steps

    # Iterate thru the path
    for i, node in enumerate(path):
        # Transform points for map frame (from grid)
        x = node[0] * res
        y = node[1] * res     

        # Calculate angle (yaw)
        if i < len(path) - 1:
            angle = math.atan2(path[i+1][1] - node[1], path[i+1][0] - node[0])
        else:
            angle = prev_angle

        # Create pose object and assign properties
        pose = Pose()
        pose.position.x = x - 2.8
        pose.position.y = y - 2.8
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        poses.append(pose) 

        # Determine whether to add this step (don't if we on a line, unless the previous min_checkpoint
        # points were omitted, then add it for precision of path; ensure goal is added too)
        if angle != prev_angle or count > MIN_CHECKPOINT or i == len(path) - 1:
            # Store the calculated step
            step = (x, y)
            steps.append(step)
            prev_angle = angle
            count = 0
        else:
            # Count the number of steps on a straight line we've skipped
            count += 1

    return(poses, steps)

def publish_pose_array(self, poses):
    '''
    Pose sequence publishing method, input parameter of the pose array
    '''
    pose_msg = PoseArray()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"
    pose_msg.poses = poses
    
    self.pose_pub.publish(pose_msg)

def euclidean_distance(point1, point2):
    """Euclidean distance heuristic function"""
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def astar(map, start_point, end_point):
    """A* Algorithm with Euclidean distance heuristic"""

    visited = {start_point: None}  # Change the starting point in visited to None
    queue = [(start_point, 0)]  # Queue now holds (point, g_score) tuples

    while queue:
        current, current_g_score = min(queue, key=lambda x: x[1])  # Select the point with the lowest g_score
        queue.remove((current, current_g_score))

        if current == end_point:
            # Reconstruct the path
            shortest_path = []
            while current:
                shortest_path.append(current)
                current = visited[current]
            shortest_path.reverse()  # Reverse the path to get it from start to end
            return shortest_path, len(shortest_path)
            return shortest_path, len(shortest_path)

        adjacent_points = []
        current_col, current_row = current

        # Up
        if current_row > 0 and map[int(current_row - 1)][int(current_col)] != 100:
            adjacent_points.append((current_col, current_row - 1))

        # Right
        if current_col < (len(map[0]) - 1) and map[int(current_row)][int(current_col + 1)] != 100:
            adjacent_points.append((current_col + 1, current_row))

        # Down
        if current_row < (len(map) - 1) and map[int(current_row + 1)][int(current_col)] != 100:
            adjacent_points.append((current_col, current_row + 1))

        # Left
        if current_col > 0 and map[int(current_row)][int(current_col - 1)] != 100:
            adjacent_points.append((current_col - 1, current_row))

        # Up & Right
        if current_row > 0 and current_col < (len(map[0]) - 1) and map[int(current_row - 1)][int(current_col + 1)] != 100:
            adjacent_points.append((current_col + 1, current_row - 1))

        # Up & Left
        if current_row > 0 and current_col > 0 and map[int(current_row - 1)][int(current_col - 1)] != 100:
            adjacent_points.append((current_col - 1, current_row - 1))

        # Down & Right
        if current_row < (len(map) - 1) and current_col < (len(map[0]) - 1) and map[int(current_row + 1)][int(current_col + 1)] != 100:
            adjacent_points.append((current_col + 1, current_row + 1))

        # Down & Left
        if current_row < (len(map) - 1) and current_col > 0 and map[int(current_row + 1)][int(current_col - 1)] != 100:
            adjacent_points.append((current_col - 1, current_row + 1))

        for point in adjacent_points:
            if point not in visited:
                g_score = current_g_score + 1  # Distance between adjacent points is always 1 in this grid
                h_score = euclidean_distance(point, end_point)  # Calculate the heuristic score
                f_score = g_score + h_score  # Calculate the total score
                visited[point] = current
                queue.append((point, f_score))

    # If there's no path found
    return None, None

def bfs(map, start_point, end_point):
        """BFS Function"""
        """When the method / function is called, it will return the appropriate path of the robot corresponding
        to a given map (occupancy grid), along with the starting and ending points. This is where the actual BFS
        algorithm occurs.
        """

        """Parameters"""
        """The map (as an occupancy grid) is given as 'map', the starting point is given as 'start_point', and
        the ending point is given as 'end_point'."""

        """Calculations - BFS Algorithm"""
        visited = {end_point: None}
        queue = [end_point]

        while queue:
            current = queue.pop(0)

            if current == start_point:
                shortest_path = []

                while current:
                    shortest_path.append(current)
                    current = visited[current]
                
                return shortest_path
            
            adjacent_points =[]

            current_col, current_row = current

            # Up
            if current_row > 0:
                if map[int(current_row - 1)][int(current_col)] != 100:
                    adjacent_points.append((current_col, current_row - 1))

            # Right
            if current_col < (len(map[0]) - 1):
                if map[int(current_row)][int(current_col + 1)] != 100:
                    adjacent_points.append((current_col + 1, current_row))

            # Down
            if current_row < (len(map) - 1):
                if map[int(current_row + 1)][int(current_col)] != 100:
                    adjacent_points.append((current_col, current_row + 1))
            
            # Left
            if current_col > 0:
                if map[int(current_row)][int(current_col - 1)] != 100:
                    adjacent_points.append((current_col - 1, current_row))
            
            ## ## ##

            # Up & Right
            if current_row > 0 and current_col < (len(map[0]) - 1):
                if map[int(current_row - 1)][int(current_col + 1)] != 100:
                    adjacent_points.append((current_col + 1, current_row - 1))  

            # Up & Left
            if current_row > 0 and current_col > 0:
                if map[int(current_row - 1)][int(current_col - 1)] != 100:
                    adjacent_points.append((current_col - 1, current_row - 1))    
            
            # Down & Right
            if current_row < (len(map) - 1) and current_col < (len(map[0]) - 1):
                if map[int(current_row + 1)][int(current_col + 1)] != 100:
                    adjacent_points.append((current_col + 1, current_row + 1))
            
            # Down & Left
            if current_row < (len(map) - 1) and current_col > 0:
                if map[int(current_row + 1)][int(current_col - 1)] != 100:
                    adjacent_points.append((current_col - 1, current_row + 1))
            
            for point in adjacent_points:
                if point not in visited:
                    visited[point] = current
                    queue.append(point)

class NonstaticObstacle:

    def __init__(self, robot_num, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, min_threshold_distance = MIN_THRESHOLD_DISTANCE,
            scan_angle=[MIN_OBSTACLE_CHECK_RAD, MAX_OBSTACLE_CHECK_RAD]):

        # Setting up velocity command publishers for an obstacle
        self._cmd_pub = rospy.Publisher("robot_" + str(robot_num) + "/cmd_vel", Twist, queue_size=1)
        self._laser_sub = rospy.Subscriber("robot_" + str(robot_num) + "/base_scan", LaserScan, self._laser_callback, queue_size=1)

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # NOTE: assumption: the one at angle 0 corresponds to the front.

        if not self._close_obstacle:
            # Find the minimum range value between min_scan_angle and
            # max_scan_angle
            # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            # Please double check the LaserScan message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
            ####### TODO: ANSWER CODE BEGIN #######

            # Minimum range is initially set to maximum valid laser detection distance
            minimum_distance = msg.range_max  

            # Find the proper range of indices that correspond to the distance data found in the LaserScan message array 
            min_index = int((self.scan_angle[0] - msg.angle_min) / msg.angle_increment)
            max_index = int((self.scan_angle[1] - msg.angle_min) / msg.angle_increment)

            # Loop through the distance data to see if any valid distances are less than the current minimum distance
            for index in range(min_index, max_index + 1):

                # If the distance is both valid and less than the current minimum distance, then replace the min with this new value
                if msg.ranges[index] < minimum_distance and msg.range_min < msg.ranges[index] < msg.range_max:
                    
                    minimum_distance = msg.ranges[index]  


            # If the minimum distance is less than the minimum threshold distance, then set the flag
            if minimum_distance < self.min_threshold_distance:
                
                self._close_obstacle = True
            
            ####### ANSWER CODE END #######

    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C
            
            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.
            
            if not self._close_obstacle:

                self.move(self.linear_velocity, 0)    # Translate the robot if it's not too close to an obstacle

            else:

                random_angle = random.uniform(-math.pi, math.pi)    # Generate a random direction for the robot to move in
                rotation_time = abs(random_angle / self.angular_velocity)   # Derive the time needed for rotation 
               
                start_time = rospy.get_rostime()    # Start the timer for the rotation 
                # Loop until the desired amount of time for rotation is up 
                while rospy.get_rostime() - start_time <= rospy.Duration(rotation_time):
                    
                    if random_angle < 0:
                        self.move(0, -1 * self.angular_velocity)    # For a clockwise rotation
                    else:
                        self.move(0, self.angular_velocity)     # For a counterclockwise rotation

                self._close_obstacle = False    # Unset the flag when done rotating

            rate.sleep()

class fsm(Enum):
    
    EXPLORE_FRONTIER = 1
    MOVE = 2
    AVOID = 3
    TSP = 4
    EXIT = 5

class UberEatsCar:

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, resolution = RESOLUTION, width = WIDTH, height = HEIGHT, origin = ORIGIN, threshold = THRESHOLD, sample_size = THRESHOLD_SAMPLE_SIZE, customers = CUSTOMERS,  obstacle_check_angle = [MIN_OBSTACLE_CHECK_RAD, MAX_OBSTACLE_CHECK_RAD], goal_distance = GOAL_DISTANCE, avoid_linear_velocity=AVOID_LINEAR_VELOCITY, avoid_angular_velocity=AVOID_ANGULAR_VELOCITY, padding_distance=PADDING_DISTANCE):

        # Setting up publishers and subscribers for the robot
        self.map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=1)
        self.odomSub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("robot_0/pose_sequence", PoseArray, queue_size = 1)

        ######################### Parameters #######################

        # Parameters
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.threshold = threshold
        self.sample_size = sample_size
        self.padding_distance = padding_distance

        # Robot location
        self.xpos = 0
        self.ypos = 0
        self.orientation = 0

        # Robot state
        self.fsm = fsm.EXPLORE_FRONTIER

        # Store customer locations
        self.customers = customers

        self.angle_tolerance = 0.04
        self.distance_tolerance = 0.12

        #################### Occupancy Grid Mapping #########################

        # Initialize the map parameters
        self.resolution = resolution
        self.height = height
        self.width = width

        self.map = OccupancyGrid()
        self.map.header.frame_id = "/robot_0/odom"
        self.map.header.stamp = rospy.Time.now()
        self.map.info.resolution = self.resolution
        self.map.info.height = self.height
        self.map.info.width = self.width

        self.map.info.origin.position.x = origin[0]
        self.map.info.origin.position.y = origin[1]
        self.map.info.origin.position.z = 0
        self.map.info.origin.orientation.x = 0
        self.map.info.origin.orientation.y = 0
        self.map.info.origin.orientation.z = 0
        self.map.info.origin.orientation.w = 0

        self.map.data = [-1] * self.width * self.height
        self.griddata = np.array([-1] * self.width * self.height).reshape(self.width, self.height)

        # Initial publish
        self.map_pub.publish(self.map)

        # Keep track of cell reading history in order to discriminate between occupied/free
        self.occupied = [0 for _ in range(self.width * self.height)]
        self.not_occupied = [0 for _ in range(self.width * self.height)]

        self.shortest_path = None
        self.rotating_now = False
        self.next_goal = None
        self.still_exploring = True

        #################### Obstacle avoidance parameters ######################

        self.obstacle_check_angle = obstacle_check_angle
        self.right_side_obstacle = True    # Determines whether robot should turn left or right to avoid
        self.goal_distance = goal_distance     # checks
        self.avoid_linear_velocity = avoid_linear_velocity
        self.avoid_angular_velocity = avoid_angular_velocity

        self.publish_msg = False


    ######################### Storing the Odometry ############################
    def _odom_callback(self, msg):
        """Stores the odometry of the robot"""

        self.xpos = msg.pose.pose.position.x - self.map.info.origin.position.x
        self.ypos = msg.pose.pose.position.y - self.map.info.origin.position.y
        quaternion = msg.pose.pose.orientation
        explicit_quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quaternion)
        self.orientation = yaw


    def publish_pose_array(self, poses):
        '''
        Pose sequence publishing method, input parameter of the pose array
        '''
        pose_msg = PoseArray()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.poses = poses
        
        self.pose_pub.publish(pose_msg)
        
    #################### Processing Laser Scan Data #####################

    def laser_callback(self, msg):
        """Processing of laser message."""
        
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
            self.fsm = fsm.AVOID
       
        if self.still_exploring and not self.rotating_now:
            """Processing of laser message."""
            # Access to the index of the measurement in front of the robot.
            # LaserScan Message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
            
            if self.fsm != fsm.TSP:
                # Convert the laser scan message to a list of ranges.
                ranges = list(msg.ranges)

                # if self.map.data[index] != 100:

                # Update the occupancy grid map, by looping through the measurements.
                for i, distance in enumerate(ranges):
                    # Check to see if the range is out of bounds.
                    if distance >= msg.range_max:
                        continue

                    # Create a range of values up to the measurement (for free space locations).
                    space_list = np.arange(0.0, distance, self.resolution)
                    
                    for space_range in space_list:
                        # Calculate the position of the free space, according to the 'odom' reference frame.
                        angle = msg.angle_min + i * msg.angle_increment
                        x = self.xpos + space_range * math.cos(self.orientation + angle)
                        y = self.ypos + space_range * math.sin(self.orientation + angle)

                        # Convert the physical position to grid coordinates.
                        x_grid = int(round((x) / self.resolution))
                        y_grid = int(round((y) / self.resolution))

                        # Update the the occupancy grid accordingly.
                        if x_grid >= 0 and x_grid < self.width and y_grid >= 0 and y_grid < self.height:
                            # Determine the appropriate index for the array.
                            index = y_grid * self.width + x_grid

                            self.not_occupied[index] += 1

                    # Calculate the position of the obstacle, according to the 'odom' reference frame.
                    angle = msg.angle_min + i * msg.angle_increment
                    x = self.xpos + distance * math.cos(self.orientation + angle)
                    y = self.ypos + distance * math.sin(self.orientation + angle)

                    # Convert the physical position to grid coordinates.
                    x_grid = int(round((x) / self.resolution))
                    y_grid = int(round((y) / self.resolution))

                    # Update the occupancy grid, accordingly.
                    if x_grid >= 0 and x_grid < self.width and y_grid >= 0 and y_grid < self.height:
                        # Determine the appropriate index for the array.
                        index = y_grid * self.width + x_grid
                        self.occupied[index] += 1

                for index in range(self.width * self.height):
                    
                    if self.occupied[index] * self.threshold > self.not_occupied[index]:
                        self.map.data[index] = 100
                        self.griddata[int(index/self.height)][int(index%self.width)] = 100
                    elif self.occupied[index] == 0 and self.not_occupied[index] == 0:
                        self.map.data[index] = -1
                        self.griddata[int(index/self.height)][int(index%self.width)] = -1
                    else:
                        self.map.data[index] = 0
                        self.griddata[int(index/self.height)][int(index%self.width)] = 0

                # Publish the map information/data to the appropriate topic.
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

    def expand_boundaries(self, grid):
        rows = len(grid)
        cols = len(grid[0])
        # Initialize a map of the same size that can be fully explored 
        newmap = grid.copy()
        # Calculate the expansion factor
        expand = int(0)

        # Iterate through each cell
        for r in range(rows):
            for c in range(cols):
                # If there is an obstacle in the map
                if grid[r][c] == 100: 
                    # Mark this cell as not explorable, and expand this by the appropriate factor
                    newmap[r][c] = 100
                    for v in range(1, expand+1):
                        if r-v >= 0:
                            newmap[r-v][c] = 100
                        if r+v < rows:
                            newmap[r+v][c] = 100
                        if c-v >= 0:
                            newmap[r][c-v] = 100
                        if c+v < cols:
                            newmap[r][c+v] = 100
                        
        return np.array(newmap)
    
    def move_to(self, x, y):
        """Move the robot to a given position."""
        # Rotation
        self.rotating_now = True

        # Calculate using arctan and the current orientation of the robot (modulo 2 pi).
        angle = (math.atan2(y - self.ypos, x - self.xpos) - self.orientation) % (2 * math.pi)

        # Continue to rotate until reaching the appropriate rotation.
        while angle > self.angle_tolerance:
            # If the value of the angle is in the interval (0, pi), rotate counterclockwise (positive angular velocity).
            if angle <= math.pi:
                self.move(0.0, self.angular_velocity)
            
            # If the value of the angle is in the interval (-pi, 0), rotate clockwise (negative angular velocity).
            else:
                self.move(0.0, -self.angular_velocity)
            
            # Recalculate the angle, based on the new orientation.
            angle = (math.atan2(y - self.ypos, x - self.xpos) - self.orientation) % (2 * math.pi)

        self.rotating_now = False

        self.stop()

        # Translation
        # Calculate the x and y distances.
        x_distance = x - self.xpos
        y_distance = y - self.ypos

        # Calculate the actual (Euclidean) distance using sqrt.
        distance = math.sqrt(x_distance ** 2 + y_distance ** 2)

        # Continue moving until reaching the appropriate location.
        while distance > self.distance_tolerance:
            self.move(self.linear_velocity, 0.0)
            
            # Recalculate the x and y distances, based on the new position.
            x_distance = x - self.xpos
            y_distance = y - self.ypos

            # Recalculate the actual (Euclidean) distance, based on the new position.
            distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
        
        self.stop()

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self._cmd_pub.publish(twist_msg)
    
    def frontier_exploration(self, map):
        """
        Input: 2D array form of the map
        Output: List of frontier points to be visited
        """

        data = map

        boundary_data = data.copy()

        for i in range(1, len(data) - 1):
            for j in range(1, len(data[0]) - 1):
                p1 = abs(data[i][j] - data[i][j - 1])
                p2 = abs(data[i][j] - data[i][j + 1])
                p3 = abs(data[i][j] - data[i - 1][j])
                p4 = abs(data[i][j] - data[i + 1][j])

                boundary_data[i][j] = max(p1, p2, p3, p4)

        # # # # #

        # Update the appropriate data.
        for i in range(len(data)):
            for j in range(len(data[0])):
                if data[i][j] == -1:
                    data[i][j] = -100
        
        # # # # #

        updated_boundary_data = data.copy()

        for i in range(1, len(data) - 1):
            for j in range(1, len(data[0]) - 1):
                p1 = abs(data[i][j] - data[i][j - 1])
                p2 = abs(data[i][j] - data[i][j + 1])
                p3 = abs(data[i][j] - data[i - 1][j])
                p4 = abs(data[i][j] - data[i + 1][j])

                updated_boundary_data[i][j] = max(p1, p2, p3, p4)
        
        difference_data = data.copy()

        for i in range(1, len(data) - 1):
            for j in range(1, len(data[0]) - 1):
                difference_data[i][j] = boundary_data[i][j] - updated_boundary_data[i][j]
        
        inter_data = data.copy()

        for i in range(1, len(data) - 1):
            for j in range(1, len(data[0]) - 1):
                inter_data[i][j] = difference_data[i][j] - updated_boundary_data[i][j]
        
        final_data = inter_data.copy()

        for i in range(len(inter_data)):
            for j in range(len(inter_data[0])):
                if abs(inter_data[i][j]) >= 230 or abs(inter_data[i][j]) <= 150:
                    final_data[i][j] = 0
        
        # # # # #

        open_space = data.copy()

        # Update the appropriate data.
        for i in range(len(data)):
            for j in range(len(data[0])):
                if abs(data[i][j]) >= 10:
                    open_space[i][j] = 0
                else:
                    open_space[i][j] = 200
        
        # Region-Growing (BFS)
        regions = []
        to_visit = set()

        # Add to the 'to_visit' set.
        for i in range(len(final_data)):
            for j in range(len(final_data[0])):
                if final_data[i][j] != 0:
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
            if len(region) > 10:
                regions.append(region)

        points = []

        for region in regions:
            points.append([sum(ele) / len(region) for ele in zip(*region)])
        
        for i in range(len(data)):
            for j in range(len(data[0])):
                if data[i][j] == -100:
                    data[i][j] = -1

        return points

    def spin(self):
        
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz

        # if the robot just spawned, stay still for 3 seconds and just map surrounding environment
        rospy.sleep(3)
        self.translate(0.3)

        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C

            if self.fsm == fsm.EXIT:
                rospy.signal_shutdown("Script terminated")
                rospy.logerr("ROS node interrupted.")
                break

            elif self.fsm == fsm.MOVE:
                # move to the best frontier point
                for point in self.shortest_path:
                    # print(point)
                    self.move_to(point[0] * self.resolution, point[1] * self.resolution)
                    if self.next_goal is not None:
                        if self.griddata[self.next_goal[1]][self.next_goal[0]] == 100:
                            self.stop()
                            self.fsm = fsm.EXPLORE_FRONTIER
                self.fsm = fsm.EXPLORE_FRONTIER

            elif self.fsm == fsm.EXPLORE_FRONTIER:
                # restructure the map into a 2D array
                reshaped_map = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
                
                print("EF - MAP DONE")
                
                self.padding_distance = 5

                # Create a copy of the original occupancy grid.
                new_grid = np.copy(reshaped_map)

                # Iterate over each cell in the occupancy grid.
                for i in range(reshaped_map.shape[0]):
                    for j in range(reshaped_map.shape[1]):
                        # If the cell is 0 and has a neighboring cell (representing and obstacle)
                        # within a set distance, set it to 100 (obstacle).
                        if reshaped_map[i][j] == 0:
                            for k in range(-self.padding_distance, self.padding_distance + 1):
                                for l in range(-self.padding_distance, self.padding_distance + 1):
                                    # Check the bounds of the occupancy grid.
                                    if i + k >= 0 and i + k < reshaped_map.shape[0] and j + l >= 0 and j + l < reshaped_map.shape[1]:
                                        if reshaped_map[i + k][j + l] == 100:
                                            new_grid[i][j] = 100
                                            break
                
                # find the optimal frontier exploration points

                print("EF - FRONTIER POINTS")
                
                with open("data_2.csv","w+") as my_csv:
                    csvWriter = csv.writer(my_csv, delimiter=',')
                    csvWriter.writerows(reshaped_map)
            
                my_csv.close()
                use = self.expand_boundaries(new_grid)
                frontier_points = self.frontier_exploration(new_grid)

                print(frontier_points)

                if len(frontier_points) > 0:
                    # pick the closest point 
                    # Cast to integer values.
                    points = [[round(i), round(j)] for [i, j] in frontier_points]

                    # allocation of path memory 
                    self.shortest_path = [[0 for _ in range(1000)] for _ in range(1000)]

                    print("EF - PATH")

                    for i in range(len(points)):
                            
                        use = self.expand_boundaries(new_grid)
                        new_path, length = astar(use, (int((self.xpos) * 20), int((self.ypos) * 20)), (int(points[i][0]), int(points[i][1])))
    
                        if len(new_path) < len(self.shortest_path):
                            self.next_goal = (int(points[i][0]), int(points[i][1]))
                            self.shortest_path = new_path
                    
                    poses, steps = create_poses(self.shortest_path, self.resolution)
                    
                    newposes = []
                    for pose in poses:
                        newpose = pose
                        
                        newpose.position.x = (pose.position.x / self.resolution - 5) * self.resolution
                        newpose.position.y = (pose.position.y / self.resolution - 5) * self.resolution
                        newposes.append(newpose)
                
                    self.publish_pose_array(poses)
                    self.publish_msg = True

                    self.fsm = fsm.MOVE
                else:
                    self.fsm = fsm.TSP

            elif self.fsm == fsm.AVOID:

                if self.right_side_obstacle:
                    self.move(self.avoid_linear_velocity, self.avoid_angular_velocity)
                else:
                    self.move(self.avoid_linear_velocity, -self.avoid_angular_velocity)

            elif self.fsm == fsm.TSP:
                self.still_exploring = False
                print("TSP NOW")

                # RIGHT HERE
                np.savetxt('output.csv', self.map.data, delimiter=",")
                self.fsm = fsm.EXIT
                break

            rate.sleep()

def main():
    """Main Function."""

    # 1st. initialization of node.
    rospy.init_node("uber_eats_car")

    # Initialization of the class for the uber eats car
    uber_eats_car = UberEatsCar()

    # Initialization of the nonstatic obstacles
    nonstatic_obstacle1 = NonstaticObstacle(1)
    nonstatic_obstacle2 = NonstaticObstacle(2)
    nonstatic_obstacle3 = NonstaticObstacle(3)

    # Sleep for a few seconds to wait for the registration
    rospy.sleep(3)

    # If interrupted, send a stop command before interrupting
    rospy.on_shutdown(uber_eats_car.stop)
    rospy.on_shutdown(nonstatic_obstacle1.stop)
    rospy.on_shutdown(nonstatic_obstacle2.stop)
    rospy.on_shutdown(nonstatic_obstacle3.stop)

    try:
        uber_eats_car = threading.Thread(target=uber_eats_car.spin)

        ### Load with more obstacles as desired
        obstacle1 = threading.Thread(target=nonstatic_obstacle1.spin)
        obstacle2 = threading.Thread(target=nonstatic_obstacle2.spin)
        obstacle3 = threading.Thread(target=nonstatic_obstacle3.spin)

        uber_eats_car.start()
        obstacle1.start()
        obstacle2.start()
        obstacle3.start()

        uber_eats_car.join()
        obstacle1.join()
        obstacle2.join()
        obstacle3.join()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    """Run the main function."""
    main()

