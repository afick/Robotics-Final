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

# Running Multiple Robots Concurrently
import threading

# Constants
FREQUENCY = 10 #Hz
LINEAR_VELOCITY = 0.1 #m/s
ANGULAR_VELOCITY = math.pi/8 #rad/s
ORIGIN = (-5, -5) #(m, m)

RESOLUTION = 0.05
HEIGHT = 400
WIDTH = 400
SLACK = 4

THRESHOLD = 0.3 # threshold probability to declare a cell with an obstacle
THRESHOLD_SAMPLE_SIZE = 600 # threshold sample size where observations are finalized

MIN_OBSTACLE_CHECK_RAD = -20.0 / 180 * math.pi
MAX_OBSTACLE_CHECK_RAD = 20.0 / 180 * math.pi
GOAL_DISTANCE = 0.4
AVOID_LINEAR_VELOCITY = 0.1 #m/s
AVOID_ANGULAR_VELOCITY = 90 #rad/s

ANGLE_TOLERANCE = 0.04
DISTANCE_TOLERANCE = 0.12

MIN_THRESHOLD_DISTANCE = 1.0

EXPANSION = 7

# Topic names
DEFAULT_CMD_VEL_TOPIC = "robot_0/cmd_vel"
DEFAULT_ODOM_TOPIC = "robot_0/odom"
DEFAULT_SCAN_TOPIC = "robot_0/base_scan" # use scan for actual robot
DEFAULT_MAP_TOPIC = "map"

# Customer Locations
CUSTOMERS = [(6, 1), (1, 1), (1, 4), (6, 5)]


def create_poses(path, res):
    '''
    Function to assemble the pose array and the step sequence that the robot will take
    '''
    # Initalize 3 lists of different types of poses
    poses = [] # List of pose objects for publisher
    prev_angle = 0 # track angle of previous 

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
        pose.position.x = x - (5 - 2.8) + 0.2
        pose.position.y = y - (5 - 2.2) + 0.2
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        poses.append(pose) 


    return poses

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
    print("no path")
    return None, None

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
    MOVE = 1
    AVOID = 2
    TSP = 3
    EXIT = 4

class UberEatsCar:

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, resolution = RESOLUTION, width = WIDTH, height = HEIGHT, origin = ORIGIN, threshold = THRESHOLD, sample_size = THRESHOLD_SAMPLE_SIZE, customers = CUSTOMERS,  obstacle_check_angle = [MIN_OBSTACLE_CHECK_RAD, MAX_OBSTACLE_CHECK_RAD], goal_distance = GOAL_DISTANCE, avoid_linear_velocity=AVOID_LINEAR_VELOCITY, avoid_angular_velocity=AVOID_ANGULAR_VELOCITY, angle_tolerance=ANGLE_TOLERANCE, distance_tolerance=DISTANCE_TOLERANCE, expansion=EXPANSION):

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

        # Robot location
        self.xpos = 0
        self.ypos = 0
        self.orientation = 0

        # Robot state
        self.fsm = fsm.TSP

        self.occupied = [0 for _ in range(width * height)]
        self.not_occupied = [0 for _ in range(width * height)]

        # Store customer locations

        self.customers_global = customers
        self.customers = list()
        for customer in customers:
            self.customers.append((int((customer[0] + 3) / resolution), int((customer[1] + 3) / resolution)))
        # self.customers = customers

        self.angle_tolerance = angle_tolerance
        self.distance_tolerance = distance_tolerance

        self.expansion = expansion  # set the expansion factor

        #################### Occupancy Grid Mapping #########################

        # Initialize the map parameters
        self.resolution = resolution
        self.height = height
        self.width = width

        data = []
        with open('output.csv', 'r') as csvfile: 
            reader = csv.reader(csvfile, delimiter=',') 
            for row in reader:
                data.append(int(float(row[0])))
        self.reshaped_data = np.array(data, dtype = int).reshape((400, 400))

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

        self.map.data = np.ndarray.tolist(self.reshaped_data.flatten())

        # Initial publish
        self.map_pub.publish(self.map)

        self.shortest_path = None
        self.rotating_now = False
        self.still_exploring = True

        #################### Obstacle Avoidance Parameters ######################

        self.obstacle_check_angle = obstacle_check_angle
        self.right_side_obstacle = True    # Determines whether robot should turn left or right to avoid
        self.goal_distance = goal_distance     # Determines whether robot should enter avoid mode
        self.avoid_linear_velocity = avoid_linear_velocity
        self.avoid_angular_velocity = avoid_angular_velocity

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self._cmd_pub.publish(twist_msg)
    

    def _odom_callback(self, msg):
        """Stores the odometry of the robot"""

        self.xpos = msg.pose.pose.position.x - self.map.info.origin.position.x
        self.ypos = msg.pose.pose.position.y - self.map.info.origin.position.y
        quaternion = msg.pose.pose.orientation
        explicit_quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quaternion)
        self.orientation = yaw


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

    def publish_pose_array(self, poses):
        '''
        Pose sequence publishing method, input parameter of the pose array
        '''
        pose_msg = PoseArray()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.poses = poses
        
        self.pose_pub.publish(pose_msg)

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

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

    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz

        # if the robot just spawned, stay still for 3 seconds and just map surrounding environment
        rospy.sleep(3)

        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C

            if self.fsm == fsm.EXIT:
                rospy.logerr("ROS node interrupted.")
                break

            elif self.fsm == fsm.MOVE:
                # move to the best frontier point
                for point in self.shortest_path:

                    self.move_to(point[0] * self.resolution, point[1] * self.resolution)
                
                self.fsm = fsm.EXPLORE_FRONTIER

            elif self.fsm == fsm.AVOID:

                if self.right_side_obstacle:
                    self.move(self.avoid_linear_velocity, self.avoid_angular_velocity)
                else:
                    self.move(self.avoid_linear_velocity, -self.avoid_angular_velocity)

            elif self.fsm == fsm.TSP:
                self.still_exploring = False
                print("TSP")

                self.map_pub.publish(self.map)
                # solve traveling salesman problem  
                _, target_order, path = self.determine_sequence(self.customers, self.reshaped_data)
                print(self.customers_global)
                print(target_order)
                print("(0 in target order corresponds to user's starting location)")
                # flatPath = [element for innerList in path for element in innerList]
                # poses = create_poses(flatPath, RESOLUTION)
                # self.publish_pose_array(poses)
                for subpath in path:
                    poses = create_poses(subpath, self.resolution)
                    self.publish_pose_array(poses)
                    for point in subpath:
                        self.move_to(int(point[0]) * self.resolution, int(point[1]) * self.resolution)

                        if point in self.customers:
                            self.customers.remove(point)

                self.fsm = fsm.EXIT
                # the robot is done once it reaches the final customer
                break

        if self.fsm == fsm.EXIT:
            rospy.signal_shutdown("Script terminated")
            rate.sleep()

    def expand_boundaries(self, grid):
        rows = len(grid)
        cols = len(grid[0])
        # Initialize a map of the same size that can be fully explored 
        newmap = grid.copy()
        # Set the expansion factor
        expand = self.expansion

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
    
    def calc_distances(self, targets, grid): 
        ''' 
        Function that creates adjacency matrix for the target nodes 
        based on the occupancy grid, using A* to find the shortest distance between each node.
        '''
        targets.insert(0, (int(self.xpos/RESOLUTION), int(self.ypos/RESOLUTION))) ## TODO: Replace with current pos.
        # Initialize empty adjacency matrix
        adjacency = np.zeros((len(targets), len(targets)))
        indices = [i for i in range(len(targets))]
        # Create map of reachable and unreachable positions
        padded_map = self.expand_boundaries(grid)
        # Store the paths between two positions
        paths = {}

        # Go through all possible combinations of 2 targets
        for start, end in list(itertools.combinations(indices, 2)):
            steps, length = astar(padded_map, targets[start], targets[end])
            paths[(targets[start], targets[end])] = steps
            paths[(targets[end], targets[start])] = steps[::-1]
            adjacency[start][end], adjacency[end][start] = length, length

        return adjacency, paths

    def determine_sequence(self, targets, grid):
        '''
        Function to determine the sequence in which to visit the targets
        '''
        # Assemble the distance adjacency matrix, and store the paths between individual targets
        dists, subpaths = self.calc_distances(targets, grid)

        # Perform held_karp algorithm to minimize total distance
        time, path = self.held_karp(dists)

        # Put together the full path by combining 
        # the sub paths between the individual targets
        total_path = []
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i+1]
            total_path.append(subpaths[(targets[start], targets[end])])

        return time, path, total_path

    def held_karp(self, dists):
        """
        Implementation of Held-Karp, an algorithm that solves the Traveling
        Salesman Problem using dynamic programming with memoization.

        Parameters:
            dists: distance matrix

        Returns:
            A tuple, (cost, path).
        """
        n = len(dists)

        # Maps each subset of the nodes to the cost to reach that subset, as well
        # as what node it passed before reaching this subset.
        # Node subsets are represented as set bits.
        C = {}

        # Set transition cost from initial state
        for k in range(1, n):
            C[(1 << k, k)] = (dists[0][k], 0)

        # Iterate subsets of increasing length and store intermediate results
        # in classic dynamic programming manner
        for subset_size in range(2, n):
            for subset in itertools.combinations(range(1, n), subset_size):
                # Set bits for all nodes in this subset
                bits = 0
                for bit in subset:
                    bits |= 1 << bit

                # Find the lowest cost to get to this subset
                for k in subset:
                    prev = bits & ~(1 << k)

                    res = []
                    for m in subset:
                        if m == 0 or m == k:
                            continue
                        res.append((C[(prev, m)][0] + dists[m][k], m))
                    C[(bits, k)] = min(res)

        # We're interested in all bits but the least significant (the start state)
        bits = (2**n - 1) - 1

        # Calculate optimal cost
        res = []
        for k in range(1, n):
            res.append((C[(bits, k)][0] , k))
        opt, parent = min(res)

        # Backtrack to find full path
        path = []
        for _ in range(n - 1):
            path.append(parent)
            new_bits = bits & ~(1 << parent)
            __, parent = C[(bits, parent)]
            bits = new_bits
        
        path.append(0)

        return opt, list(reversed(path))


def main():
    """Main Function."""

    # 1st. initialization of node.
    rospy.init_node("uber_eats_car")

    # Initialization of the class for the uber eats car
    uber_eats_car = UberEatsCar()
    nonstatic_obstacle1 = NonstaticObstacle(1)

    # Sleep for a few seconds to wait for the registration
    rospy.sleep(6)

    # If interrupted, send a stop command before interrupting
    rospy.on_shutdown(uber_eats_car.stop)
    rospy.on_shutdown(nonstatic_obstacle1.stop)

    try:
        uber_eats_car = threading.Thread(target=uber_eats_car.spin)

        ### Load with more obstacles as desired
        obstacle1 = threading.Thread(target=nonstatic_obstacle1.spin)

        uber_eats_car.start()
        obstacle1.start()

        uber_eats_car.join()
        obstacle1.join()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    """Run the main function."""
    main()
