#!usr/bin/env python
#
# Author: Kevin Cao
# Date: 5/16/2023

import numpy as np
import math
import time 
from enum import Enum

import rospy
import tf

# For the purposes of running multiple robots concurrently
import threading

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

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

# Topic names
DEFAULT_CMD_VEL_TOPIC = "robot_0/cmd_vel"
DEFAULT_ODOM_TOPIC = "robot_0/odom"
DEFAULT_SCAN_TOPIC = "robot_0/base_scan" # use scan for actual robot
DEFAULT_MAP_TOPIC = "map"

class fsm(Enum):
    MOVE = 0
    RECALCULATE = 1
    PD = 2

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


class NonstaticDetection:

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, resolution = RESOLUTION, width = WIDTH, height = HEIGHT, origin = ORIGIN, threshold = THRESHOLD, sample_size = THRESHOLD_SAMPLE_SIZE):

        # Setting up publishers and subscribers for robot 0
        self.map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self.odomSub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        self.fsm = fsm.MOVE

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

        # Parameters
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.threshold = threshold
        self.sample_size = sample_size

        # Robot location
        self.xpos = 0
        self.ypos = 0
        self.orientation = 0

        # Static/Nonstatic trackers

        # each cell has a 2 element array (unoccupied, occupied) counter
        self.cell_history = [[0, 0] for _ in range(self.width * self.height)]

        # PD controll er variables
        self.kp = 0.5  # Proportional gain
        self.kd = 0.1  # Derivative gain
        self.target_distance = 1.0  # Desired distance from obstacles
        self.prev_error = 0.0  # Previous error value for derivative term
        self.obstacle_direction = 0.0  # Direction of the obstacle relative to the robot
        self.last_t = rospy.get_rostime()   # Stores previous timestamp

    def update_history(self, ray, endpoint):

        # if point is found in ray, then it has an unoccupied reading
        for x, y in ray:
            index = y * self.width + x
            self.cell_history[index][0] += 1

        if 0 <= endpoint[0] < self.width and 0 <= endpoint[1] < self.height:
            index = endpoint[1] * self.width + endpoint[0]
            self.cell_history[index][1] += 1

    def update_grid(self):

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

    def _laser_callback(self, msg):
        """Processing of the laser message"""
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        
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

        ###### PD controller ######

        # Extract laser scan data and calculate the error and obstacle direction 
        ranges = msg.ranges
        closest_distance = min(ranges)
        error = self.target_distance - closest_distance
        # Store the current error for the next iteration
        self.prev_error = error
        if closest_distance > self.target_distance:
            self.fsm = fsm.MOVE
            return
        
        self.fsm = fsm.PD
        
        self.obstacle_direction = ranges.index(min(ranges)) * msg.angle_increment

        # Calculate PD terms
        proportional = self.kp * error

        time = rospy.get_rostime()
        # Change in time since last measurement
        dt = (time - self.last_t).to_sec()
        derivative = self.kd * (error - self.prev_error) / dt
        self.last_t = time

        control_signal = proportional + derivative

        # Adjust control signal based on obstacle direction
        if self.obstacle_direction > 0:  # Obstacle on the right
            control_signal *= -1

        self.move(self.linear_velocity, control_signal)
        # Publish the control signal
        # twist_msg = Twist()
        # twist_msg.angular.z = control_signal
        # twist_msg.linear.x = self.linear_velocity
        # self.cmd_vel_pub.publish(twist_msg)

    def _odom_callback(self, msg):
        """Stores the odometry of the robot"""

        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        explicit_quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quaternion)
        self.orientation = yaw

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

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)
    
    def robot_motion(self):

        self.rotate(math.pi/2)
        self.translate(2.5)

        """
        self.rotate(math.pi/2)
        self.translate(1)
        self.rotate(-math.pi/2)
        self.translate(4.7)
        self.rotate(math.pi/2)
        self.translate(4)
        self.rotate(math.pi/2)
        self.translate(5)

        """

        """
        # Test with real robot
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time <= rospy.Duration(30):
            continue

        """
        # self.robot_done = True

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
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


def main():
    """Main Function."""

    # 1st. initialization of node.
    rospy.init_node("nonstatic_detection")

    # Initialization of the class for the path planner.
    nonstatic_detection = NonstaticDetection()
    nonstatic_obstacle = NonstaticObstacle()

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(nonstatic_detection.stop)
    rospy.on_shutdown(nonstatic_obstacle.stop)

    try:

        # Simulation environment test (uncomment and run in PA3 test environment)
        robot = threading.Thread(target=nonstatic_detection.robot_motion)
        nonstatic = threading.Thread(target=nonstatic_obstacle.obstacle_motion)

        robot.start()
        nonstatic.start()

        robot.join()
        nonstatic.join()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    """Run the main function."""
    main()
