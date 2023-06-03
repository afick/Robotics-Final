#!/usr/bin/env python

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


class Avoider:

    def __init__(self):
        rospy.init_node('avoider')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # PD controller variables
        self.kp = 0.5  # Proportional gain
        self.kd = 0.1  # Derivative gain
        self.target_distance = 1.0 # Desired minimum distance from obstacles
        self.prev_error = 0.0 # Previous error value for derivative term 
        self.obstacle_direction = 0.0  # Direction of the obstacle relative to the robot

        self.linear_velocity = 0.5

        
    def laser_callback(self, msg):
        # Extract laser scan data and calculate the error and obstacle direction
        ranges = msg.ranges
        closest_distance = min(ranges)
        error = self.target_distance - closest_distance
        obstacle_direction = ranges.index(min(ranges)) * msg.angle_increment

        # Calculate PD terms
        proportional = self.kp * error
        derivative = self.kd * (error - self.prev_error)
        control_signal = proportional + derivative

        # Adjust control signal based on obstacle direction
        if obstacle_direction > 0:  # Obstacle on the right
            control_signal *= -1

        # Publish the control signal
        twist_msg = Twist()
        twist_msg.angular.z = control_signal
        twist_msg.linear.x = self.linear_velocity
        self.cmd_vel_pub.publish(twist_msg)

        # Store the current error for the next iteration
        prev_error = error