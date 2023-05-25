#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.


# Import of python modules.
from math import pi, sqrt 
import math

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for cmd_vel
from sensor_msgs.msg import LaserScan  # message type for scan
from nav_msgs.msg import Odometry # message for odom
from nav_msgs.msg import OccupancyGrid # message for map
import tf # library for transformation
import numpy as np

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_SCAN_TOPIC = 'base_scan'
DEFAULT_MAP_TOPIC = 'map'

# Frequency at which the loop operates
FREQUENCY = 10  # Hz.

# Dimensions of the Occupancy Grid
WIDTH = 150
HEIGHT = 150

LINEAR_VELOCITY = 0.25  # m/s
ANGULAR_VELOCITY = math.pi/16 #rad/s

UPDATE_RATE = 2 # Seconds

class Pa4:
    def __init__(self):
        # initialize ROS node and subscribers/publishers
        self.cmd_vel_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback)
        # self.odom_pub = rospy.Publisher(DEFAULT_ODOM_TOPIC, Odometry, queue_size=10)
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback)
        self.map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=10)
        self.rate = rospy.Rate(FREQUENCY)

        # initialize variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laser_msg = None
        self.linear_velocity = LINEAR_VELOCITY
        self.angular_velocity = ANGULAR_VELOCITY

        # set up the occupancy grid
        self.resolution = 0.1
        self.width = WIDTH
        self.height = HEIGHT
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'odom'
        self.map.info.resolution = self.resolution
        self.map.info.width = self.width
        self.map.info.height = self.height
        self.map.info.origin.position.x = -WIDTH /2 * self.resolution
        self.map.info.origin.position.y = -HEIGHT / 2 * self.resolution
        self.map.data = [-1] * self.width * self.height


        # PID controller variables
        self.kp = 0.5   # Proportional gain
        self.ki = 0.0   # Integral gain
        self.kd = 0.1   # Derivative gain

        self.target_distance = 0.5   # Desired distance from obstacles

        self.current_distance = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

        # Time for checking occupancy grid, only want to update based on update rate to not overload the system
        self.last_time = rospy.get_rostime()
        
    def laser_callback(self, msg):
        """Callback function which is called when a new message of type LaserScan is received by the subscriber."""
        self.laser_msg = msg

        current_distance = min(msg.ranges)

        error = self.target_distance - self.current_distance
        proportional = self.kp * error
        integral += self.ki * error
        derivative = self.kd * (error - self.previous_error)

        control_signal = proportional + integral + derivative

        self.previous_error = error

        # Create a Twist message to control the robot's movement
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5   # Set linear velocity
        cmd_vel_msg.angular.z = control_signal   # Set angular velocity based on control signal

        # Publish the Twist message to control the robot's movement
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # Update the OccupancyGrid map based on update rate
        if rospy.get_rostime() - self.last_time > rospy.Duration(UPDATE_RATE):

            shift_x = int(self.width / 2)
            shift_y = int(self.height / 2)

            # Calculate the initial position of the occupancy grid in the 'odom' frame
            origin_x = self.map.info.origin.position.x + (self.width / 2) * self.resolution
            origin_y = self.map.info.origin.position.y + (self.height / 2) * self.resolution

            self.map.data = [-1] * self.width * self.height

            # Convert the LaserScan to a list of ranges.
            ranges = list(msg.ranges)

            # Update the OccupancyGrid map.
            for i, range_val in enumerate(ranges):

                if range_val == float('inf'):
                    continue

                # Calculate the position of the obstacle, according to the 'odom' reference frame.
                angle = msg.angle_min + i * msg.angle_increment
                x = self.current_x + range_val * math.cos(self.current_yaw + angle)
                y = self.current_y + range_val * math.sin(self.current_yaw + angle)

                # Convert the physical position to grid coordinates with the shift
                x_grid = int(round((x - origin_x) / self.resolution)) + shift_x
                y_grid = int(round((y - origin_y) / self.resolution)) + shift_y

                # Check if the grid coordinates are within the map boundaries.
                if 0 <= x_grid < self.width and 0 <= y_grid < self.height:
                    # Determine the appropriate index for the flattened array.
                    index = y_grid * self.width + x_grid

                    # Apply Bresenham's algorithm from the robot to the occupied cell
                    start_x = x_grid
                    start_y = y_grid
                    end_x = int(round((self.current_x - origin_x) / self.resolution)) + shift_x
                    end_y = int(round((self.current_y - origin_y) / self.resolution)) + shift_y

                    dx = abs(end_x - start_x)
                    dy = abs(end_y - start_y)
                    sx = 1 if start_x < end_x else -1
                    sy = 1 if start_y < end_y else -1
                    err = dx - dy

                    while start_x != end_x or start_y != end_y:
                        # Check if the grid coordinates are within the map boundaries.
                        if 0 <= start_x < self.width and 0 <= start_y < self.height:
                            # Determine the appropriate index for the flattened array.
                            index = start_y * self.width + start_x
                            self.map.data[index] = 0  # Mark grid location as free

                        e2 = 2 * err
                        if e2 > -dy:
                            err -= dy
                            start_x += sx
                        if e2 < dx:
                            err += dx
                            start_y += sy

                    # Set the occupancy value of the occupied cell to 100
                    last_index = y_grid * self.width + x_grid
                    # only mark a cell as occupied if it was not already marked as free, this is to prevent dynamic obstacles from being marked as part of the static environment
                    if self.map.data[last_index] == -1:
                        self.map.data[last_index] = 100  # Mark grid location as occupied

            self.last_time = rospy.get_rostime()
        
            self.map_pub.publish(self.map)