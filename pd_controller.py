#!/usr/bin/env python

# Author: Carter Kruse
# Date: April 21, 2023

# Import Relevant Libraries
import rospy # Module - ROS APIs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Import Python Libraries
import math
import numpy as np
from enum import Enum

# Import PD Controller Library
from PD_Class import PD

# Constants
# Topic Names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'

# Velocity
LINEAR_VELOCITY = 0.22 # m/s

# Field Of View (Radians)
MIN_SCAN_ANGLE = -20.0 / 180.0 * math.pi
MAX_SCAN_ANGLE = 20.0 / 180 * math.pi

# Tolerances
GOAL_DISTANCE = 0.6 # m

# Gains
K_P = 12
K_D = 1000

# Frequency at which the loop operates.
FREQUENCY = 10 # Hz

class FSM(Enum):
    MOVE = 0
    STOP = 1

class PD_Controller():
    """Class - ROS Node"""

    def __init__(self, linear_velocity = LINEAR_VELOCITY, min_scan_angle = MIN_SCAN_ANGLE,
                 max_scan_angle = MAX_SCAN_ANGLE, goal_distance = GOAL_DISTANCE,
                 k_p = K_P, k_d = K_D):
        """Initialization Function / Constructor"""

        # Set Up (Publishers / Subscribers)
        # Publisher - Sends velocity commands.
        self.cmd_vel_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size = 1)

        # Subscriber - Recieves messages from the laser.
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size = 1)

        # Parameters
        self.linear_velocity = linear_velocity # Constant Linear Velocity
        self.min_scan_angle = min_scan_angle # Minimum Scan Angle
        self.max_scan_angle = max_scan_angle # Maximum Scan Angle
        self.goal_distance = goal_distance # Goal Distance (To Wall)

        # Gains
        self.k_p = k_p
        self.k_d = k_d

        # PD Control
        self.pd_control = PD(k_p, k_d)

        # FSM Variable
        self.fsm = FSM.STOP

        # Wait for a few seconds for the registration (to ROS master).
        rospy.sleep(2.0)
    
    def move(self, linear_velocity, angular_velocity):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)
    
    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def laser_callback(self, msg):  
        """Processing of laser message."""
        # Access to the index of measurement is determined by the 'front' of the robot.
        # LaserScan Message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

        # Finding the minimum range value between 'min_scan_angle' and 'max_scan_angle', using the min/max indices.
        min_index = max(int(np.floor((self.min_scan_angle - msg.angle_min) / msg.angle_increment)), 0)
        max_index = min(int(np.ceil((self.max_scan_angle - msg.angle_min) / msg.angle_increment)), len(msg.ranges) - 1)
        
        # Determining the minimum distance, which is given according to the list of values.
        min_distance = np.min(msg.ranges[min_index:max_index + 1])

        # Calculate the error, which is the difference between the current distance and the goal distance.
        if min_distance < self.goal_distance:
            self.error = min_distance - self.goal_distance
        else:
            self.fsm = FSM.RECALCULATE
            self.error = 0

            # return to path function

        # self.error = min_distance - self.goal_distance

        # Set the finite state machine to 'MOVE'.
        self.fsm = FSM.MOVE
    
    # pseudocode - return to path function
    # in the past, the PD controller simply stopped and adjusted its
    # forward/backward movement depending on the obstacle
    
    # to make it a little more slick, this is removed, and now the
    # angular velocity is adjusted to avoid the obstacle
    # the issue is finding a way to return to the original path

    # this could be accomplished by setting a point further on the
    # path that the obstacle will not be at, or by finding a way to
    # determine the amount deviated from the original path

    # either way, odometry will have to be used, still have to 
    # figure this out and create an implementation

    def run(self):
        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.

        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            if self.fsm == FSM.MOVE:
                # The robot follows the wall on its right side.

                # Use the PD controller to determine the appropriate angular velocity.
                angular_velocity = -self.pd_control.step(self.error)

                # Move the robot accordingly, by publishing the linear/angular velocities.
                self.move(self.linear_velocity, angular_velocity)

            else:
                self.stop()

            # Looping at 10 Hz.
            rate.sleep()
        
        self.fsm = FSM.STOP

def main():
    """Main Function"""

    # 1st - Initialize node.
    rospy.init_node("PD_Controller")

    # 2nd - Create an instance of the class with the relevant publishers/subscribers.
    controller = PD_Controller()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(controller.stop)

    # PD Controller
    try:
        controller.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node Interrupted")

if __name__ == "__main__":
    """Run the main function."""
    main()
