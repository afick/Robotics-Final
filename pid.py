#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Avoider:
    def __init__(self):
        rospy.init_node('avoider')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # PID controller variables
        self.kp = 0.5  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.1  # Derivative gain
        self.target_distance = 1.0  # Desired distance from obstacles
        self.prev_error = 0.0  # Previous error value for derivative term
        self.integral = 0.0  # Accumulated integral term
        self.obstacle_direction = 0.0  # Direction of the obstacle relative to the robot

        self.linear_velocity = 0.5
        
    def laser_callback(self, msg):
        # Extract laser scan data and calculate the error and obstacle direction
        ranges = msg.ranges
        closest_distance = min(ranges)
        error = self.target_distance - closest_distance
        obstacle_direction = ranges.index(min(ranges)) * msg.angle_increment

        # Calculate PID terms
        proportional = self.kp * error
        integral += self.ki * error
        derivative = self.kd * (error - self.prev_error)
        control_signal = proportional + integral + derivative

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