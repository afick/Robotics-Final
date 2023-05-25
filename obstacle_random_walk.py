#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Alex Fick
# Date: April 3

# Import of python modules.
import math # use of pi.

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from random import random

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 20 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.5 # m/s
ANGULAR_VELOCITY = math.pi/2 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.75 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_RAD = -60.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +60.0 / 180 * math.pi


class RandomWalk():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

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
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        
        
        if not self._close_obstacle:
            # Find the minimum range value between min_scan_angle and
            # max_scan_angle
            # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            # Please double check the LaserScan message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
            ####### TODO: ANSWER CODE BEGIN #######

            # Initialize the minimum range value
            minimum = msg.range_max

            inc = msg.angle_increment
            # First applicable index of ranges
            begin = int((self.scan_angle[0] + math.pi / 2) / inc) 
            # Last applicable index of ranges
            end = int((self.scan_angle[1] + math.pi / 2) / inc)

            ranges = msg.ranges
            # Store smallest and largest for data validity check below
            smallest = msg.range_min
            largest = msg.range_max

            # Iterate thru the indices of readings
            for i in range(begin, end):
                reading = ranges[i]
                
                if reading >= smallest and reading <= largest: # Data validity check below
                    
                    if minimum > reading: 
                        minimum = reading
                        
                    # Found a reading that is below minimum threshold, set flag to true
                    if reading < self.min_threshold_distance:
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

            ####### TODO: ANSWER CODE BEGIN #######

            # If there is an obstacle, we spin
            if self._close_obstacle: 
                # Choose a random angle between -pi and pi
                angle = random() * 2 * math.pi - math.pi
                # Calculate the duration of the spin
                duration = abs(angle / self.angular_velocity)
                
                # Spin for the calcaulated duration
                start_time = rospy.get_rostime()
                while not rospy.is_shutdown():
                    # Check if done.
                    if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                        break
                    self.move(0, -1 * self.angular_velocity) if angle < 0 else self.move(0, self.angular_velocity)

                # Reset the obstacle flag  
                self._close_obstacle = False
                
            # If no obstacle, move forwards
            else:
                self.move(self.linear_velocity, 0)

            ####### ANSWER CODE END #######

            rate.sleep()


def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("random_walk")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    random_walk = RandomWalk()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(random_walk.stop)

    # Robot random walks.
    try:
        random_walk.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
