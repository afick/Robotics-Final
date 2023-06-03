from __future__ import division

class PD:
    """PD Class - To understand the content of the class, please read the docstrings carefully."""

    def __init__(self, k_p, k_d):
        """Initialization Function"""
        """Serves to initialize all of the instance variables."""

        """Parameters"""
        """The proportional gain constant is given as 'kp', the derivative gain constant is given as 'kd'."""
        self.k_p = k_p
        self.k_d = k_d
        self.previous_error = None

    def step(self, error):
        """Step Function"""
        """At each sensor update, this method will be called, and will return a command output to control
        the rotation of the robot. This is where the actual PD calculations occur. The discrete version of
        the PD control function is implemented."""

        """Parameters"""
        """The current error (in meters) (difference between the goal location and the current location) is
        given as 'error'."""

        """Calculations - PD Controller"""
        control_command = 0
        if self.previous_error:
            control_command = (self.k_p * error) + (self.k_d * (error - self.previous_error))
            
        self.previous_error = error

        return control_command

    def reset(self):
        """This method is called when the simulation is reset. The PD terms should be reset so that previously
        stored values will not affect the current calculations."""
        self.previous_error = None
