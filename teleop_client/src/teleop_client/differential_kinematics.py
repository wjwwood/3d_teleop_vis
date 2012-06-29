#!/usr/bin/env python

import sys
from math import *
import numpy as np

class DifferentialKinematics(object):
    """Kinematics of a differential robot"""
    def __init__(self, track_length, radius, x=0.0, y=0.0, w=0.0):
        """
        track_length is the distance from the center of each wheel to the center of axel. (m)
        radius is the radius of the tires. (m)
        x is the initial position of the robot in the X_I direction. (m)
        y is the initial position of the robot in the Y_I direction. (m)
        w is the initial rotation of the robot in the Inertial (I) frame. (rad)
        """
        self.track_length = track_length
        self.radius = radius
        
        self.x = x
        self.y = y
        self.w = w
        
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.inverse_rotation_matrix = np.mat([[cos(self.w), -sin(self.w), 0],
                                               [sin(self.w), cos(self.w),  0],
                                               [0,           0,            1]])
    
    def forward(self, left_wheel_speed = None, right_wheel_speed = None):
        """
        This returns (linear_vel, angular_vel) given left and right wheel speed.
        
        left_wheel_speed is the angular velocity of the left wheel of the robot. (rev/s)
        right_wheel_speed is the angular velocity of the right wheel of the robot. (rev/s)
        Returns the linear and angular velocity of the robot in the robot frame (R).
        """
        if not left_wheel_speed: left_wheel_speed = self.left_wheel_speed
        if not right_wheel_speed: right_wheel_speed = self.right_wheel_speed
        left_wheel_speed = float(left_wheel_speed)
        right_wheel_speed = float(right_wheel_speed)
        self.linear_velocity = (left_wheel_speed + right_wheel_speed) * self.radius / 2.0
        self.angular_velocity = (left_wheel_speed - right_wheel_speed) * self.radius / (2.0 * self.track_length)
        self.left_wheel_speed = left_wheel_speed
        self.right_wheel_speed = right_wheel_speed
        return (self.linear_velocity, self.angular_velocity)
    
    def inverse(self, linear_vel = None, angular_vel = None):
        """
        This returns (left_wheel_speed, right_wheel_speed) given linear and angular velocity.
        
        linear_vel is the linear velocity in the Robot frame (R). (m/s)
        angular_vel is the angular velocity in the Robot frame (R). (rad/s)
        Returns the left wheel and right wheel speed. (rev/s)
        """
        if not linear_vel: linear_vel = self.linear_velocity
        if not angular_vel: angular_vel = self.angular_velocity
        if angular_vel == 0.0:
            R = 0.0
        else:
            R = linear_vel/angular_vel
        if R == 0.0:
            linear_velocity_delta = 0.0
        else:
            linear_velocity_delta = 2 * ((linear_vel * self.track_length)/R)
        if linear_vel == 0.0:
            self.left_wheel_speed = -angular_vel
            self.right_wheel_speed = angular_vel
        else:
            self.left_wheel_speed = (linear_vel + (linear_velocity_delta/2.0))
            self.right_wheel_speed = (linear_vel - (linear_velocity_delta/2.0))
        self.linear_velocity = linear_vel
        self.angular_velocity = angular_vel
        return (self.left_wheel_speed,
                self.right_wheel_speed)
    
    def rotate_to_inertial(self, x_r, y_r, w_r):
        """Rotates the x, y, w in the Robot frame into x, y, w in the Intertial frame"""
        
        zeta_I = self.inverse_rotation_matrix * np.mat([[x_r],
                                                        [y_r],
                                                        [w_r]])
        return (float(zeta_I[0]), float(zeta_I[1]), float(zeta_I[2]))
    
    def step_simulation(self, time_delta):
        """
        Increments the simulation one step given the time delta to step.
        
        time_delta is the duration of the step in the kinematics simulation. (secs)
        Returns the new (x, y, w) position and orientation of the robot.
        """
        # Calculate updates in the robot frame
        x_r = self.linear_velocity * 0.2666666666667 * time_delta
        y_r = 0.0
        w_r = self.angular_velocity * 0.2666666666667 * time_delta
        
        # Rotate into the Inertial frame
        zeta_I = self.rotate_to_inertial(x_r, y_r, w_r)
        
        # Increment the absolute position in the Inertial frame
        self.x += zeta_I[0]
        self.y += zeta_I[1]
        self.w -= zeta_I[2]
        
        # Update the inverse rotation matrix
        self.inverse_rotation_matrix = np.mat([[cos(self.w), -sin(self.w), 0],
                                               [sin(self.w), cos(self.w),  0],
                                               [0,           0,            1]])
        return (self.x, self.y, self.w)
    

if __name__ == '__main__':
    dk = DifferentialKinematics(0.30, 0.15)
    
    import unittest
    
    class DifferentialKinematicsTestCase(unittest.TestCase):
        """Tests the kinematic equations of the differential system"""
        def runTest(self):
            dk1 = DifferentialKinematics(1.0, 1.0, w = pi/2.0)
            self.assertEqual(dk1.forward(4,2), (3,1), "Forward kinematics.")
            self.assertEqual(dk1.inverse(3,1), (4,2), "Inverse kinematics. {0} != {1}".format(dk1.inverse(3,1), (4,2)))
            result = dk1.rotate_to_inertial(3, 0, 1)
            (a,b,c) = result
            (a_,b_,c_) = (0,3,1)
            self.assertAlmostEqual(a, a_, msg="Rotation into Inertial Frame. {0} != {1}".format(dk1.rotate_to_inertial(3, 0, 1), (0,3,1)))
            self.assertAlmostEqual(b, b_, msg="Rotation into Inertial Frame. {0} != {1}".format(dk1.rotate_to_inertial(3, 0, 1), (0,3,1)))
            self.assertAlmostEqual(c, c_, msg="Rotation into Inertial Frame. {0} != {1}".format(dk1.rotate_to_inertial(3, 0, 1), (0,3,1)))
            dk1.inverse(3,0)
            x,y,w = dk1.step_simulation(1.0)
            self.assertAlmostEqual(y, 3.0, msg="Simulation did not step as expected: {0} != {1}".format(y, 3.0))
    
    suite = unittest.TestLoader().loadTestsFromTestCase(DifferentialKinematicsTestCase)
    unittest.TextTestRunner(verbosity=2).run(suite)