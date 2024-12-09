# Some initial stationary kinematics for the 5 dof Pixar lamp

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from pixar.helpers.GeneratorNode      import GeneratorNode
from pixar.helpers.TransformHelpers   import *
from pixar.helpers.TrajectoryUtils    import *

#
#   Gimbal Kinematics
#
def fkin(q):
    T_01 = T_from_Rp(Rotz(q[0]), pzero())
    T_12 = T_from_Rp(Roty(-np.pi/2)@Roty(-q[1]), pxyz(0, 0, 1))
    T_23 = T_from_Rp(Roty(-q[2]), pxyz(1, 0, 0))
    T_34 = T_from_Rp(Roty(-q[3]), pxyz(1, 0, 0))
    T_45 = T_from_Rp(Rotx(q[4]), pxyz(1, 0, 0))
    return T_01 @ T_12 @ T_23 @ T_34 @ T_45

def Jac(q):
    # currently this only produces the position, not orientation
    a = -sin(q[1]) - sin(q[1]+q[2]) - sin(q[1]+q[2]+q[3])
    b = -cos(q[1]) - cos(q[1]+q[2]) - cos(q[1]+q[2]+q[3])
    d = -cos(q[1]+q[2]) - cos(q[1]+q[2]+q[3])
    e = -sin(q[1]+q[2]) - sin(q[1]+q[2]+q[3])
    f = -cos(q[1]+q[2]+q[3])
    g = -sin(q[1]+q[2]+q[3])

    return np.array([[-a*sin(q[0]), b*cos(q[0]), d*cos(q[0]), f*cos(q[0]), 0],
                     [a*cos(q[0]), b*sin(q[0]), d*sin(q[0]), f*sin(q[0]), 0],
                     [0, a, e, g, 0]])
#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Initialize the current joint position to the starting
        # position and set the desired orientation to match.
        self.qd = np.radians(np.array([0, 135, -90, 0, 0]))
        self.Rd = Reye()

        # Pick the convergence bandwidth.
        self.lam = 20


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # we're just going to make the lamp rotate in a wavy circle around its head
        xd = np.array([np.cos(t), np.sin(t), 1+np.cos(t)])
        vd = np.array([-np.sin(t), np.cos(t), -np.sin(t)])
        
        # Grab the last joint value and desired orientation.
        qdlast = self.qd

        # Compute the inverse kinematics
        xr = p_from_T(fkin(qdlast))
        qddot = np.linalg.pinv(Jac(qdlast)) @ (vd + self.lam*ep(xd, xr))

        # Integrate the joint position.
        qd = qdlast + dt * qddot

        # Save the joint value and desired orientation for next cycle.
        Rd = Reye()
        wd = pzero()
        self.qd = qd
        self.Rd = Rd
        
        # Return the desired joint and task (orientation) pos/vel.
        return (qd, qddot, None, None, Rd, wd)



#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', "/lamp", 100, Trajectory)
    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()


    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
