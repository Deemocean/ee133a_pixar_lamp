# Some initial stationary kinematics for the 5 dof Pixar lamp

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from pixar.helpers.GeneratorNode      import GeneratorNode
from pixar.helpers.TransformHelpers   import *
from pixar.helpers.TrajectoryUtils    import *

# link length
l = 0.5

# lamp head radius
r = 0.25

#
#   Lamp Kinematics
#
def fkin(q):
    T_01 = T_from_Rp(Rotz(q[0]), pzero())
    T_12 = T_from_Rp(Roty(-np.pi/2)@Roty(-q[1]), pxyz(0, 0, l))
    T_23 = T_from_Rp(Roty(-q[2]), pxyz(l, 0, 0))
    T_34 = T_from_Rp(Roty(-q[3]), pxyz(l, 0, 0))
    T_45 = T_from_Rp(Rotx(q[4]), pxyz(l, 0, 0))
    return [T_01, T_12, T_23, T_34, T_45], T_01 @ T_12 @ T_23 @ T_34 @ T_45

def Jac(q):
    # this is the jacobian calculator for the lamp center of mass, NOT tip
    # re-evaluates at each position
    
    # first we must determine where each link's COM is
    frames, _ = fkin(q)
    c1 = frames[0] @ pxyz(0, 0, l/2) #FIXME
    c2 = frames[0] @ frames[1] @ pxyz(l/2, 0, 0)
    c3 = frames[0] @ frames[1] @ frames[2] @ pxyz(l/2, 0, 0)
    c4 = frames[0] @ frames[1] @ frames[2] @ frames[3] @ pxyz(l/2, 0, 0)
    c5 = frames[0] @ frames[1] @ frames[2] @ frames[3] @ frames[4] @ pxyz(r/2, 0, 0)

    # we also need to find where each joint is
    p1 = pzero()
    p2 = frames[0] @ pxyz(l, 0, 0)
    p3 = frames[0] @ frames[1] @ pxyz(l, 0, 0)
    p4 = frames[0] @ frames[1] @ frames[2] @ pxyz(l, 0, 0)
    p5 = frames[0] @ frames[1] @ frames[2] @ frames[3] @ pzero()

    # calculate the columns of the jacobian for the given configuration
    j1 = np.vstack(cross(nz(), c1 - p1))#, nz())
    j2 = np.vstack(cross(ny(), c2 - p2))#, ny())
    j3 = np.vstack(cross(ny(), c3 - p3))#, ny())
    j4 = np.vstack(cross(ny(), c4 - p4))#, ny())
    j5 = np.vstack(cross(nz(), c5 - p5))#, nz())

    # complete the jacobian
    J = np.hstack((j1, j2, j3, j4, j5))

    return J
#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Initialize the current joint position to the starting
        # position and set the desired orientation to match.
        self.qd = np.zeros((5,))
        self.Rd = Reye()

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # we're just going to make the center of mass go up and down
        xd = np.array([0, 0, 1+np.cos(t)])
        vd = np.array([0, 0, -np.sin(t)])
        
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
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
