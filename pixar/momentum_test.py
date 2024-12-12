# Some initial stationary kinematics for the 5 dof Pixar lamp

import rclpy
import numpy as np
from geometry_msgs.msg import Point

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from pixar.helpers.GeneratorNode      import GeneratorNode
from pixar.helpers.TransformHelpers   import *
from pixar.helpers.TrajectoryUtils    import *

from pixar.helpers.KinematicChain     import KinematicChain

# link length
LENGTH = 0.5

# lamp head radius
RADIUS = 0.25

# link masses
m1 = 0.1
m2 = 0.1
m3 = 0.1
m4 = 0.1
m5 = 0.1

#
#   Lamp Kinematics
#
# def fkin(q):
#     T_01 = T_from_Rp(Rotz(q[0]), pzero())
#     T_12 = T_from_Rp(Roty(-np.pi/2)@Roty(-q[1]), pxyz(0, 0, l))
#     T_23 = T_from_Rp(Roty(-q[2]), pxyz(l, 0, 0))
#     T_34 = T_from_Rp(Roty(-q[3]), pxyz(l, 0, 0))
#     T_45 = T_from_Rp(Rotx(q[4]), pxyz(l, 0, 0))
#     return [T_01, T_12, T_23, T_34, T_45], T_01 @ T_12 @ T_23 @ T_34 @ T_45

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Initialize the current joint position to the starting
        # position and set the desired orientation to match.
        # KC expectes the joint name to be a list
        self.chain0 = KinematicChain(node, 'base', 'link0', self.jointnames()[0:1])
        self.chain1 = KinematicChain(node, 'base', 'link1', self.jointnames()[0:2])
        self.chain2 = KinematicChain(node, 'base', 'link2', self.jointnames()[0:3])
        self.chain3 = KinematicChain(node, 'base', 'head', self.jointnames()) # note that j4 and 5 both act on the head
        # we need each frame for the jacobian calculation above
        self.qd = [0.,0.,0.,0.,0.]
        self.Rd = Reye()

        # Pick the convergence bandwidth.
        self.lam = 20

        self.pub = node.create_publisher(Point, '/com', 10)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5']
    
    # define the jacobian for kinematic calculations
    def Jac(self, q):
        # this is the jacobian calculator for the lamp center of mass, NOT tip
        # re-evaluates at each position
        
        # we can't use the Kinematic Chain class to get the Jacobian because that
        # returns the one for the tip, not the center of mass
        
        # first we must determine where each link's COM is


        pos0, R0, _, _ = self.chain0.fkin(q[0:1])

        pos1, R1 , _, _ = self.chain1.fkin(q[0:2])

        pos2, R2, _, _ = self.chain2.fkin(q[0:3])

        pos3, R3, _, _ = self.chain3.fkin(q[0:5])
    

        # Transformation Matrix is 4x4
        # If you are transfoming a point(instead of a frame), the R part is not needed

        # F0 = T_from_Rp(R0, pos0)
        # F1 = T_from_Rp(R1, pos1)
        # F2 = T_from_Rp(R2, pos2)
        # F3 = T_from_Rp(R3, pos3)

        # c1 = F0 @ pxyz(0, 0, LENGTH/2)
        # c2 = F1 @ pxyz(LENGTH, 0, 0)
        # c3 = F2 @ pxyz(LENGTH/2, 0, 0)
        # c4 = F3 @ pxyz(RADIUS/2, 0, 0)
        c1 = pos0 + pxyz(0, 0, LENGTH/2)
        c2 = pos1 + pxyz(LENGTH, 0, 0)
        c3 = pos2 + pxyz(LENGTH/2, 0, 0)
        c4 = pos3 + pxyz(RADIUS/2, 0, 0)

        # # we also need to find where each joint is
        # p1 = pzero()
        # p2 = F0 @ pxyz(LENGTH, 0, 0)
        # p3 = F1 @ pxyz(LENGTH, 0, 0)
        # p4 = F2 @ pxyz(LENGTH, 0, 0)
        p1 = pzero()
        p2 = pos0+ pxyz(LENGTH, 0, 0)
        p3 = pos1+ pxyz(LENGTH, 0, 0)
        p4 = pos2+ pxyz(LENGTH, 0, 0)
        # note that the fifth joint is connected directly to the fourth link

        # calculate the jacobian for each link's center of mass
        # note that the orientation of the center of mass doesn't really matter
        # J1 = np.zeros((5, 3))

        # J1[:, 0] = np.vstack(cross(nz(), c1 - p1), nz())
        
        # J2 = np.zeros((5, 3))
        # J2[:, 0] = np.vstack(cross(nz(), c2 - p1), nz())
        # J2[:, 1] = np.vstack(cross(ny(), c2 - p2), ny())

        # J3 = np.zeros((5, 3))
        # J3[:, 0] = np.vstack(cross(nz(), c3 - p1), nz())
        # J3[:, 1] = np.vstack(cross(ny(), c3 - p2), ny())
        # J3[:, 2] = np.vstack(cross(ny(), c3 - p3), ny())

        # J4 = np.zeros((5, 3))
        # J4[:, 0] = np.vstack(cross(nz(), c4 - p1), nz())
        # J4[:, 1] = np.vstack(cross(ny(), c4 - p2), ny())
        # J4[:, 2] = np.vstack(cross(ny(), c4 - p3), ny())
        # J4[:, 3] = np.vstack(cross(ny(), c4 - p4), ny())

        # # recall that j5 and j4 are at the same spot, so c4 and p4 are the same as c5 and p5
        # J5 = np.zeros((5, 3))
        # J5[:, 0] = np.vstack(cross(nz(), c4 - p1), nz())
        # J5[:, 1] = np.vstack(cross(ny(), c4 - p2), ny())
        # J5[:, 2] = np.vstack(cross(ny(), c4 - p3), ny())
        # J5[:, 3] = np.vstack(cross(ny(), c4 - p4), ny())
        # J5[:, 4] = np.vstack(cross(nx(), c4 - p4), nx())

        # Each J is 3x5, representing only the linear part

        J1 = np.zeros((3, 5))
        J1[:, 0] = cross(nz(), c1 - p1)

        J2 = np.zeros((3, 5))
        J2[:, 0] = cross(nz(), c2 - p1)
        J2[:, 1] = cross(ny(), c2 - p2)

        J3 = np.zeros((3, 5))
        J3[:, 0] = cross(nz(), c3 - p1)
        J3[:, 1] = cross(ny(), c3 - p2)
        J3[:, 2] = cross(ny(), c3 - p3)

        J4 = np.zeros((3, 5))
        J4[:, 0] = cross(nz(), c4 - p1)
        J4[:, 1] = cross(ny(), c4 - p2)
        J4[:, 2] = cross(ny(), c4 - p3)
        J4[:, 3] = cross(ny(), c4 - p4)

        # c4 and p4 are reused for J5 as per the note
        J5 = np.zeros((3, 5))
        J5[:, 0] = cross(nz(), c4 - p1)
        J5[:, 1] = cross(ny(), c4 - p2)
        J5[:, 2] = cross(ny(), c4 - p3)
        J5[:, 3] = cross(ny(), c4 - p4)
        J5[:, 4] = cross(nx(), c4 - p4)
    
        # now find the total COM jacobian by summing and dividing by the mass
        J = (m1*J1 + m2*J2 + m3*J3 + m4*J4 + m5*J5)/(m1+m2+m3+m4+m5)

        #return J, J1, J2, J3, J4, J5
        return J

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # we're just going to make the center of mass go up and down
        xd = np.array([1, 0, 4+np.cos(t)])
        vd = np.array([0, 0, -np.sin(t)])
        
        # Grab the last joint value and desired orientation.
        qdlast = self.qd

        # Compute the inverse kinematics
        # first we need to use the fkin to find the location of each COM

        pos0, R0, _, _ = self.chain0.fkin(qdlast[0:1])
        pos1, R1 , _, _ = self.chain1.fkin(qdlast[0:2])
        pos2, R2, _, _ = self.chain2.fkin(qdlast[0:3])
        pos3, R3, _, _ = self.chain3.fkin(qdlast[0:5])

        # F0 = T_from_Rp(R0, pos0)
        # F1 = T_from_Rp(R1, pos1)
        # F2 = T_from_Rp(R2, pos2)
        # F3 = T_from_Rp(R3, pos3)

        # xr1 = F0 @ pxyz(0, 0, LENGTH/2)
        # xr2 = F1 @ pxyz(LENGTH, 0, 0)
        # xr3 = F2 @ pxyz(LENGTH/2, 0, 0)
        # xr4 = F3 @ pxyz(RADIUS/2, 0, 0)

        xr1 = pos0 +  pxyz(0, 0, LENGTH/2)
        xr2 = pos1 + pxyz(LENGTH, 0, 0)
        xr3 = pos2 +  pxyz(LENGTH/2, 0, 0)
        xr4 = pos3 +  pxyz(RADIUS/2, 0, 0)

        # the position of the robot's center of mass is used as the reference
        xr = (xr1 + xr2 + xr3 + xr4) / (m1 + m2 + m3 + m4)

        # 5x3 @ 3x1 = 5x1  
        qddot = np.linalg.pinv(self.Jac(qdlast)) @ (vd + self.lam*ep(xd, xr)) 

        # print("self.Jac(qdlast) = ", self.Jac(qdlast).shape)
        # print("pinv = ", np.linalg.pinv(self.Jac(qdlast)).shape)

        # Integrate the joint position.
        qd = qdlast + dt * qddot

        # Save the joint value and desired orientation for next cycle.
        Rd = Reye()
        wd = pzero()
        self.qd = qd
        self.Rd = Rd

        # Publish the point xr

        self.pub.publish(Point_from_p(xr))

        p_base_in_world = [0.0,0.0,0.0]
        q_base_in_world = quat_from_R(Reye())

        print("qd   = ", qd.shape)
        print("qdot = ", qddot.shape)
        
        # Return the desired joint and task (orientation) pos/vel.
        return (qd, qddot, xd, vd, Rd, wd, p_base_in_world,q_base_in_world)
    
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
