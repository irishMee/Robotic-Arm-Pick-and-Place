
#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            #Create symbols for joint variables
                q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
                d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
                a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
                alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
              ###DH Parameter Table
              #I need to set up the DH parameter table from the Kuka arm after setting reference frames. 
              #The values for a's and d's were found from the urdf xacro file.

                s = {alpha0:     0, a0:      0, d1:  0.75,
                     alpha1: -pi/2, a1:   0.35, d2:     0,   q2: q2-pi/2,
                     alpha2:     0, a2:   1.25, d3:     0,
                     alpha3: -pi/2, a3: -0.054, d4:  1.50,
                     alpha4:  pi/2, a4:      0, d5:     0,
                     alpha5: -pi/2, a5:      0, d6:     0,
                     alpha6:     0, a6:      0, d7: 0.303,   q7: 0
                    }
              ### Homogeneous Transforms
               ### Homogeneous Transforms
              #These are found from base_link to link_7 which is the gripper

                T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
                               [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                               [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                               [                  0,                   0,            0,               1]])
                T0_1 = T0_1.subs(s)

                T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
                               [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                               [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                               [                  0,                   0,            0,               1]])
                T1_2 = T1_2.subs(s)

                T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
                               [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                               [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                               [                  0,                   0,            0,               1]])
                T2_3 = T2_3.subs(s)

                T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
                               [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                               [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                               [                  0,                   0,            0,               1]])
                T3_4 = T3_4.subs(s)

                T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
                               [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                               [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                               [                  0,                   0,            0,              1]])
                T4_5 = T4_5.subs(s)

                T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                               [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                               [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                               [                  0,                   0,            0,              1]])
                 T5_6 = T5_6.subs(s)

                T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
                               [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                               [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                               [                  0,                   0,            0,              1]])
                T6_G = T6_G.subs(s)

                #Composition of Homogeneous Transforms from base to gripper
                T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
                T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
                T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
                T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
                T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
                T0_G = simplify(T0_6 * T6_G) # base_link to link_G(which is the gripper)

                #There is a correction needed due to the difference in the definition of the Gripper
                #link in URDF versus DH Convention. We start with building a rotation about the 
                #z axis and the y axis
                R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
                              [sin(np.pi),  cos(np.pi), 0, 0],
                              [         0,           0, 1, 0],
                              [         0,           0, 0, 1]])
                R_y = Matrix([[ cos(-np.pi/2), 0, sin(-np.pi/2), 0],
                              [             0, 1,             0, 0],
                              [-sin(-np.pi/2), 0, cos(-np.pi/2), 0],
                              [             0, 0,             0, 1]])
                R_corr = simplify(R_z * R_y)

                #Total Homogeneous Transform between the Base_link and the Gripper_link with
                #the orientation correction applied
                T_total = simplify(T0_G * R_corr)


            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
             # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            # We first need to solve for the cartesian coordinates of the Wrist Center      
            # The total rotation matrix is shown below
            R_T = Matrix([[cos(roll)*cos(pitch), cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw), 
                           cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw)],
                          [sin(roll)*cos(pitch), sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), 
                           sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw)],
                          [-sin(pitch), cos(pitch)*sin(yaw), cos(pitch)*cos(yaw)]])
             #N is the vector along the gripper link z-axis and is the third column in the rotation matrix
            Nx = R_T[0,2]
            Ny = R_T[1,2]
            Nz = R_T[2,2]

            #The link offset is dg from the DH parameter table
            dg = 0.303

            #We can now calculate the wrist center 
            WCx = px - (dg * Nx) 
            WCy = py - (dg * Ny)
            WCz = pz - (dg * Nz)

            #We can calculate the first three joint angles
            q1 = atan2(WCy, WCx)
            #For the second and third angles, we need to calculate the sides of the 
            #triangle in order to use the law of cosines
            #a is equal to a2 from the DH parameter table
            a = 1.25
            #b was found using trig
            b = sqrt(WCy**2 + a**2 - 2*(a*WCx) + WCx**2)
            #D was found using the law of cosines
            D = (WCx**2 + WCy**2 - a**2 - b**2)/(2*a*b)
            #Below are the formulas for the second and third angles
            q2 = atan2(b, a)
            q3 = atan2(D, sqrt(1 - D**2))

            #Now we have to set up the equations for the final three angles
            #Since we have figured out formulas for the first three angles,
            #we can plug those into the rotation matrix of the  first three links
            T0_3 = T0_3.evalf(subs={q1: q1, q2: q2, q3: q3})
            R0_3 = T0_3[0:3,0:3]

            #In order to find the final three angles, I used Euler angles from a rotation matrix
            #where R3_6 = R6(q6)R5(q5)R4(q4) which gives us the following equations
            R3_6 = inv(R0_3) * R_T
            q4 = atan2(R3_6[2,1], R3_6[2,2])
            q5 = atan2(-1*R3_6[2,0], sqrt((R3_6[0,0])**2 + (R3_6[1,0])**2))
            q6 = atan2(R3_6[1,0], R3_6[0,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [q1, q2, q3, q4, q5, q6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    
if __name__ == "__main__":
    IK_server()


