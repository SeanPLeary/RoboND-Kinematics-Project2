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
import numpy as np
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

    
        # Create symbols for DH parameters
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') 
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
 
        
        
        # DH Parameters stored as a dictionary
        s = {alpha0:     0, a0:      0, d1:  0.75,
             alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
             alpha2:     0, a2:   1.25, d3:     0, 
             alpha3: -pi/2, a3: -0.054, d4:  1.50, 
             alpha4:  pi/2, a4:      0, d5:     0, 
             alpha5: -pi/2, a5:      0, d6:     0, 
             alpha6:     0, a6:      0, d7: 0.303, q7: 0
            }   

        # Homogeneous Transforms
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
                       [                  0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s) 

        T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                       [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                  0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s) 

        T6_EE = Matrix([[           cos(q7),            -sin(q7),            0,              a6],
                       [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                  0,                   0,            0,               1]])
        T6_EE = T6_EE.subs(s)

        # total homogeneous transform from base to end-effector
        #T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)

        # Correct from URDF to DH parameters
        
        R_z = Matrix([[    cos(pi),    -sin(pi),           0,            0],
                      [    sin(pi),     cos(pi),           0,            0],
                      [          0,           0,           1,            0],
                      [          0,           0,           0,            1]])

        R_y = Matrix([[ cos(-pi/2),           0,  sin(-pi/2),            0],
                      [          0,           1,           0,            0],
                      [-sin(-pi/2),           0,  cos(-pi/2),            0],
                      [          0,           0,           0,            1]])

        R_corr = simplify(R_z * R_y)
        #T0_EE = simplify(T0_EE*R_corr)

        # Homogenous Transforms for 1st 3 joints to contaol EE position
        T0_3 = simplify(T0_1 * T1_2 * T2_3) 

        # extract rotation matrix
        R0_3 = T0_3[0:3, 0:3]

        (r,p,y) = symbols('r p y') 
        R_roll  = Matrix([[      1,       0,       0],
                          [      0,  cos(r), -sin(r)],
                          [      0,  sin(r),  cos(r)]])

        R_pitch = Matrix([[ cos(p),       0,  sin(p)],
                          [      0,       1,       0],
                          [-sin(p),       0,  cos(p)]])

        R_yaw   = Matrix([[ cos(y), -sin(y),       0],
                          [ sin(y),  cos(y),       0],
                          [      0,       0,       1]])

        # total rotation matrix
        R0_6 = R_roll * R_pitch * R_yaw 
        
        # rotation matrix for last 3 joints (spherical wrist) control rotation of EE
        R3_6 = R0_3.T * R0_6 

        count = len(req.poses)

        
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])



            R0_6_eval = R0_6.evalf(subs={r: roll, p: pitch, y: yaw})

            # Calculate Wrist Center vectors (centered on O5) by subtracting the EE vector
            WC = Matrix([px, py, pz]) - s[d7] * R0_6_eval * Matrix([1, 0, 0])
           

            (WCx, WCy, WCz) = WC[0], WC[1], WC[2]

            # project WC on base plane to get theta1 rotation angle
            theta1 = atan2(WCy, WCx)

            # distance from joint2 to joint5
            r25 = sqrt((WCz - s[d1])**2 + (WCx - s[a1]*cos(theta1))**2 + (WCy - s[a1]*sin(theta1))**2)
            
            # distance from joint2 to joint3
            r23 = s[a2]

            # distance from joint3 to joint5
            r35 = sqrt(s[d4]**2 + s[a3]**2)
            
            # helper angle 
            beta1 = atan2(s[a3], s[d4])
            
            # theta35 is opposite r35. use cose rule to calculate from r23,r25,r35
            cos_theta35 = (r25**2 + r23**2 - r35**2)/(2*r25*r23)
            if 1-cos_theta35**2 < 0:
                sin_theta35 = 0
            else:
                sin_theta35 = sqrt(1-cos_theta35**2)
                
            theta35 = atan2(sin_theta35, cos_theta35)          
            
            # another helper function
            beta2 = atan2(WCz-s[d1], sqrt((WCx-s[a1]*cos(theta1))**2 + (WCy-s[a1]*sin(theta1))**2))
            
            # theta2, beta2, theta35 make-up a right triangle
            theta2 = pi/2 - theta35 - beta2

            # theta25 is opposite r25. use cose rule to calculate from r23,r25,r35
            cos_theta25 = (r35**2 + r23**2 - r25**2)/(2*r35*r23)
            if 1-cos_theta25**2 < 0:
                sin_theta25 = 0
            else:
                sin_theta25 = sqrt(1-cos_theta25**2)
           
            theta25 = atan2(sin_theta25, cos_theta25)
            
            # theta3, beta1, theta25 make-up a right triangle
            theta3 = pi/2 - theta25 - beta1


            # Sperical Wrist Angles

            R3_6_eval = R3_6.evalf(subs={r: roll, p: pitch, y: yaw, q1: theta1, q2: theta2, q3: theta3})
            
            # reference: http://docs.ros.org/hydro/api/hrl_geom/html/namespacehrl__geom_1_1transformations.html#a8ca9b0bdcd7c401ef619c1a644b3203f
            (theta4, theta5, theta6) = tf.transformations.euler_from_matrix(np.array(R3_6_eval).astype(np.float64), axes = 'ryzx')

            theta5 = theta5 - pi/2
            theta6 = theta6 - pi/2
            
            
            count -= 1
            print("IK_server running;  count = ", count)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)



        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
