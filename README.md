## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/Kuka_config.png
[image2]: ./misc_images/Sperical_Wrist_Location.png
[image3]: ./misc_images/theta1.png
[image4]: ./misc_images/beta1.png
[image5]: ./misc_images/theta2.png
[image6]: ./misc_images/theta3.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an image from the lessons showing the parameters used for creating the DH table.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

** DH Parameters Table **
n | \alpha_{n-1} | a_{n-1} | d_{n} | q_{n}
--- | --- | --- | --- | --- |
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | q2-pi/2
3 | 0 | 1.25 | 0 | q3
4 |  -pi/2 | -0.054 | 1.50 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
7 | 0 | 0 | 0.303 | 0


Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

** individual transformation matrices about each joint **
```python
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

```

** generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose **
```python
	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


** Inverse Position Kinematics **


![alt text][image2]

```python
        #wrist center vector components:
        WC = Matrix([px, py, pz]) - s[d7] * R0_6_eval * Matrix([1, 0, 0])
        (WCx, WCy, WCz) = WC[0], WC[1], WC[2]
```


![alt text][image3]

```python
            # project WC on base plane to get theta1 rotation angle
            theta1 = atan2(WCy, WCx)
```

![alt text][image4] ![alt text][image5] ![alt text][image6]

```python
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
```


** Inverse Orientation Kinematics **

```python
   # total rotation matrix
   R0_6 = R_roll * R_pitch * R_yaw 
        
   # rotation matrix for last 3 joints (spherical wrist) control rotation of EE
   R3_6 = R0_3.T * R0_6
   # reference: http://docs.ros.org/hydro/api/hrl_geom/html/namespacehrl__geom_1_1transformations.html#a8ca9b0bdcd7c401ef619c1a644b3203f
   (theta4, theta5, theta6) = tf.transformations.euler_from_matrix(np.array(R3_6_eval).astype(np.float64), axes = 'ryzx')
   theta5 = theta5 - pi/2
   theta6 = theta6 - pi/2
```




### Project Implementation

#### 1. `IK_server.py` file contains the properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis.  


I used atan2 for angle calcualtions to return the appropriate quadrant of the computed angles. Checked for complex numbers. Had at least 90% success rate but also had some collisions between links 4 and 6. 

 




