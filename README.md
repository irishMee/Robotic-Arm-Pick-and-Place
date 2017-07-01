# Robotic-Arm-Pick-and-Place
## Forward Kinematics Analysis

1. Here is an image from my notebook of the Kuka-arm with reference frames.

    ![alt text](https://github.com/irishMee/Robotic-Arm-Pick-and-Place/blob/master/DH_table_reference_frames.jpg)
  
2. The forward kinematic analysis is shown in my jupyter file named Forward and Inverse Kinematic Analysis. 
3. I used the values from the URDF file in order to fill out the twist angles and the a and d offset values as shown below.
    ```python
        ###DH Parameter Table
        #I need to set up the DH parameter table from the Kuka arm after setting 
        #reference frames. 
        #The values for a's and d's were found from the urdf xacro file.

        s = {alpha0:     0, a0:      0, d1:  0.75,
             alpha1: -pi/2, a1:   0.35, d2:     0,   q2: q2-pi/2,
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:  1.50,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303,   q7: 0
            }
     ```
 4. The individual transform matrices from the base_link to link 7 which is the gripper_link are shown below.
     ```python
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
     ```
5. The total homogeneous transform matrix is derived by multiplying each individual transform matrix from base to gripper as shown          below.
    ```python 
        #Composition of Homogeneous Transforms from base to gripper
        T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
        T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
        T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
        T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
        T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
        T0_G = simplify(T0_6 * T6_G) # base_link to link_G(which is the gripper)
    ```
 6. The final homogeneous matrix prints in an element list to look like the following.
    ![alt text](https://github.com/irishMee/Robotic-Arm-Pick-and-Place/blob/master/Final_homogeneous_matrix.jpg)
  
## Inverse Kinematic Analysis

1. The inverse kinematic solutions can be found in the IK_server.py file.
2. We had to use inverse kinematics in order to find formulas for the six joint variables.
3. In order to find the first three angles, we have to find the cartesian coordinates of the wrist center. Below is the code for solving
   for the wrist center.
   ```python
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
     ```
4. The first joint angle followed the example given during the lessons, but the second and third angles needed some extra attention.
5. Here is an image from my notebook of the set up of the second and third revolute joints.
    ![alt text](https://github.com/irishMee/Robotic-Arm-Pick-and-Place/blob/master/joint2_joint3.jpg)
6. This allowed me to use the law of cosines to find the angle for joint 3. Joint 3 had a +- solution for the x component because of        of being similar to an elbow joint. I choose the positive angle because when running the demo arm, it seemed that the robot made
   most movements in positive x direction. Also, I was able to use this diagram to find the length
   of the arms.
7. I was able to find the final joint variables (4,5,6) by using Euler's Methods. These formulas are shown in the code below.
    ```python
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
    ```
## Final Thoughts

When the Kuka arm was placed in test mode, it was able to successfully pick up and place the blue object. However, I noticed that the       arm seemed to move at wider angles as compared to the demo. I believe that it would be necessary for me to put more constraints on       the angles so that in the real world the robot does not swing too wide. 



