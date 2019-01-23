## Project: Kinematics Pick & Place

[//]: # (Image References)

[joint_digram_from_kr210_urdf_file]: ./misc_images/joint_digram_from_kr210_urdf_file.jpg
[l01-16-l-denavit-hartenberg-parameter-definitions-01]: ./misc_images/l01-16-l-denavit-hartenberg-parameter-definitions-01.png
[frame]: ./misc_images/frame.jpg
[image3]: ./misc_images/misc2.png
[frame_DH_param]: ./misc_images/frame_DH_param.jpg
[Arm_top_side_view]: ./misc_images/Arm_top_side_view.jpg
[theta2-theta3]: ./misc_images/theta2-theta3.png
[theta2-calculation]: ./misc_images/theta2-calculation.png
[theta3-calculation]: ./misc_images/theta3-calculation.png
[theta4-5-6]: ./misc_images/theta4-5-6.png 

[kuka-drop]: ./misc_images/kuka-drop.png 
[kuka-grasp]: ./misc_images/kuka-grasp.png
[kuka-retrive]: ./misc_images/kuka-retrive.png
[kuka-tobin]: ./misc_images/kuka-tobin.png 


### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


Below is the digram and table of joints coordinates(x, y, z) taken from KR210.urdf.xacro file.

![joint digram][joint_digram_from_kr210_urdf_file]


Join | x | y | z
--- | --- | --- | ---
j0 | 0 | 0 | 0
j1 | 0 | 0 | 0.33
j2 | 0.35 | 0 | 0.42
j3 | 0 | 0 | 1.25
j4 | 0.96 | 0 | -0.054
j5 | 0.54 | 0 | 0
j6 | 0.193 | 0 | 0



As discussed in Lesson 14:12 we will be creating the Denavit-Hartenberg table using the convention described by John J Craig's. 

The parameter names and definitions are summarized as follows:

- alpha(i-1) = angle between Z(i-1) and Z(i) measured about X(i-1) in a right-hand sense.

- a(i−1) (link length) = distance from Z(i-1) to Z(i) measured along X(i−1) where X(i−1) is perpendicular to both Z(i−1) to Z(i)

- d(i)(link offset) = signed distance from X(i−1) to X(i) measured along Z(i). Note that this quantity will be a variable in the case of prismatic joints.

- theta(i)(joint angle) = angle between X(i−1) to X(i) measured about Z(i) in a right-hand sense. Note that this quantity will be a variable in the case of a revolute joint.

![frame DH param][frame_DH_param]

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | q2-pi/2 
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -p1/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

First we define the DH table object in the code from the above table.
```python
DH_Table = { 	alpha0:		0, 		a0:	0, 	d1:	0.75, 	q1:	q1,
		alpha1:		-pi/2.0,	a1:	0.35, 	d2:	0, 	q2:	-pi/2.0 + q2,
		alpha2:		0,		a2:	1.25, 	d3:	0, 	q3:	q3,
		alpha3:		-pi/2.0,	a3:	-0.054,	d4:	1.5, 	q4:	q4,
		alpha4:		pi/2.0,		a4:	0, 	d5:	0, 	q5:	q5,
		alpha5:		-pi/2.0,	a5:	0, 	d6:	0, 	q6:	q6,
		alpha6:		0,		a6:	0, 	d7:	0.303, 	q7:	0}
```

Then after we define the combine rotation matrix as provided in the Lession 14:12
```python
def TF_Matrix(alpha, a, d, q):
	TF = Matrix(	[[	cos(q), 		-sin(q), 		0,		a],
			[	sin(q)*cos(alpha), 	cos(q)*cos(alpha),	-sin(alpha), 	-sin(alpha)*d],
			[	sin(q)*sin(alpha), 	cos(q)*sin(alpha),	cos(alpha), 	cos(alpha)*d],
			[	0,			0, 			0,		1]])
	return TF
```

Then we apply the DH parameters on the DH object to get the individual tranformation matrix.
```python
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```

Once we get all the individual transformation matrix we can compute the generalized homogeneous transform between base_link and gripper_link.
```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE 
```

Now we compute the rotation matrix of EE with error correction. 
By first calculating the indvidual rotation against each axis namly ROT_x, ROT_y and ROT_z. 
Then computing the combined rotation matrix ROT_EE by doing a dot product of ROT_x, ROT_y and ROT_z. 
Later we create a rotation error correction matrix ROT_Error and using this calcualte the correct ROT_EE.
```
#rotation matrix for roll
ROT_x = Matrix([[1,0,0],
	[0, cos(r), -sin(r)],
	[0,sin(r), cos(r)]]) # ROLL

#rotation matrix for pitch
ROT_y = Matrix([[cos(p),0,sin(p)],
	[0, 1, 0],
	[-sin(p), 0, cos(p)]]) # PITCH

#rotation matrix for yaw
ROT_z = Matrix([[cos(y),-sin(y), 0],
	[sin(y), cos(y), 0],
	[0, 0, 1]]) # YAW

#combine rotation matrix of end effactor 
ROT_EE = ROT_z * ROT_y * ROT_x

#rotation error matrix used for correction
ROT_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

#creating a new combine rotation matrix of end effactor with error correction
ROT_EE = ROT_EE * ROT_Error
```

Later we can use the transformation matrix to get the rotation matrix for WC
```
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
```

And to calculate the rotation matrix  we do a dot product of inverse WC rotaion matrix R0_3 and rotation matrix of EE i.e. ROT_EE. 
These matrix will be needed to calculate the theta angles of the joints.
```
R3_6 = R0_3.transpose() * ROT_EE
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

As discussed in Lesson 14:18 we need to use the "analytical" or "closed-form" solution as out robotic arm satisfy the "Three neighboring joint axes intersect at a single point" condition. 

>Closed-form solutions are specific algebraic equation(s) that do not require iteration to solve and have two main advantages: generally they are much faster to solve than numerical approaches and it is easier to develop rules for which of the possible solutions is the appropriate one.

So we can break the solution in to two diffrent part: 
1. The Cartesian coordinates of the wrist center.

Here's is the draw out for the wrist center in form of top view and side view. 

![alt text][Arm_top_side_view]

By looking at the projection in the above image we can calculate the theta angles of the wrist center.  
theta1 = atan2(y, x)

Before calculating theta2 and theta3 we need more details mentioned in the image below
![theta2-theta3]
We need to calculate side A, B and C length shown in the above image.
```
A = sqrt(pow((0.96 + 0.54), 2) + pow(0.054, 2)) = 1.501
B = sqrt(pow(WCx,2) + pow(WCy,2) + pow((WCz-0.75),2)
C = 1.25
```

Now we can caluculate the angles a, b and c using Cosine Laws as stated in the Lesson "Project: Robotic Arm: Pick and Place:15".
```
a = acos((pow(B) + pow(C) - pow(A))/ (2 x B x C))
b = acos((pow(A) + pow(C) - pow(B))/ (2 x A x C))
c = acos((pow(B) + pow(A) - pow(C))/ (2 x B x A))
```

Now we calculate the coordinates of joint 3 shown in the image above 
```
y = WCz - 0.75
x = sqrt(pow(WCx,2) + pow(WCy,2) - 0.35)
```

![theta2-calculation]

From these coordinates we can calcualte theta2 as 
```
theta2 = pi/2 - a - atan2(WCz - 0.75, sqrt(pow(WCx,2) + pow(WCy,2)) -0.35 
```
![theta3-calculation]
As shown in the image above we can formulate the calculation of theta3
```
theta3 = pi/2 - b - atan(0.054, 1.5)
```

2. The composition of rotations to orient the end effector. 

First we get the rotation matrix R0_3 from the transformation matrix and providing the theta1, theta2 and theta3 values.
```
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
```

We can calculate the remaining rotation matrix for 3 to 6 as:
```
R3_6 = R0_3.transpose() * ROT_EE # where ROT_EE is the combined rotation matrix of the end effactor
```

![theta4-5-6]

Now we can use the eular angle equations to get values for theta4, theta5 and theta6. A special case exists for multiple solution when depending on value of theta5. 
```
theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
if theta5 > pi :
	theta4 = atan2(-R3_6[2,2], R3_6[0,2])	    
	theta6 = atan2(R3_6[1,1], -R3_6[1,0])
else:
	theta4 = atan2(R3_6[2,2], -R3_6[0,2])	    
	theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

Screenshot from pickup to drop
1. ![kuka-retrive]
2. ![kuka-grasp]
3. ![kuka-tobin]
4. ![kuka-drop]




