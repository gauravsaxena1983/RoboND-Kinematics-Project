## Project: Kinematics Pick & Place

[//]: # (Image References)

[joint_digram_from_kr210_urdf_file]: ./misc_images/joint_digram_from_kr210_urdf_file.jpg
[frame]: ./misc_images/frame.jpg
[image3]: ./misc_images/misc2.png

### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is the table of joints coordinates(x, y, z) from KR210.urdf.xacro file.

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

by looking at the joint coordinated from above and the origin frame from below image we got the following DH tabel.

![frame][frame]

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





#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


