## Project: Kinematics Pick & Place

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Values getting from URDF file.

![alt text][URDF]

Based on previous course analysis, configuration of joins shows below. 
To make things easier, let the origin of O4, O5, O6 to be coincided at the origin of joint 5 as the wrist center(WC). 
Note: the `end-effector` is represented as the "E" in my annotated figure, but in code I use "7".

![alt text][annotated]

Based on the values from URDF file and the anyalsis of joins configuration, I can get the D-H parameter table below.

![alt text][DH parameter table]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

From the lecture, I learned the homogeneous transform from frame i-1 to frame i can be expressed as below.

![alt text][homogenous transform expression]

So, the individual transform matrices about each joint can be obtained by using the values from DH table.

![alt text][homogenous transforms]

![alt text][homogenous transforms2]

Finally a homogeneous transform matrix from base_link to gripper_link can be obtained by using only the position and orientation of the gripper_link.

![alt text][final]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Step1：Calculate WC position informantion by using the function below

![alt text][step1]

Step2: Solve theta1, theta2, and theta3 by using WC position information

First, calculate theta1, which is pretty straight forward

![alt text][theta1]

Next, calculate theta2 and theta3 by using Cosine Laws

![alt text][theta figure]

![alt text][theta analysis]

Step2: Solve theta4, theta5, and theta6 by using WC pitch, roll, and yaw information

![alt text][theta4-6]

Using individual rotations production to get the overall RPY rotation between link 3 and link 5.

![alt text][R3-6]

![alt text][calculate-theta4-6]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

- px, py, pz and roll, pitch, yaw are end-effector position and orientation values given from URDF/Gazebo rather than DH parameters. So compensate for rotation discrepancy between DH parameters and Gazebo is needed.
- We use px, py, pz to calculate wcx, wcy, wcz
- Then use wcx, wcy, wcz to calculate theta1, theta2, and theta3
- Finally use theta1, theta2, and theta3 to calculate theta4, theta5, and theta6

![alt text][code]

Results:
During testing, I have one failed experience for grabing object because when the arm arrived at the grabing position and try to grab the object, the ending effector change the position of object slightly. So the arm moving to garbage bin without target, which results in the task failed.

My final result is 10/11 success.

![alt text][results1]

![alt text][results2]

My guess the reason of arm "hitting" object is that Moveit! using RRT algorism which cause minor "accident". It might cannot generate smoooth path, and the path does not include orientation information. These reasons may lead to the movement of arm are not smooth. And even though all points of path will not tauch the object, the gaps between two poins might have collision with the target.

![alt text][rrt]


