
# Artifical potential fields (APF) +  MuJoCo Simulation
![Demo Video](video/demo.mov)

 ## 1. Using the method APF to create a trajectory for avoiding the obstacle 
 (https://journals.sagepub.com/doi/abs/10.1177/027836498600500106)

<img width="337" alt="image" src="https://github.com/user-attachments/assets/c4acbd23-7137-4f17-ab2d-9a15977ebb6f">
<img width="326" alt="image" src="https://github.com/user-attachments/assets/bf33adc9-6dcf-4351-9ea0-edfb78b5710e">

#### In `src/apf_trajectory.cpp`, we implement the APF method:  

(1) Given the start point, target point, and spherical obstacles (with radii)

(2) we calculate attractive and repulsive forces. Combining them gives the trajectory points.

(3) We export the generated trajectory to `build/trajectory.txt` for visualization in MuJoCo.

However, by adjusting the constants in the formulas, we can get different trajectory results — some good, some bad.

<img width="513" alt="image" src="https://github.com/user-attachments/assets/5355422b-bc3b-49ab-a63d-c5ed3dd490a7">

For example: Green Ball is Initial Point, Red ball is Goal Point

<img width="728" alt="image" src="https://github.com/user-attachments/assets/7ecd0b19-6d0a-4d67-b793-9ad5efa241e1">

Of course, the radius of the obstacles also has a significant impact on the trajectory. I think: For each case, what these parameters are may require further calculations

## 2. Visualisation in a simulated environment MuJoCo

In `src/traj_to_XML.cpp`, we convert the generated trajectory into an `XML` file for visualization and integrate it with the `Franka Emika Panda` [robot](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda). This makes the robotic arm's future movements clearer.

<img width="424" alt="image" src="https://github.com/user-attachments/assets/665ffaf4-54c5-434d-8905-c6af7d4d912c">
->
<img width="448" alt="image" src="https://github.com/user-attachments/assets/92d1ddd7-449e-42d2-ad3f-21c6a9c35a31">



## 3. Tracking the previous generating trajectory using the following control methods (Joint positions by integrating the velocities)

#### In `scripts/diffik_nullspace.py`, we import the `XML` model from the previous step to make the robot move along the generated trajectory.

(1) We use the `Pinocchio Robotics library` to compute forward kinematics and the Jacobian matrix. For this step w have to use `Panda' [URDF](https://github.com/StanfordASL/PandaRobot.jl) and give the urdf to the Pinocchio for initializing kinematic model (Jacobian)

(2) We use the following control law: [Control](https://github.com/kevinzakka/mjctrl/blob/main/README.md)

<img width="593" alt="image" src="https://github.com/user-attachments/assets/afc44c9b-881e-471b-90f6-46fd1e79d089">

We import the control input command and use this method to obtain the robot arm's `joint velocities`, and then get the `joint positions` by integrating the velocities. 

(3) Then, we solve the forward kinematics problem using `Pinocchio` and update the current position in `MuJoCo`. 

Using the output trajectorys (desired and actual) we can see the performance of the system.

<img width="375" alt="image" src="https://github.com/user-attachments/assets/96546975-713a-499e-90fe-a6f98740ec60">

## 4. Some Aussumptions 

(1) Although the trajectory obtained using APF allows the robot arm's end effector to avoid obstacles, in some cases, other parts of the robot may still collide. To address this issue, we can use solutions like collision avoidance in `MoveIt` to prevent collisions between different parts of the robot. 

(2) In addition to this control method, I also tried using the `KDL solver` in `scripts/track_robot.py` to compute the inverse kinematics results and simulate them in MuJoCo. However, we were unable to control the orientation. So in this method the movement is not smoothly.

![Demo Video](images/KDL.gif)

(3) If our desired trajectory is outside the robot arm's reachable workspace, an error will occur. Therefore, we need to ensure that the target point is within the reachable workspace of the robot.

<img width="308" alt="image" src="https://github.com/user-attachments/assets/bf39064b-1605-4b7f-bdef-20d7b084c03d">


