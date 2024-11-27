# APF_Robot_Control

In summary, we have two tasks：(visualisation in a simulated environment MuJoCo)
1. Using the method APF to create a trajectory for avoiding the obstacle (https://journals.sagepub.com/doi/abs/10.1177/027836498600500106)

<img width="337" alt="image" src="https://github.com/user-attachments/assets/c4acbd23-7137-4f17-ab2d-9a15977ebb6f">
<img width="326" alt="image" src="https://github.com/user-attachments/assets/bf33adc9-6dcf-4351-9ea0-edfb78b5710e">

In `src/apf_trajectory.cpp`, we implement the APF method:  
Given the start point, target point, and spherical obstacles (with radii), we calculate attractive and repulsive forces. Combining them gives the trajectory points.We export the generated trajectory to `build/trajectory.txt` for visualization in MuJoCo.

However, by adjusting the constants in the formulas, we can get different trajectory results — some good, some bad.

<img width="513" alt="image" src="https://github.com/user-attachments/assets/5355422b-bc3b-49ab-a63d-c5ed3dd490a7">

<img width="728" alt="image" src="https://github.com/user-attachments/assets/7ecd0b19-6d0a-4d67-b793-9ad5efa241e1">

Of course, the radius of the obstacles also has a significant impact on the trajectory. I think: For each case, what these parameters are may require further calculations



2. Tracking the previous generating trajectory using the following control methods (Joint positions by integrating the velocities)
<img width="593" alt="image" src="https://github.com/user-attachments/assets/afc44c9b-881e-471b-90f6-46fd1e79d089">

