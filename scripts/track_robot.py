import mujoco
import mujoco.viewer
import pinocchio
import numpy as np
import os
os.environ["MUJOCO_GL"] = "egl"


def load_trajectory(file_path):
    trajectory = []
    with open(file_path, 'r') as f:
        for line in f:
            trajectory.append([float(coord) for coord in line.strip().split(',')])
    return np.array(trajectory)


def compute_task_space_error(desired, current, desired_rotation=None, current_rotation=None):
    
    #
    position_error = desired[:3] - current[:3]
    rotation_error = np.zeros(3)  
    
    if desired_rotation is not None and current_rotation is not None:
        
        rotation_error = np.linalg.norm(desired_rotation - current_rotation)

    return np.hstack([position_error, rotation_error])  

def inverse_kinematics(robot, robot_data, target_position, q_init, max_iter=1000, tolerance=1e-6):
    q = q_init.copy()
    for _ in range(max_iter):


        pinocchio.forwardKinematics(robot, robot_data, q)
        pinocchio.updateFramePlacements(robot, robot_data)
        end_effector_frame = robot.getFrameId("panda_hand")  
        current_pos = robot_data.oMf[end_effector_frame].translation

        error = compute_task_space_error(target_position, current_pos)


        if np.linalg.norm(error) < tolerance:
            break

        jacobian = pinocchio.computeFrameJacobian(robot, robot_data, q, end_effector_frame)

        jacobian_pseudo_inv = np.linalg.pinv(jacobian)
        delta_q = jacobian_pseudo_inv.dot(error)

        q += delta_q

    return q

def main():
    model = mujoco.MjModel.from_xml_path("/home/bingkun/HIWI_Huang/model/franka_emika_panda/apf_panda.xml")  # 替换为你的模型路径
    data = mujoco.MjData(model)


    data.qpos[:] = [0.0] * model.nq 
    qmin = model.jnt_range[:, 0]  
    qmax = model.jnt_range[:, 1] 

    viewer = mujoco.viewer.launch_passive(model, data)

    
    robot = pinocchio.buildModelFromUrdf("/home/bingkun/HIWI_Huang/model/panda.urdf")  
    robot_data = robot.createData()
    q_init = np.zeros(robot.nq)

   
    trajectory = load_trajectory('/home/bingkun/HIWI_Huang/build/trajectory.txt')
    dt = 0.1  

    tracked_positions = []


    for t, desired_pos in enumerate(trajectory):

        qsol = inverse_kinematics(robot, robot_data, desired_pos, q_init)

        data.qpos[:] = qsol

        pinocchio.forwardKinematics(robot, robot_data, data.qpos)
        pinocchio.updateFramePlacements(robot, robot_data)
        end_effector_frame = robot.getFrameId("panda_hand")
        current_pos = robot_data.oMf[end_effector_frame].translation

        print(f"Step: {t}")
        print(f"Desired: {desired_pos}")
        print(f"Current: {current_pos}")
        print(f"Current: {desired_pos-current_pos}")

        mujoco.mj_step(model, data)
        viewer.sync()
        tracked_positions.append(np.copy(current_pos))
        

    np.savetxt("tracked_positions.txt", np.array(tracked_positions), delimiter=",")
    print("output tracked_positions.txt")


if __name__ == "__main__":
    main()
