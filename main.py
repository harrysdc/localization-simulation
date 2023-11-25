import numpy as np
import matplotlib.pyplot as plt
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
from data import generateControls, generatePath, generateSensorData
from kf import kalmanFilter, motionModel
from pf import particleFilter


def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)

    # load robot and obstacle resources
    robots, obstacles = load_env('twotables.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)


    v, w = generateControls()
    path = generatePath(v, w, start_config)
    Q = np.array([
        [0.1, 0.0, 0.0],
        [0.0, 0.1, 0.0], 
        [0.0, 0.0, 0.1]
        ])
    sensor_data = generateSensorData(path, Q)



    """
    TODO: Call KF, PF to estimate robot pose
    TODO: Check collision of estiamte pose using collision_fn
    TODO: Compute psoe error between gt and estimation
    TODO: Tune sensor noise covariance Q
    """
    


    gt_x = [x for x, y, theta in path]
    gt_y = [y for x, y, theta in path]
    sd_x = [x for x, y, theta in sensor_data]
    sd_y = [y for x, y, theta in sensor_data]
    plt.scatter(gt_x, gt_y, s=2, label='ground truth')
    plt.scatter(sd_x, sd_y, s=2, label='sensor data')
    plt.title('KF Pose estimation')
    plt.legend()
    plt.pause(0.001)
    plt.ioff()
    plt.show()



    # execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # keep graphics window opened
    wait_if_gui()
    disconnect()




if __name__ == '__main__':
    main()