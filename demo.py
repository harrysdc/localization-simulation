import numpy as np
import matplotlib.pyplot as plt
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
from data import generateControls, generatePath, generateSensorData
from kf import kalmanFilter, motionModel, plot_cov
from pf import ParticleFilter


def demoKF(screenshot=False):
    connect(use_gui=True)
    robots, obstacles = load_env('twotables.json')
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)

    v, w = generateControls()
    path = generatePath(v, w, start_config)
    Q = np.identity(3) * 0.02
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # number of data points
    N = 720 
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = np.array(sensor_data[0]).transpose()
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.eye(3) #3x3
    R = np.identity(3) * 0.001

    for i in range(1,N):
        z = np.matrix(sensor_data[i]).transpose()
        u = np.array([[v[i]*np.cos(z[2,0])*0.1],[v[i]*np.sin(z[2,0])*0.1],[0]])
        mu, Sigma = kalmanFilter(np.reshape(mu, (3, 1)), Sigma, z, u, Q, R)
        estimated_states[:,i] = np.squeeze(mu)

    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print("Total Error: %f"%total_error)

    plt.plot(estimated_states[0,0:N], estimated_states[1,0:N],'r',linewidth=1.0, label='KF estimate')
    gt_x = [x for x, y, theta in path]
    gt_y = [y for x, y, theta in path]
    sd_x = [x for x, y, theta in sensor_data]
    sd_y = [y for x, y, theta in sensor_data]
    plt.scatter(gt_x, gt_y, s=2, label='ground truth')
    plt.scatter(sd_x, sd_y, s=2, label='sensor data', color = 'gray')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('KF Pose estimation')
    plt.legend()
    plt.pause(0.001)
    plt.ioff()
    plt.show()

    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    wait_if_gui()
    disconnect()


def demoPF():
    connect(use_gui=True)
    robots, obstacles = load_env('twotables.json')
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)

    v, w = generateControls()
    path = generatePath(v, w, start_config)
    Q = np.identity(3) * 0.02
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # initialize mean and covariance of state estimate gaussian
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.eye(3)
    # number of data points
    N = 720
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = np.array(sensor_data[0]).transpose()
    R = np.identity(3) * 0.001

    noise = np.identity(3) * 0.001
    pf = ParticleFilter(np.array([sensor_data[0][0], sensor_data[0][1], sensor_data[0][2]]), noise)
    for i in range(1,N):
        z = np.matrix(sensor_data[i]).transpose()
        mu = pf.particleFilter(v[i], w[i], z)
        estimated_states[:,i] = np.squeeze(mu)

    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print("Total Error: %f"%total_error)

    plt.plot(estimated_states[0,0:N], estimated_states[1,0:N],'r', label='PF estimate')
    gt_x = [x for x, y, theta in path]
    gt_y = [y for x, y, theta in path]
    sd_x = [x for x, y, theta in sensor_data]
    sd_y = [y for x, y, theta in sensor_data]
    plt.scatter(gt_x, gt_y, s=2, label='ground truth')
    plt.scatter(sd_x, sd_y, s=2, label='sensor data', color = 'gray')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('PF Pose estimation')
    plt.legend()
    plt.pause(0.001)
    plt.ioff()
    plt.show()
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    wait_if_gui()
    disconnect()


if __name__ == '__main__':
    demoPF()
