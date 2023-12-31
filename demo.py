import numpy as np
import matplotlib.pyplot as plt
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
from data import generateControls, generateControls_3, generateControls_zigzag, generatePath, generateSensorData, generatePFSensorData, odometry
from kf import kalmanFilter, motionModel, plot_cov
from pf import PF

def demoKF_lownoise(screenshot=False):
    connect(use_gui=True)
    robots, obstacles = load_env('twotables.json')
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)

    v, w = generateControls()
    path = generatePath(v, w, start_config)
    Q = np.eye(3) * 0.02
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # number of data points
    N = len(v)
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = np.array(sensor_data[0]).transpose()
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.identity(3) #3x3
    R = np.identity(3) * 0.001

    points_collision = []
    for i in range(1,N):
        z = np.matrix(sensor_data[i]).transpose()
        u = np.array([[v[i]*np.cos(z[2,0])*0.1],[v[i]*np.sin(z[2,0])*0.1],[0]])
        mu, Sigma = kalmanFilter(np.reshape(mu, (3, 1)), Sigma, z, u, Q, R)
        # if collision_fn((mu[0], mu[1], mu[2])):
        #     print("Cannot Estimate -- mean in collision, iteration: ", i)
        #     N = i
        #     break
        estimated_states[:,i] = np.squeeze(mu)
        if collision_fn((mu[0], mu[1], mu[2])):
            points_collision.append((mu[0], mu[1], mu[2]))

    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error = np.sum(np.linalg.norm(state_errors, axis=0))
    print("KF Total Path Error: %f"%total_error)

    plt.plot(estimated_states[0,0:N], estimated_states[1,0:N],'r',linewidth=1.0, label='KF estimate')
    cp_x = [x for x, y, theta in points_collision]
    cp_y = [y for x, y, theta in points_collision]
    # plt.scatter(cp_x, cp_y, s=50, alpha=0.5, label='points in obstacles', color = 'yellow')

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

    #execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    wait_if_gui()
    disconnect()


def demoPF_lownoise():
    connect(use_gui=True)
    robots, obstacles = load_env('twotables.json')
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)

    v, w = generateControls()
    path = generatePath(v, w, start_config)
    Q = np.eye(3) * 0.02
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # initialize state
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.eye(3) * 0.001
    # number of data points
    N = len(v)
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = path[0]
    points_collision = []


    pf = PF(path[0])
    prev_odom = path[0]
    prev_gt_pose = path[0]
    for i in range(1, N):
        input = [v[i], w[i]]
        curr_odom = odometry(input, prev_odom)
        curr_sensor_data, curr_gt_pose = generatePFSensorData(input, prev_gt_pose)
        
        pf.action_model(curr_odom)
        pf.update_weight(curr_sensor_data)
        pf.resample()
        pf.posteriorPose()
        estimated_states[:,i] = pf.state
        state_errors = curr_gt_pose - pf.state
        prev_odom = curr_odom
        prev_gt_pose = curr_gt_pose

        if collision_fn(tuple(pf.state)):
            points_collision.append(tuple(pf.state))


    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print("PF Total Path Error: %f"%total_error)

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
    #execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    wait_if_gui()
    disconnect()

def demoKF(screenshot=False):
    connect(use_gui=True)
    robots, obstacles = load_env('twotables.json')
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)

    v, w = generateControls()
    path = generatePath(v, w, start_config)
    Q = np.eye(3) * 1.0
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # number of data points
    N = len(v)
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = np.array(sensor_data[0]).transpose()
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.identity(3) #3x3
    R = np.identity(3) * 0.001

    points_collision = []
    for i in range(1,N):
        z = np.matrix(sensor_data[i]).transpose()
        u = np.array([[v[i]*np.cos(z[2,0])*0.1],[v[i]*np.sin(z[2,0])*0.1],[0]])
        mu, Sigma = kalmanFilter(np.reshape(mu, (3, 1)), Sigma, z, u, Q, R)
        # if collision_fn((mu[0], mu[1], mu[2])):
        #     print("Cannot Estimate -- mean in collision, iteration: ", i)
        #     N = i
        #     break
        estimated_states[:,i] = np.squeeze(mu)
        if collision_fn((mu[0], mu[1], mu[2])):
            points_collision.append((mu[0], mu[1], mu[2]))

    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error = np.sum(np.linalg.norm(state_errors, axis=0))
    print("KF Total Path Error: %f"%total_error)

    plt.plot(estimated_states[0,0:N], estimated_states[1,0:N],'r',linewidth=1.0, label='KF estimate')
    cp_x = [x for x, y, theta in points_collision]
    cp_y = [y for x, y, theta in points_collision]
    # plt.scatter(cp_x, cp_y, s=50, alpha=0.5, label='points in obstacles', color = 'yellow')

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

    #execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
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
    Q = np.eye(3) * 1.0
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # initialize state
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.eye(3) * 0.001
    # number of data points
    N = len(v)
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = path[0]
    points_collision = []


    pf = PF(path[0])
    prev_odom = path[0]
    prev_gt_pose = path[0]
    for i in range(1, N):
        input = [v[i], w[i]]
        curr_odom = odometry(input, prev_odom)
        curr_sensor_data, curr_gt_pose = generatePFSensorData(input, prev_gt_pose)
        
        pf.action_model(curr_odom)
        pf.update_weight(curr_sensor_data)
        pf.resample()
        pf.posteriorPose()
        estimated_states[:,i] = pf.state
        state_errors = curr_gt_pose - pf.state
        prev_odom = curr_odom
        prev_gt_pose = curr_gt_pose

        if collision_fn(tuple(pf.state)):
            points_collision.append(tuple(pf.state))


    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print("PF Total Path Error: %f"%total_error)

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
    #execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    wait_if_gui()
    disconnect()

def demoKF_zigzag(screenshot=False):
    connect(use_gui=True)
    robots, obstacles = load_env('twotables.json')
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)

    v, w = generateControls_zigzag()
    path = generatePath(v, w, start_config)
    Q = np.eye(3) * 1.0
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # number of data points
    N = len(v)
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = np.array(sensor_data[0]).transpose()
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.identity(3) #3x3
    R = np.identity(3) * 0.001

    points_collision = []
    for i in range(1,N):
        z = np.matrix(sensor_data[i]).transpose()
        u = np.array([[v[i]*np.cos(z[2,0])*0.1],[v[i]*np.sin(z[2,0])*0.1],[0]])
        mu, Sigma = kalmanFilter(np.reshape(mu, (3, 1)), Sigma, z, u, Q, R)
        # if collision_fn((mu[0], mu[1], mu[2])):
        #     print("Cannot Estimate -- mean in collision, iteration: ", i)
        #     N = i
        #     break
        estimated_states[:,i] = np.squeeze(mu)
        if collision_fn((mu[0], mu[1], mu[2])):
            points_collision.append((mu[0], mu[1], mu[2]))

    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error = np.sum(np.linalg.norm(state_errors, axis=0))
    print("KF Total Path Error: %f"%total_error)

    plt.plot(estimated_states[0,0:N], estimated_states[1,0:N],'r',linewidth=1.0, label='KF estimate')
    cp_x = [x for x, y, theta in points_collision]
    cp_y = [y for x, y, theta in points_collision]
    # plt.scatter(cp_x, cp_y, s=50, alpha=0.5, label='points in obstacles', color = 'yellow')

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

    #execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    wait_if_gui()
    disconnect()


def demoPF_zigzag():
    connect(use_gui=True)
    robots, obstacles = load_env('twotables.json')
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints)) # (-3.4, -1.4, 0.0)
    goal_config = (2.6, -1.3, -np.pi/2)

    v, w = generateControls_zigzag()
    path = generatePath(v, w, start_config)
    Q = np.eye(3) * 1.0
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    # initialize state
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.eye(3) * 0.001
    # number of data points
    N = len(v)
    estimated_states = np.zeros((3,N))
    estimated_states[:,0] = path[0]
    points_collision = []


    pf = PF(path[0])
    prev_odom = path[0]
    prev_gt_pose = path[0]
    for i in range(1, N):
        input = [v[i], w[i]]
        curr_odom = odometry(input, prev_odom)
        curr_sensor_data, curr_gt_pose = generatePFSensorData(input, prev_gt_pose)
        
        pf.action_model(curr_odom)
        pf.update_weight(curr_sensor_data)
        pf.resample()
        pf.posteriorPose()
        estimated_states[:,i] = pf.state
        state_errors = curr_gt_pose - pf.state
        prev_odom = curr_odom
        prev_gt_pose = curr_gt_pose

        if collision_fn(tuple(pf.state)):
            points_collision.append(tuple(pf.state))


    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print("PF Total Path Error: %f"%total_error)

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
    #execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    wait_if_gui()
    disconnect()


if __name__ == '__main__':
    print("Expected runtime: < 5 mins")
    print("Demo KF and PF estimation")
    print("Sensor noise level: high (val=1.0)")
    print("Please close the plot and press enter if it pauses")
    demoKF_lownoise()
    demoPF_lownoise()
    demoKF()
    demoPF()
    demoKF_zigzag()
    demoPF_zigzag()
