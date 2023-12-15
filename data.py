import numpy as np
import random
from kf import motionModel

def generateControls():
    """
    Generate two list of control inputs v (velocity) and w (angular velocity) for the PR2 robot to move around the tables
    TODO: tune steps
    """

    # turn L
    v1 = [0.05 for i in range(40)]
    w1 = [0.4 for i in range(40)]
    # forward
    v2 = [0.2 for i in range(110)]
    w2 = [0.0 for i in range(110)]
    # turn R
    v3 = [0.05 for i in range(40)]
    w3 = [-0.4 for i in range(40)]
    # forward
    v4 = [0.3 for i in range(105)]
    w4 = [0.0 for i in range(105)]
    # turn R
    v5 = [0.05 for i in range(41)]
    w5 = [-0.4 for i in range(41)]
    # forward
    v6 = [0.2 for i in range(100)]
    w6 = [0.0 for i in range(100)]
    
    # turn L
    v7 = [0.05 for i in range(43)]
    w7 = [0.4 for i in range(43)]
    # forward
    v8 =  [0.2 for i in range(90)]
    w8 = [0.0 for i in range(90)]
    # turn L
    v9 =  [0.05 for i in range(40)]
    w9 =  [0.4 for i in range(40)]
    # forward
    v10 = [0.2 for i in range(120)]
    w10 = [0.0 for i in range(120)]

    v = v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8 + v9 + v10
    w = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10
    with open('test_path.txt', 'w') as file:
        v_str = ', '.join(map(str, v))
        w_str = ', '.join(map(str, w))
        file.write('[' + v_str + ']\n')
        file.write('[' + w_str + ']\n')
    
    return v, w


def generateControls_3():
    """
    Generate two list of control inputs v (velocity) and w (angular velocity) for the PR2 robot to move around the tables
    TODO: tune steps
    """

    # forward
    v1 = [0.2 for i in range(20)]
    w1 = [0.0 for i in range(20)]

    # backward
    v2 = [-0.2 for i in range(20)]
    w2 = [0.0 for i in range(20)]

    # forward
    v3 = [0.2 for i in range(20)]
    w3 = [0.0 for i in range(20)]

    # backward
    v4 = [-0.2 for i in range(20)]
    w4 = [0.0 for i in range(20)]

    # forward
    v5 = [0.2 for i in range(20)]
    w5 = [0.0 for i in range(20)]

    # backward
    v6 = [-0.2 for i in range(20)]
    w6 = [0.0 for i in range(20)]

    # forward
    v7 = [0.2 for i in range(20)]
    w7 = [0.0 for i in range(20)]

    # backward
    v8 = [-0.2 for i in range(20)]
    w8 = [0.0 for i in range(20)]

    # forward
    v9 = [0.2 for i in range(20)]
    w9 = [0.0 for i in range(20)]

    # backward
    v10 = [-0.2 for i in range(20)]
    w10 = [0.0 for i in range(20)]

    v = v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8 + v9 + v10
    w = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10
    with open('test_path.txt', 'w') as file:
        v_str = ', '.join(map(str, v))
        w_str = ', '.join(map(str, w))
        file.write('[' + v_str + ']\n')
        file.write('[' + w_str + ']\n')
    
    return v, w

def generateControls_zigzag():
    """
    Generate two list of control inputs v (velocity) and w (angular velocity) for the PR2 robot to move around the tables
    TODO: tune steps
    """

    # forward
    v1 = [0.25 for i in range(50)]
    w1 = [0.0 for i in range(50)]
    
    # turn L
    v2 = [0.0 for i in range(25)]
    w2 = [0.75 for i in range(25)]
    
    # forward
    v3 = [0.25 for i in range(50)]
    w3 = [0.0 for i in range(50)]

    # turn R
    v4 = [0.0 for i in range(25)]
    w4 = [-0.75 for i in range(25)]
    
    # forward
    v5 = [0.25 for i in range(50)]
    w5 = [0.0 for i in range(50)]

    # turn L
    v6 = [0.0 for i in range(25)]
    w6 = [0.75 for i in range(25)]
    
    # forward
    v7 = [0.25 for i in range(50)]
    w7 = [0.0 for i in range(50)]

    # turn R
    v8 = [0.0 for i in range(25)]
    w8 = [-0.75 for i in range(25)]
    
    # forward
    v9 = [0.25 for i in range(50)]
    w9 = [0.0 for i in range(50)]

    # turn L
    v10 = [0.0 for i in range(25)]
    w10 = [0.75 for i in range(25)]
    
    # forward
    v11 = [0.25 for i in range(50)]
    w11 = [0.0 for i in range(50)]

    # turn R
    v12 = [0.0 for i in range(25)]
    w12 = [-0.75 for i in range(25)]
    
    # forward
    v13 = [0.25 for i in range(50)]
    w13 = [0.0 for i in range(50)]

    # turn L
    v14 = [0.0 for i in range(25)]
    w14 = [0.75 for i in range(25)]
    
    # forward
    v15 = [0.25 for i in range(50)]
    w15 = [0.0 for i in range(50)]

    # turn R
    v16 = [0.0 for i in range(25)]
    w16 = [-0.75 for i in range(25)]
    
    # forward
    v17 = [0.25 for i in range(50)]
    w17 = [0.0 for i in range(50)]

    v = v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8 + v9 + v10 +v11 +v12 +v13 +v14 + v15 + v16 + v17
    w = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 +w11 +w12 +w13 +w14 + w15 + w16 + w17
    with open('test_path.txt', 'w') as file:
        v_str = ', '.join(map(str, v))
        w_str = ', '.join(map(str, w))
        file.write('[' + v_str + ']\n')
        file.write('[' + w_str + ']\n')
    
    return v, w

def generatePath(v, w, start_pose):
    """
    Use motion model to generate ground truth path (a list of poses)
    v -- control input linear velocity
    w -- control input angular veloctiy
    """

    path = []
    path.append(start_pose)
    
    curr_pose = start_pose
    for i in range(len(v)):
        new_pose = motionModel(v[i], w[i], curr_pose)
        path.append(tuple(np.squeeze(new_pose)))
        curr_pose = new_pose

    return path

def generateSensorData(path, Q):
    """
    Simulate a location sensor by adding noise to the ground truth poses
    path -- a list of ground truth poses
    Q -- noise covariance 3x3
    """

    sensor_data = []
    for i in range(len(path)):
        gt = path[i]
        noisy_x = gt[0] + random.gauss(0, Q[0,0])
        noisy_y = gt[1] + random.gauss(0, Q[1,1])
        noisy_theta = gt[2] + random.gauss(0, Q[2,2])
        sensor_data.append((noisy_x, noisy_y, noisy_theta))
    
    return sensor_data


def generatePFSensorData(input, prev_pose, sensor_cov=np.eye(3), dt=0.1):
#def generatePFSensorData(input, prev_pose, sensor_cov=np.eye(3)*0.02, dt=0.1):
    v = input[0]
    w = input[1]
    delta_x = v*dt*np.cos(prev_pose[2] + w*dt)
    delta_y = v*dt*np.sin(prev_pose[2] + w*dt)
    delta_theta = w * dt
    delta_pose = np.array([delta_x, delta_y, delta_theta])
    curr_pose = prev_pose + delta_pose

    new_pose = np.random.multivariate_normal(curr_pose, sensor_cov)
    return new_pose, curr_pose


def odometry(input, prev_pose, C=0.5, dt=0.1):
    v = input[0]
    w = input[1]

    R = np.array([[(C* abs(v)+C*abs(w))**2, 0], [0, (C* abs(v)+C*abs(w))**2]])
    rand = np.random.multivariate_normal(input, R)
    v_actual = rand[0]
    w_actual = rand[1]

    delta_x = v_actual*dt*np.cos(prev_pose[2] + w_actual*dt)
    delta_y = v_actual*dt*np.sin(prev_pose[2] + w_actual*dt)
    delta_theta = w_actual * dt
    delta_pose = np.array([delta_x, delta_y, delta_theta])
    new_pose = prev_pose + delta_pose
    return new_pose
