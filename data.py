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
        noisy_x = gt[0] + random.uniform(-Q[0,0], Q[0,0])
        noisy_y = gt[1] + random.uniform(-Q[1,1], Q[1,1])
        noisy_theta = gt[2] + random.uniform(-Q[2,2], Q[2,2])
        sensor_data.append((noisy_x, noisy_y, noisy_theta))
    
    return sensor_data