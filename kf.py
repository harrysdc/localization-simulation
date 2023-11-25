import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def plot_cov(mean,cov,plot_axes):
    """
    plots the covariance matrix as an ellipsoid at 2*sigma
    """
    lambda_, v = np.linalg.eig(cov)
    lambda_ = np.sqrt(lambda_)

    ell = Ellipse(xy=mean,
              width=lambda_[0]*2, height=lambda_[1]*2,
              angle=np.rad2deg(np.arccos(v[0, 0])))
    #ell.set_facecolor('none')
    ell.set_facecolor((1.0, 1.0, 1.0, 0))
    ell.set_edgecolor((0, 0, 0, 1))
    plot_axes.add_artist(ell)
    plt.scatter(mean[0,0],mean[1,0],c='r',s=5)

def motionModel(v, w, curr_pose, dt=0.1):
    """
    Thrun Prob. Robotics equation 5.13
    curr_pose -- [x y theta] ^ T
    v -- control input linear velocity
    w -- control input angular veloctiy
    """
    # avoid divide by zero
    if w == 0:
        w = 0.0001

    x = curr_pose[0]
    y = curr_pose[1]
    theta = curr_pose[2]

    delta_pose = np.zeros((3,1))
    delta_pose[0] = -v/w*np.sin(theta) + v/w*np.sin(theta+w*dt)
    delta_pose[1] =  v/w*np.cos(theta) - v/w*np.cos(theta+w*dt)
    delta_pose[2] = w*dt

    curr_pose = np.array(curr_pose).reshape((3,1))
    new_pose = curr_pose + delta_pose

    return new_pose

def sensorModel(Q):
    """
    Q -- sensor noise covariance
    TODO: Implement sensor model
    """
    pass

def kalmanFilter(v, w):
    """
    TODO: Implement KF
    v -- control input linear velocity
    w -- control input angular veloctiy
    """
    pass