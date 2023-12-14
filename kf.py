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
    delta_pose[0] = v*np.cos(theta)*dt #-v/w*np.sin(theta) + v/w*np.sin(theta+w*dt)
    delta_pose[1] = v*np.sin(theta)*dt #v/w*np.cos(theta) - v/w*np.cos(theta+w*dt)
    delta_pose[2] = w*dt

    curr_pose = np.array(curr_pose).reshape((3,1))
    new_pose = curr_pose + delta_pose

    return new_pose

def sensorModel(mu_bar):
    """
    mu_bar -- predicted state 3x1
    TODO: Implement sensor model
    sensor reports noisy state estimate
    """
    x = mu_bar[0]
    y = mu_bar[1]
    theta = mu_bar[2]

    Q = np.zeros((3,1)) #NOT SURE IF THIS IS A GOOD MODEL
    Q[0] = 0.8 * x
    Q[1] = 0.8 * y
    Q[2] = 0.8 * theta
    return Q

def kalmanFilter(mu, Sigma, z, u, Q, R, dt = 0.1):
    A = np.identity(3)
    B = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 0]])
    C = np.identity(3)

    #prediction step
    mu_bar = np.dot(A, mu) + np.dot(B, u)
    Sigma_bar = np.dot(A, np.dot(Sigma, np.transpose(A))) + R
    #correction step
    K = np.dot(Sigma_bar, np.dot(np.transpose(C), np.linalg.inv(np.dot(C, np.dot(Sigma_bar, np.transpose(C))) + Q)))
    mu_new = mu_bar + np.dot(K, (z - np.dot(C, mu_bar)))
    # print(mu_bar)
    Sigma_new = np.dot((np.identity(3) - np.dot(K, C)), Sigma_bar)
    ###YOUR CODE HERE###
    return mu_new, Sigma_new

# def jacobian_g(control, mu):
#     '''
#     returns jacobian of nonlinear dynamics function
#     control -- 2x1 vector with v and w
#     mu -- current mean of state
#     '''
#     J = np.identity(3) # identity matrix 3x3
#     return J

# def jacobian_h(mu_bar, dt=0.1): 
#     # returns jacobian of sensor nonlinear equation (though this implementation is linear)
#     J = np.identity(3)
#     J[0,0] = 0.8
#     J[1,1] = 0.8
#     J[2,2] = 0.8
#     return J 

# def extendedKalmanFilter(x_prev, v, w, mu, Sigma, R, Q):
#     """
#     TODO: Implement KF
#     x_prev -- current state (from sensor)
#     v -- control input linear velocity
#     w -- control input angular velocity
#     mu -- current mean of state
#     Sigma -- current covariance of state
#     R -- noise covariance
#     Q -- noise covariance
#     """
#     #g for dynamics function given state and control
#     #h for sensor measurement at state

#     #prediction step
#     control = np.reshape(np.array([v[0,0], w[0,0]]), (2,1)) #2x1
#     G = jacobian_g(control, mu) #3x3, z is 3x1 THIS IS IDENTITY RN! need to recompute for every step
#     mu_bar = motionModel(v, w, mu) + np.dot(G, (x_prev - np.reshape(mu, (3, 1)))) #np.dot(G, z) + np.dot(jacobian_h(z), control) #3x1 ADD PROCESS NOISE
#     x = mu_bar
#     Sigma_bar = np.dot(G, np.dot(Sigma, np.transpose(G))) + R

#     #correction step
#     H = jacobian_h(mu_bar) #takes in curr_pose
#     K = np.dot(Sigma_bar, np.dot(np.transpose(H), np.linalg.pinv(np.dot(H, np.dot(Sigma_bar, np.transpose(H))) + Q))) #3x3, pinv or inv?
#     mu_new = mu_bar + np.dot(K, ((sensorModel(mu_bar) + np.dot(H, (x - mu_bar))) - sensorModel(mu_bar))) 
#     Sigma_new = np.dot((np.identity(3) - np.dot(K, H)), Sigma_bar)

#     return mu_new, Sigma_new
