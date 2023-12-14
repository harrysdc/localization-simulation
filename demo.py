import numpy as np
import matplotlib.pyplot as plt
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
from data import generateControls, generatePath, generateSensorData
from kf import kalmanFilter, motionModel, plot_cov
from pf import ParticleFilter


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
        [0.05, 0.0, 0.0],
        [0.0, 0.05, 0.0], 
        [0.0, 0.0, 0.05]
        ])
    sensor_data = generateSensorData(path, Q)

    plt.ion()
    plot_axes = plt.subplot(111, aspect='equal')   

    """
    TODO: Call KF, PF to estimate robot pose
    TODO: Check collision of estiamte pose using collision_fn
    TODO: Compute psoe error between gt and estimation
    TODO: Tune sensor noise covariance Q
    TODO: plot estimated poses
    """
    # initialize mean and covariance of state estimate gaussian
    mu = np.array(sensor_data[0]).transpose() # 3x1
    Sigma = np.eye(3) #3x3

    N = 720 # number of data points to consider, can tune this later
    estimated_states = np.zeros((3,N)) #3x100
    estimated_states[:,0] = np.array(sensor_data[0]).transpose()

    
    # noise covariance matrices    
    R = np.identity(3) * 0.001
    Q = np.identity(3) * 0.001

    # pf
    noise = np.identity(3) * 0.001
    # print(np.array([sensor_data[0][0], sensor_data[0][1]]))
    pf = ParticleFilter(np.array([sensor_data[0][0], sensor_data[0][1], sensor_data[0][2]]), noise)

    for i in range(1,N):
        z = np.matrix(sensor_data[i]).transpose() #current state, 3x1
        vel = v[i] #current v
        ang_vel = w[i] #current w
        # comes from v and w from generateControls

        u = np.array([[vel*np.cos(z[2,0])*0.1],[vel*np.sin(z[2,0])*0.1],[0]]) #dx, dy, dtheta

        #run the Kalman Filter
        # mu, Sigma = kalmanFilter(np.reshape(mu, (3, 1)), Sigma, z, u, Q, R) #extendedKalmanFilter(z, vel, ang_vel, mu, Sigma, R, Q) 

        #or the Particle Filter
        mu = pf.particleFilter(vel, ang_vel, z)
        
        #store the result (KF)
        estimated_states[:,i] = np.squeeze(mu) #does this need sigma too?
        # if i%3==0:
        #     plot_cov(mu,Sigma,plot_axes)

    #compute the error between your estimate and ground truth
    state_errors = np.transpose(estimated_states[:,0:N]) - path[0:N]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print("Total Error: %f"%total_error)

    # TODO: plot estimated poses? not sure
    # KF
    # plt.plot(estimated_states[0,0:N], estimated_states[1,0:N],'r',linewidth=1.0, label='KF estimate')

    # PF
    plt.plot(estimated_states[0,0:N], estimated_states[1,0:N],'go', label='PF estimate')

    plt.xlabel('x')
    plt.ylabel('y')


    gt_x = [x for x, y, theta in path]
    gt_y = [y for x, y, theta in path]
    sd_x = [x for x, y, theta in sensor_data]
    sd_y = [y for x, y, theta in sensor_data]
    plt.scatter(gt_x, gt_y, s=2, label='ground truth')
    plt.scatter(sd_x, sd_y, s=2, label='sensor data', color = 'gray')
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
