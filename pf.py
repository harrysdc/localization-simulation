import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
import random
import math
from scipy.stats import multivariate_normal
import copy
import matplotlib.pyplot as plt


class particle():
    def __init__(self):
        self.x = 0
        self.y = 0 
        self.theta = 0
        self.weight = 0


class PF():
    def __init__(self, initial_pose):
        self.n = 500
        self.particles = []

        for i in range(self.n):
            p = particle()
            self.particles.append(p)

        # initialize as equal weight
        for p in self.particles:
            p.x = initial_pose[0]
            p.y = initial_pose[1]
            p.theta = initial_pose[2]
            p.weight = 1.0/self.n
        
        self.prev_odom = initial_pose
        self.state = [0, 0, 0]
        self.state[0] = initial_pose[0]
        self.state[1] = initial_pose[1]
        self.state[2] = initial_pose[2]

    def action_model(self, curr_odom, k1=0.01, k2=0.01):
    #def action_model(self, curr_odom, k1=0.05, k2=0.05):
        """
        prediction step
        rotation-translation-rotation (RTR) model
        """
        delta_x = curr_odom[0] - self.prev_odom[0]
        delta_y = curr_odom[1] - self.prev_odom[1]
        delta_theta = curr_odom[2] - self.prev_odom[2]

        rot1 = self.angle_diff(np.arctan2(delta_y, delta_x), self.prev_odom[2])
        
        direction = 1.0
        if(abs(rot1) > np.pi/2):
            rot1 = self.angle_diff(np.pi, rot1)
            direction = -1.0
        trans = math.sqrt(delta_x**2 + delta_y**2)
        trans = trans * direction

        rot2 = self.angle_diff(delta_theta, rot1)

        rot1_std = math.sqrt(k1 * abs(rot1))
        trans_std = math.sqrt(k2 * abs(trans))
        rot2_std = math.sqrt(k1 * abs(rot2))

        for p in self.particles:
            sampled_rot1 = np.random.normal(rot1, rot1_std)
            sampled_trans = np.random.normal(trans, trans_std)
            sampled_rot2 = np.random.normal(rot2, rot2_std)
            p.x += sampled_trans * np.cos(p.theta + sampled_rot1)
            p.y += sampled_trans * np.sin(p.theta + sampled_rot1)
            p.theta = self.wrapToPi(p.theta + sampled_rot1 + sampled_rot2)
        
        self.prev_odom = curr_odom


    def update_weight(self, sensor_data, sensor_cov=np.eye(3)):
    #def update_weight(self, sensor_data, sensor_cov=np.eye(3)*0.02):
        """
        corrcetion step
        """
        sum = 0
        rv = multivariate_normal(sensor_data, sensor_cov)
        for p in self.particles:
            p.weight = rv.pdf([p.x, p.y, p.theta])
            sum += p.weight

        for p in self.particles:
            p.weight /= sum  


    def posteriorPose(self):
        """
        circular mean
        """

        x_mean, y_mean = 0, 0
        sinSum, cosSum = 0, 0

        for p in self.particles:
            x_mean += p.weight * p.x
            y_mean += p.weight * p.y
            cosSum += p.weight * np.cos(p.theta)
            sinSum += p.weight * np.sin(p.theta)

        self.state[0] = x_mean
        self.state[1] = y_mean
        self.state[2] = np.arctan2(sinSum, cosSum)


    def resample(self):
        sum = 0
        new_particles = copy.deepcopy(self.particles)
        r = np.random.rand(1) / self.n
        c = self.particles[0].weight
        i = 0
        for j in range(self.n):
            U = r + j * (1 / self.n)

            while U > c and i < self.n-1:
                i += 1
                c += self.particles[i].weight
            new_particles[j] = copy.deepcopy(self.particles[i])
            sum += self.particles[i].weight
        
        self.particles = new_particles
        for p in self.particles:
            p.weight /= sum


    def wrapToPi(self, angle):
        angle = (( -angle + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0
        return angle


    def angle_diff(self, theta1, theta2):
        diff = theta1 - theta2
        if diff > np.pi:
            diff = diff - 2*np.pi
        elif diff < -np.pi:
            diff = diff + 2*np.pi
        return diff
