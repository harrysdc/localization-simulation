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
        self.sensor_cov = np.eye(3) * 0.02
        self.n = 100
        self.particles = []

        for i in range(self.n):
            p = particle()
            self.particles.append(p)

        self.pervious_odom_pose = initial_pose
        self.k1 = 0.5
        self.k2 = 0.5
        self.pose = [0, 0, 0]
        self.pose[0] = initial_pose[0]
        self.pose[1] = initial_pose[1]
        self.pose[2] = initial_pose[2]

        # initialize particles
        for p in self.particles:
            p.x = initial_pose[0]
            p.y = initial_pose[1]
            p.theta = initial_pose[2]
            p.weight = 1.0/self.n


    def action_model(self, odom_pose):
        """
        prediction step
        rotation-translation-rotation (RTR) model
        """
        delta_x = odom_pose[0] - self.pervious_odom_pose[0]
        delta_y = odom_pose[1] - self.pervious_odom_pose[1]
        delta_theta = odom_pose[2] - self.pervious_odom_pose[2]

        rot1 = self.angle_diff(np.arctan2(delta_y, delta_x), self.pervious_odom_pose[2])
        direction = 1.0
        if(abs(rot1) > np.pi/2):
            rot1 = self.angle_diff(np.pi, rot1)
            direction = -1.0

        trans = math.sqrt(delta_x**2 + delta_y**2)
        trans = trans * direction
        rot2 = self.angle_diff(delta_theta, rot1)

        rot1_std = math.sqrt(self.k1 * abs(rot1))
        trans_std = math.sqrt(self.k2 * abs(trans))
        rot2_std = math.sqrt(self.k1 * abs(rot2))

        for p in self.particles:
            sampled_rot1 = np.random.normal(rot1, rot1_std)
            sampled_trans = np.random.normal(trans, trans_std)
            sampled_rot2 = np.random.normal(rot2, rot2_std)
            p.x += sampled_trans * np.cos(p.theta + sampled_rot1)
            p.y += sampled_trans * np.sin(p.theta + sampled_rot1)
            p.theta = self.wrap2Pi(p.theta + sampled_rot1 + sampled_rot2)
        
        self.pervious_odom_pose = odom_pose


    def update_weight(self, sensor_mean):
        """
        corrcetion step
        """
        weight_sum = 0
        rv = multivariate_normal(sensor_mean, self.sensor_cov)
        for p in self.particles:
            p.weight = rv.pdf([p.x, p.y, p.theta])
            weight_sum += p.weight

        for p in self.particles:
            p.weight /= weight_sum  
    
    def estimate_pose(self):
        # initialize var
        x_mean = 0
        y_mean = 0
        cos_theta_mean = 0
        sin_theta_mean = 0

        # calculate pose
        for p in self.particles:
            x_mean += p.weight * p.x
            y_mean += p.weight * p.y
            cos_theta_mean += p.weight * np.cos(p.theta)
            sin_theta_mean += p.weight * np.sin(p.theta)

        self.pose[0] = x_mean
        self.pose[1] = y_mean
        self.pose[2] = np.arctan2(sin_theta_mean, cos_theta_mean)

    def resample(self):
        weight_sum = 0
        new_particles = copy.deepcopy(self.particles)
        r = random.uniform(0, 1.0 / self.n) 
        c = self.particles[0].weight
        i = 0
        for m in range(self.n):
            U = r + m * (1 / self.n)

            while(U > c and i < self.n - 1):
                i += 1
                c += self.particles[i].weight
            new_particles[m] = copy.deepcopy(self.particles[i])
            weight_sum += self.particles[i].weight
        self.particles = new_particles

        for p in self.particles:
            p.weight /= weight_sum


    def wrap2Pi(self, angle):
        angle = (( -angle + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0
        return angle


    def angle_diff(self, theta1, theta2):
        diff = theta1 - theta2
        if(diff > np.pi):
            diff = diff - 2*np.pi
        elif(diff < -np.pi):
            diff = diff + 2*np.pi
        return diff


def Odometry(input, previous_pose, dt=0.1):
    c1 = 0.5
    c2 = 0.5
    c3 = 0.5
    c4 = 0.5
    v = input[0]
    w = input[1]
    R = np.array([[(c1* abs(v)+c2*abs(w))**2, 0], [0, (c3* abs(v)+c4*abs(w))**2]])
    rand = np.random.multivariate_normal(input, R)
    v_actual = rand[0]
    w_actual = rand[1]
    delta_x = v_actual*dt*np.cos(previous_pose[2] + w_actual*dt)
    delta_y = v_actual*dt*np.sin(previous_pose[2] + w_actual*dt)
    delta_theta = w_actual * dt
    delta_pose = np.array([delta_x, delta_y, delta_theta])
    new_pose = previous_pose + delta_pose
    return new_pose


def SensorModel(input, previous_pose, dt=0.1):
    v = input[0]
    w = input[1]
    delta_x = v*dt*np.cos(previous_pose[2] + w*dt)
    delta_y = v*dt*np.sin(previous_pose[2] + w*dt)
    delta_theta = w * dt
    delta_pose = np.array([delta_x, delta_y, delta_theta])
    ground_truth = previous_pose + delta_pose

    sensor_cov = np.eye(3) * 0.02
    new_pose = np.random.multivariate_normal(ground_truth, sensor_cov)
    return new_pose, ground_truth
