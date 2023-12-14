import random
import numpy as np
from scipy.stats import multivariate_normal

class ParticleFilter():
    """
    TODO: Implement PF
    """
    def particleFilter(self, v, w, z):
        '''
        v, w -- action
        z -- sensor measurement
        '''
        # prediction/correction steps
        self.apply_action(v, w)
        self.likelihood(z)

        # resampling step
        self.resampling()

        # get average particles of best X% of particles
        index = int(0.5 * self.num_particles)
        best_particles = self.particles[:index]

        mu = np.zeros((3,1))
        mu[0] = np.mean(best_particles[0])
        mu[1] = np.mean(best_particles[1])
        mu[2] = np.mean(best_particles[2])

        return mu
    
    
    def apply_action(self, v, w, dt=0.1):
        # avoid divide by zero
        # if w == 0:
        #     w = 0.0001
        # particles = []
        for particle in self.particles:
            x = particle[0]
            y = particle[1]
            theta = particle[2]

            new_pose = np.zeros((3,1))
            new_pose[0] = x + v*np.cos(theta)*dt #-v/w*np.sin(theta) + v/w*np.sin(theta+w*dt)
            new_pose[1] = y + v*np.sin(theta)*dt #v/w*np.cos(theta) - v/w*np.cos(theta+w*dt)
            new_pose[2] = theta + w*dt

            particle = new_pose

    def likelihood(self, z, cov = 0.001): #needs tuning
        weights = []# np.zeros(self.num_particles) + 1/self.num_particles
        for particle in self.particles:
            #compute euclidean distance between particle and sensor measurement
            distance = np.sqrt((particle[0] - z[0])**2 + (particle[1] - z[1])**2 + (particle[2] - z[2])**2) # use x and y position only
            weight = multivariate_normal.pdf(distance, 0, cov) #scipy.stats.norm(distance, R).pdf(z[i])
            weights.append(weight)
        weights = np.asarray(weights)
        weights += 0.0001 # avoid round to zero
        weights /= sum(weights) # normalize
        self.weights = weights

    def resampling(self): #pseudocode from low_variance_resampler
        new_particles = []
        r = np.random.rand(1) / self.num_particles
        i = 0 #counter
        c = self.weights[i] #weight of first particle
        for m in range(self.num_particles):
            U = r + (m - 1) * (1 / self.num_particles)
            while U > c:
                i += 1
                c += self.weights[i] # weight of i particle
            new_particles.append(self.particles[i])
        self.particles = np.asarray(new_particles)
        self.weights = np.zeros(self.num_particles) + 1/self.num_particles #NOT SURE

    def __init__(self, mu, sigma): 
        '''
        mu -- start position, shape (2,)
        sigma -- covariance, shape(2,2)
        '''
        self.num_particles = 200
        self.particles = np.random.multivariate_normal(mu, sigma, self.num_particles)
        self.weights = np.zeros(self.num_particles) + 1/self.num_particles

