import random
import numpy as np
import scipy
from scipy import stats

class ParticleFilter():
    """
    TODO: Implement PF
    """
    def particleFilter(self, v, w, z):
        '''
        X -- input set of particles and weight tuple (state, weight)
        u -- action
        z -- sensor measurement
        '''
        # prediction/correction steps
        particles = ParticleFilter.apply_action(v, w)
        weights = ParticleFilter.likelihood(z)
        X_bar = (particles, weights) #how to combine these two arrays?

        # resampling step
        X_return = ParticleFilter.resampling(X_bar)
        return X_return
    
    def apply_action(self, v, w, dt=0.1):
        # avoid divide by zero
        # if w == 0:
        #     w = 0.0001
        particles = []
        for particle in self.particles:
            x = particle[0][0]
            y = particle[0][1]
            theta = particle[0][2]

            new_pose = np.zeros((3,1))
            new_pose[0] = x + v*np.cos(theta)*dt #-v/w*np.sin(theta) + v/w*np.sin(theta+w*dt)
            new_pose[1] = y + v*np.sin(theta)*dt #v/w*np.cos(theta) - v/w*np.cos(theta+w*dt)
            new_pose[2] = theta + w*dt

            particles.append(new_pose)
        return particles

    def likelihood(self, z):
        weights = []
        weights.fill(1.0)
        for particle in self.particles:
            #compute euclidean distance between particle and sensor measurement
            distance = np.sqrt((particle[0][0] - z[0])**2 + (particle[0][1] - z[1])**2) # use x and y position only
            weight *= stats.norm(distance, R) #scipy.stats.norm(distance, R).pdf(z[i])
            weights.append(weight)
        weights += 0.0001 # avoid round to zero
        weights /= sum(weights) # normalize
        return weights

    def resampling(self, X_prev): #pseudocode from low_variance_resampler
        X_bar = []
        r = random.randrange(0, self.num_particles) # random number between 0 and total particle size
        c = X_prev[0][1] #weight of first particle
        i = 0 #counter
        for m in range(self.num_particles):
            U = r + (m - 1) * (1 / self.num_particles)
            while U > c:
                i += 1
                c = c + X_prev[i][1] # weight of i particle
            X_bar.append(X_prev[i])
        return X_bar


    def __init__(self, start_pos, sigma):
        #initialize X at start
        self.num_particles = 200
        self.particles = []
        for i in range(self.num_particles):
            particle = np.random.normal(start_pos, sigma, 1)
            weight = 1/self.num_particles # sum over all weights is 1
            self.X.append(particle, weight)
