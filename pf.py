import random
import numpy as np

class ParticleFilter():
    """
    TODO: Implement PF
    """
    def particleFilter(X, u, z):
        '''
        X -- input set of particles and weight tuple (state, weight)
        u -- action
        z -- sensor measurement
        '''
        X_bar = []
        X_return = []
        for particle in X:
            # sample x from distribution(state|action,prev state)
            x = np.random.normal(,,)
            # assign weight = probability(z | x)
            X_bar = X_bar.append((x, weight))
        X_return = ParticleFilter.resampling(X_bar, )
        return X_return

    def __init__(self):
        #initialize X samples uniform distribution or gaussian around start
        self.X = []
        for i in range():
            particle = 
            weight = 
            self.X.append(particle, weight)
        #counter = 0
    
    def resampling(X, ): #pseudocode from low_variance_resampler
        X_bar = []
        r = random.randrange(0, len(X)) # random number between 0 and total particle size
        c = X[0][1] #weight of first particle
        i = 0 #counter
        for m in range(len(X)):
            U = r + (m-1) * (1/len(X))
            while U > c:
                i += 1
                c = c + X[i][1] # weight of i particle
            X_bar.append(X[i])
        return X_bar
