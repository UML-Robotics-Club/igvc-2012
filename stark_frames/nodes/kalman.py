# algorithm from:  http://en.wikipedia.org/wiki/Kalman_filter#Predict
# and Probabilistic Robotics

import numpy as np

class Kalman:
    def __init__(self, size, proc_noise, meas_noise):
        self.A = np.zeros((size,size))     # state transition matrix
        self.x = np.zeros((size,1))        # previous state
        self.B = np.zeros((size,1))        # input response matrix
        self.proc_noise_std = proc_noise   # process noise standard deviation                      
        self.meas_noise_std = meas_noise   # measurment noise standard deviation
        self.P = np.zeros((size,size))     # priori estimate covariance
        self.C = np.zeros((1, size))

    def filter(self, u, z, dt): # u: control input, z: measurement
        self.predict(u, dt)
        self.update(z)
        return self.x, self.P            

    def predict(self, u, dt):
        R = self.proc_noise_std**2 * np.array([[dt**4/4, dt**3/2], # state error
                                               [dt**3/2, dt**2]])
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)   # predict state
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + R # predict covariance

    def update(self, z):
        K = np.dot(np.dot(self.P,self.C.T), # Kalman gain
                   np.linalg.inv(np.dot(np.dot(self.C,self.P), self.C.T) + self.meas_noise_std))
        self.x = self.x + np.dot(K, (z - np.dot(self.C, self.x))) # new state
        self.P = np.dot(np.eye(len(self.A)) - np.dot(K,self.C), self.P) # new covariance estimate
