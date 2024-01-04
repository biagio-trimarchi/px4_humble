#!/usr/bin/env python3

import numpy as np
import math

class gpParameters():
    def __init__(self, input_dimension):
        self.input_dimension = input_dimension      # Input Dimension
        self.sigma_err = 0.01                       # Gaussian Error Variance
        self.L = 1.0                                # Length Scale Matrix
        self.N_samples = 0                          # Total Number of Collected Samples
    
class GaussianProcess:
    def __init__(self, dim):
        self.params = gpParameters(dim)             # Initialize Parameters
    
    def k(self, x1, x2):
        # Matern Kernel of order 3/2

        # Reshape to avoid computation problems
        x1 = x1.reshape((self.params.input_dimension, 1))
        x2 = x2.reshape((self.params.input_dimension, 1))
        
        # Distance between samples
        r = np.linalg.norm(x1 - x2)

        # Compute kernel
        return (1 + np.sqrt(3)*r)*math.exp(-np.sqrt(3)*r)
    
    def dkdx(self, x1, x2):
        # Gradient of the Matern Kernel of order 3/2

        # Reshape vectors to avoid computation problems
        x1 = x1.reshape((self.params.input_dimension, 1))
        x2 = x2.reshape((self.params.input_dimension, 1))

        # Distance between samples
        r = np.linalg.norm(x1 - x2)     

        return -3 * (x1 - x2).T * math.exp(-np.sqrt(3)*r)
    
    def ddkddx(self, x1, x2):
        # Reshape vectors to avoid computation problems
        x1 = x1.reshape((self.params.input_dimension, 1))
        x2 = x2.reshape((self.params.input_dimension, 1))
        r = np.linalg.norm(x1 - x2)

        # When distance between sample is small, avoid division by 0 (the term goes asympotically to 0)
        if r < 10.0**(-6.0):
            return -3 * np.eye(self.params.input_dimension) * math.exp(-np.sqrt(3)*r)

        return ( -3 * np.eye(self.params.input_dimension) + 3*np.sqrt(3)*(x1-x2)*(x1-x2).T/r) * math.exp(-np.sqrt(3)*r)

    def addSample(self, x, y):
        x = x / self.params.L
        # Add sample to the dataset
        if self.params.N_samples == 0:
            self.data_x = x.reshape((self.params.input_dimension, 1))
            self.data_y = np.array([y]).reshape((1, 1))
        else:
            self.data_x = np.append(self.data_x, x.reshape((self.params.input_dimension, 1)), 1)
            self.data_y = np.append(self.data_y, np.array(y).reshape((1,1)), 0)
        
        self.params.N_samples = self.params.N_samples + 1
    
    def train(self):
        # Train the gaussian process
        self.K = np.zeros((self.params.N_samples, self.params.N_samples))
        for row in range(self.params.N_samples):
            for col in range(self.params.N_samples):
                self.K[row, col] = self.k(self.data_x[:, row], self.data_x[:, col])
        self.K = self.K + self.params.sigma_err * np.eye(self.params.N_samples)
        self.L_chol = np.linalg.cholesky(self.K)
        self.alpha = np.linalg.solve(self.L_chol.T, np.linalg.solve(self.L_chol, self.data_y))

    def posteriorMean(self, x):
        x = x/self.params.L
        k = np.zeros((self.params.N_samples, 1))
        for i in range(self.params.N_samples):
            k[i] = self.k(x, self.data_x[:, i])

        return k.T @ self.alpha
    
    def gradientPosterionMean(self, x):
        x = x/self.params.L
        dk = np.block([
            [self.dkdx(x, xs)] for xs in self.data_x.T 
        ])
        return self.alpha.T @ dk
    
    def hessianPosteriorMean(self, x):
        x = x/self.params.L
        sum = np.zeros((self.params.input_dimension, self.params.input_dimension))

        i = 0
        for xs in self.data_x.T:
            sum = sum + self.alpha[i] * self.ddkddx(x, xs)
            i = i+1
        return sum
    
    def posteriorVariance(self, x):
        x = x/self.params.L
        k = np.zeros((self.params.N_samples, 1))
        for i in range(self.params.N_samples):
            k[i] = self.k(x, self.data_x[:, i])
        aux = np.linalg.inv(self.K)
        return self.k(x, x) - k.T @ aux @ k
    
    def gradientPosteriorVariance(self, x):
        x = x/self.params.L
        k = np.zeros((self.params.N_samples, 1))
        for i in range(self.params.N_samples):
            k[i] = self.k(x, self.data_x[:, i])
        dk = np.block([
            [self.dkdx(x, xs)] for xs in self.data_x.T 
        ])
        aux = np.linalg.inv(self.K)
        return - 2 * k.T @ aux @ dk      # K is inverted :/
