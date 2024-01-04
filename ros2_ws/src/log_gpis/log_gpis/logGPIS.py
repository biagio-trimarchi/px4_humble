#!/usr/bin/env python3

# Libraries
import math
import numpy as np
from log_gpis.matern32GP import GaussianProcess as GP

class logGPIS:
    def __init__(self, dimension, lambda_whittle, resolution):
        self.lambda_whittle = lambda_whittle                            # Length scale of Whittle Kernal
        self.regressor = GP(dimension)                                  # Log gaussian implicit surface
        self.regressor.params.L = math.sqrt(2 * 3/2) / lambda_whittle   # Length scale of Matern 3_2 (See article, euristic choice)
        self.resolution = 0.2                                           # Resolution of state space (two point are considered the same if their distance is less than the resolution)

    def addSample(self, position):
        # Add sample to log GPIS
        self.regressor.addSample(position, 1.0)

    def train(self):
        self.regressor.train()

    def evaluate(self, position):
        # Compute estimate distance 
        return - math.log(self.regressor.posteriorMean(position)) / self.lambda_whittle

    def evaluateGradient(self, position):
        # Compute gradient of estimated distance field
        dd = self.evaluate(position)
        grad = self.regressor.gradientPosterionMean(position)
        return -grad / (self.lambda_whittle * dd)

    def evaluate_hessian(self, position):
        dd = d(p)
        grad = self.regressor.gradientPosterionMean(position)
        hess = self.regressor.hessianPosteriorMean(position)
        return grad.T @ grad / (lambda_whittle*dd**2) - hess / (self.lambda_whittle * dd)

    def getSamplesNumber(self):
        return self.regressor.params.N_samples

    def checkCollected(p):
        for tr_point in self.regressor.data_x.T:
            tr_point = tr_point * self.regressor.params.L        # Scale back the sample point (The implemented GP internally scales the sample points)
            if np.linalg.norm(tr_point - p) < self.resolution:   # If the points are "near"
                return True                                          # Return True
        return False                                             # Otherwise return False
