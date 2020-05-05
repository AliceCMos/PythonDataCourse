import numpy as np


def weighted_mean(a, weights, axis=None):
    return np.mean(a*weights, axis=axis)