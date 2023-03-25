__author__ = 'aslan'

import numpy as np


def rotox(angle):
    """ Rotation around x-axis """
    cos = np.cos(angle)
    sin = np.sin(angle)
    m = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cos, -sin],
        [0.0, sin, cos]
    ])
    return m


def rotoy(angle):
    """ Rotation around y-axis """
    cos = np.cos(angle)
    sin = np.sin(angle)
    m = np.array([
        [cos, 0.0, sin],
        [0.0, 1.0, 0.0],
        [-sin, 0.0, cos]
    ])
    return m


def rotoz(angle):
    """ Rotation around z-axis """
    cos = np.cos(angle)
    sin = np.sin(angle)
    m = np.array([
        [cos, -sin, 0.0],
        [sin, cos, 0.0],
        [0.0, 0.0, 1.0]
    ])
    return m
