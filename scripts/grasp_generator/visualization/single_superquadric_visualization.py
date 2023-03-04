from itertools import cycle

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

palette = cycle(["grey"])

import pdb

from pyquaternion import Quaternion


class SingleSuperQuadric:
    def __init__(self, eps, dimension, position, quaternion):
        self.alpha = eps[0]
        self.beta = eps[1]
        self.x = dimension[0]
        self.y = dimension[1]
        self.z = dimension[2]
        self.position = position
        self.quaternion = Quaternion(quaternion)

    def fexp(self, x, p):
        """a different kind of exponentiation"""
        return np.sign(x) * (np.abs(x) ** p)

    def tens_fld(self, A, B, C, P, Q):
        """this module plots superquadratic surfaces with the given parameters"""
        phi, theta = np.mgrid[0 : np.pi : 80j, 0 : 2 * np.pi : 80j]
        x = (A * (self.fexp(np.sin(phi), P)) * (self.fexp(np.cos(theta), Q))).flatten()
        y = (B * (self.fexp(np.sin(phi), P)) * (self.fexp(np.sin(theta), Q))).flatten()
        z = (C * (self.fexp(np.cos(phi), P))).flatten()
        points = np.column_stack((x, y, z))
        return points

    def transformation(self, points):
        for idx, point in enumerate(points):
            points[idx] = self.quaternion.rotate(point) + self.position
        return points

    def visualize(self):
        points = self.tens_fld(self.x, self.y, self.z, self.alpha, self.beta)
        points = self.transformation(points)
        return points


if __name__ == "__main__":

    dim = np.array([0.02726866, 0.03386318, 0.02026305])

    eps1 = 0.3289576
    eps2 = 0.15399751
    eps = np.array([eps1, eps2])

    position = np.array([0, 0, 0])
    quaternion = Quaternion(1, 0, 0, 0)


    alpha = [0.1, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]
    beta = [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]



    superquadric = SingleSuperQuadric(eps, dim, position, quaternion)
    points = superquadric.visualize()
    fig = go.Figure()
    fig.add_trace(
        go.Mesh3d(
            name="superquadric, alpha",
            showlegend=True,
            x=points[:, 0],
            y=points[:, 1],
            z=points[:, 2],
            alphahull=0,
            color="grey",
            opacity=1.00,
            showscale=False,
        )
    )
    fig.update_layout(scene_aspectmode="data")
    fig.show()
