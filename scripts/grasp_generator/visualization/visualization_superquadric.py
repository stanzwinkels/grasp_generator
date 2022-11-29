

import numpy as np
import ast
from plotly.subplots import make_subplots
import plotly.graph_objects as go

from itertools import cycle
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion

import pdb
# palette = cycle(['black', 'red', 'green', 'orange', 'yellow', 'purple', 'grey'])     
palette = cycle(['black', 'red', 'green'])     
palette2 = cycle(['black', 'red', 'green'])     


class SingleSuperQuadric():
    def __init__(self, eps, dimension, translation, quaternions):
        self.alpha = eps[:,0]
        self.beta = eps[:,1]
        self.x = dimension[:,0]
        self.y = dimension[:,1]
        self.z = dimension[:,2]
        self.translation = translation
        self.quaternions = quaternions

    def fexp(self, x,p):
        """a different kind of exponentiation"""
        return (np.sign(x) * (np.abs(x)**p))

    def tens_fld(self, A,B,C,P,Q):
        """this module plots superquadratic surfaces with the given parameters"""
        phi, theta = np.mgrid[0:np.pi:40j, 0:2*np.pi:40j]
        x = (A * (self.fexp(np.sin(phi),P)) * (self.fexp(np.cos(theta),Q))).flatten()
        y = (B * (self.fexp(np.sin(phi),P)) * (self.fexp(np.sin(theta),Q))).flatten()
        z = (C * (self.fexp(np.cos(phi),P))).flatten()
        points = np.column_stack((x,y,z))
        return points

    def transformation(self, points, quaternion, translation): 
        for idx, point in enumerate(points):
            points[idx] = np.dot(quaternion.rotation_matrix, point) + translation
        return points

    def coordinates(self):
        quadr_points = []
        for i in range(len(self.alpha)):
            quaternion = Quaternion(self.quaternions[i,3], self.quaternions[i,0], self.quaternions[i,1], self.quaternions[i,2])
            points = self.tens_fld(self.x[i], self.y[i], self.z[i], self.alpha[i], self.beta[i])
            points = self.transformation(points, quaternion, self.translation[i])
            quadr_points.append(points)
        return np.array(quadr_points)


def visualize_superquadric(pointcloud, superquadrics):
    fig = go.Figure()
    for idx, superquadric in enumerate(superquadrics):        
        fig.add_trace(
            go.Mesh3d(
                name = 'superquadric '+ str(idx+1),
                showlegend=True,
                x=superquadric[:,0], y=superquadric[:,1], z=superquadric[:,2], 
                alphahull= 0,                 
                color=next(palette),
                opacity=0.50))
    fig.add_trace(
        go.Scatter3d(
            name = 'pointcloud',
            showlegend=True,
            x=pointcloud[:,0], y=pointcloud[:,1], z=pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color='blue',
                        opacity=0.7)))
    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False)
    fig.update_layout(
        plot_bgcolor="white",
        paper_bgcolor="white",
        showlegend=True)
    fig.show()
    return


def visualize_superquadric_cylinder(pointcloud, superquadrics, region):
    fig = go.Figure()
    for idx, superquadric in enumerate(superquadrics):        
        fig.add_trace(
            go.Mesh3d(
                name = 'superquadric '+ str(idx+1),
                showlegend=True,
                x=superquadric[:,0], y=superquadric[:,1], z=superquadric[:,2], 
                alphahull= 0,                 
                color=next(palette),
                opacity=0.50))
    fig.add_trace(
        go.Scatter3d(
            name = 'pointcloud',
            showlegend=True,
            x=pointcloud[:,0], y=pointcloud[:,1], z=pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=0.01, 
                        color='blue',
                        opacity=0.7)))
                    
    for points in region:
        fig.add_trace(
            go.Scatter3d(
                x = points[:,0],y = points[:,1],z = points[:,2],
                mode = 'markers'))
    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False)
    fig.update_layout(
        plot_bgcolor="white",
        paper_bgcolor="white",
        showlegend=True)
    fig.show()
    return


def visualize_superquadric_segmentation(pointcloud, superquadrics, closest_primitive):
    """Segmentation using the superquadric shapes w.r.t. partial pointcloud"""
    fig = go.Figure()
    for idx, superquadric in enumerate(superquadrics):        
        fig.add_trace(
            go.Mesh3d(
                name = 'superquadric '+ str(idx+1),
                showlegend=True,
                x=superquadric[:,0], 
                y=superquadric[:,1], 
                z=superquadric[:,2], 
                alphahull= 0,                 
                color=next(palette),
                opacity=0.50))

    for value in set(closest_primitive): 
        segmented_pointcloud = pointcloud[closest_primitive == value]
        fig.add_trace(
            go.Scatter3d(
                name = 'pointcloud',
                showlegend=True,
                x=segmented_pointcloud[:,0], 
                y=segmented_pointcloud[:,1], 
                z=segmented_pointcloud[:,2],
                mode = 'markers', 
                marker=dict(size=1, 
                            color=next(palette2),
                            opacity=0.7)))
    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False,
    )

    fig.update_layout(
        # plot_bgcolor="white",
        # paper_bgcolor="white",
        showlegend=True
                # scene = dict(
                #         xaxis = dict(nticks=1, range=[-0.1,0.19],),
                #         yaxis = dict(nticks=1, range=[-0.09,0.19],),
                #         zaxis = dict(nticks=1, range=[0.48,0.78],),),
                #         width=1100,
                #         margin=dict(r=0.02, l=0.010, b=0.010, t=0.010)
                        )
    fig.show()
    return


def visualize_superquadric_true_segmentation(pointcloud, superquadrics, closest_primitive):
    """Segmentation using the superquadric shapes w.r.t. partial pointcloud"""
    fig = go.Figure()
    for idx, superquadric in enumerate(superquadrics):        
        fig.add_trace(
            go.Mesh3d(
                name = 'superquadric '+ str(idx+1),
                showlegend=True,
                x=superquadric[:,0], 
                y=superquadric[:,1], 
                z=superquadric[:,2], 
                alphahull= 0,                 
                color=next(palette),
                opacity=0.50))


    fig.add_trace(
        go.Scatter3d(
            name = 'pointcloud',
            showlegend=True,
            x=pointcloud[closest_primitive == 0][:,0], 
            y=pointcloud[closest_primitive == 0][:,1], 
            z=pointcloud[closest_primitive == 0][:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color='red',
                        opacity=0.7)))

    

    for value in set(closest_primitive): 
        segmented_pointcloud = pointcloud[closest_primitive == value]
        fig.add_trace(
            go.Scatter3d(
                name = 'pointcloud',
                showlegend=True,
                x=segmented_pointcloud[:,0], 
                y=segmented_pointcloud[:,1], 
                z=segmented_pointcloud[:,2],
                mode = 'markers', 
                marker=dict(size=1, 
                            color='grey',
                            opacity=0.7)))
    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False,
    )

    fig.update_layout(
        # plot_bgcolor="white",
        # paper_bgcolor="white",
        showlegend=True
                # scene = dict(
                #         xaxis = dict(nticks=1, range=[-0.1,0.19],),
                #         yaxis = dict(nticks=1, range=[-0.09,0.19],),
                #         zaxis = dict(nticks=1, range=[0.48,0.78],),),
                #         width=1100,
                #         margin=dict(r=0.02, l=0.010, b=0.010, t=0.010)
                        )
    fig.show()
    return


def visualize_gt_pred(pointcloud_gt, ground_truth, partial_pointcloud, partial_pred):
    fig = make_subplots(
        rows=1, cols=2,
        specs=[[{'type': 'surface'}, {'type': 'surface'}]]
    )

    for value in set(ground_truth): 
        segmented_pointcloud = pointcloud_gt[ground_truth == value]
        fig.add_trace(
            go.Scatter3d(
                name = 'pointcloud',
                showlegend=True,
                x=segmented_pointcloud[:,0], 
                y=segmented_pointcloud[:,1], 
                z=segmented_pointcloud[:,2],
                mode = 'markers', 
                marker=dict(size=1, 
                            color=next(palette),
                            opacity=0.7)),
                row=1, 
                col=1)

    for value in set(partial_pred): 
        segmented_pointcloud = partial_pointcloud[partial_pred == value]
        fig.add_trace(
            go.Scatter3d(
                name = 'pointcloud',
                showlegend=True,
                x=segmented_pointcloud[:,0], 
                y=segmented_pointcloud[:,1], 
                z=segmented_pointcloud[:,2],
                mode = 'markers', 
                marker=dict(size=1, 
                            color=next(palette2),
                            opacity=0.7)),
                row=1, 
                col=2)
    fig.update_layout(scene_aspectmode='data')  
    fig.show()


def visualize_pointclouds(partial_pointcloud, full_pointcloud): 
    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(
            name = 'partial pointcloud',
            showlegend=True,
            x=partial_pointcloud[:,0], 
            y=partial_pointcloud[:,1], 
            z=partial_pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color=next(palette2),
                        opacity=0.7)))
    fig.add_trace(
        go.Scatter3d(
            name = 'full pointcloud',
            showlegend=True,
            x=full_pointcloud[:,0], 
            y=full_pointcloud[:,1], 
            z=full_pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color=next(palette2),
                        opacity=0.7)))
    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False,
    )
    fig.update_yaxes(
        scaleanchor = "x",
        scaleratio = 1,
    )

    fig.update_layout(
        plot_bgcolor="white",
        paper_bgcolor="white",
        showlegend=True
                # scene = dict(
                #         xaxis = dict(nticks=1, range=[-0.1,0.19],),
                #         yaxis = dict(nticks=1, range=[-0.09,0.19],),
                #         zaxis = dict(nticks=1, range=[0.48,0.78],),),
                #         width=1100,
                #         margin=dict(r=0.02, l=0.010, b=0.010, t=0.010)
                        )
    fig.show()
    return