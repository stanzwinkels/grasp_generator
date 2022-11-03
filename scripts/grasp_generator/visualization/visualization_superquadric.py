

import numpy as np
import ast
from plotly.subplots import make_subplots
import plotly.graph_objects as go

from itertools import cycle
from scipy.spatial.transform import Rotation as R
import pdb
# palette = cycle(['black', 'red', 'green', 'orange', 'yellow', 'purple', 'grey'])     
palette = cycle(['black', 'red', 'green'])     
palette2 = cycle(['black', 'red', 'green'])     



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
    fig.show()

def visualize_highlight_superquadric(pointcloud, superquadrics, request_ID):
    fig = go.Figure()

    request_ID = ast.literal_eval(request_ID[0])
    for i in range(len(superquadrics)):
        shape = superquadrics[i][0:2]
        scale = superquadrics[i][2:5]
        quaternion = superquadrics[i][5:9]
        translation = superquadrics[i][9:12]
        q = R.from_quat(quaternion)
        rotation_matrix = q.as_dcm() 
        new_points = showSuperquadrics(shape, scale, rotation_matrix, translation)
        if (i+1) in request_ID:
            fig.add_trace(
                go.Mesh3d(
                    name = 'superquadric '+ str(i+1),
                    showlegend=True,
                    x=new_points[:,0], 
                    y=new_points[:,1], 
                    z=new_points[:,2], 
                    alphahull= 0, 
                    color= 'red', 
                    opacity=0.50
                    ))
        
        else: 
            fig.add_trace(
                go.Mesh3d(
                    name = 'superquadric '+ str(i+1),
                    showlegend=True,
                    x=new_points[:,0], 
                    y=new_points[:,1], 
                    z=new_points[:,2], 
                    alphahull= 0, 
                    color= 'grey', 
                    opacity=0.50
                    ))            

    fig.add_trace(
        go.Scatter3d(
            name = 'pointcloud',
            showlegend=True,
            x=pointcloud[:,0], 
            y=pointcloud[:,1], 
            z=pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color='blue',
                        opacity=0.7)))
    
    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False,
    )

    fig.update_layout(
        plot_bgcolor="white",
        paper_bgcolor="white",
        showlegend=True,
                scene = dict(
                        xaxis = dict(nticks=1, range=[-0.10,0.25],),
                        yaxis = dict(nticks=1, range=[-0.10,0.25],),
                        zaxis = dict(nticks=1, range=[0.50,0.85],),),
                        width=1100,
                        margin=dict(r=0.02, l=0.010, b=0.010, t=0.010)
                        )
    fig.show()
    return





def visualize_superquadric(pointcloud, superquadrics):
    fig = go.Figure()
    for idx, para in enumerate(superquadrics):
        shape = superquadrics[idx][0:2]
        scale = superquadrics[idx][2:5]
        quaternion = superquadrics[idx][5:9]
        translation = superquadrics[idx][9:12]
        q = R.from_quat(quaternion)
        rotation_matrix = q.as_dcm() 
        new_points = showSuperquadrics(shape, scale, rotation_matrix, translation)
        
        fig.add_trace(
            go.Mesh3d(
                name = 'superquadric '+ str(idx+1),
                showlegend=True,
                x=new_points[:,0], 
                y=new_points[:,1], 
                z=new_points[:,2], 
                # z=np.array(z_mesh.flatten()),
                alphahull= 0,                 
                color=next(palette),
                opacity=0.50
                ))

    fig.add_trace(
        go.Scatter3d(
            name = 'pointcloud',
            showlegend=True,
            x=pointcloud[:,0], 
            y=pointcloud[:,1], 
            z=pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color='blue',
                        opacity=0.7)))
    
    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False,
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
    

def visualize_superquadric_segmentation(closest_primitive, pointcloud, superquadrics):
    fig = go.Figure()

    for idx, superquadric in enumerate(superquadrics):
        shape = superquadric[0:2]
        scale = superquadric[2:5]
        quaternion = superquadric[5:9]
        translation = superquadric[9:12]
        q = R.from_quat(quaternion)
        rotation_matrix = q.as_dcm() 
        new_points = showSuperquadrics(shape, scale, rotation_matrix, translation)
        
        fig.add_trace(
            go.Mesh3d(
                name = 'superquadric '+ str(idx+1),
                showlegend=True,
                x=new_points[:,0], 
                y=new_points[:,1], 
                z=new_points[:,2], 
                # z=np.array(z_mesh.flatten()),
                alphahull= 0,                 
                color=next(palette),
                opacity=0.50
                ))

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


def showSuperquadrics(shape, scale, rotation, translation, threshold = 1e-2, num_limit = 10000, arclength = 0.3):
    # avoid numerical instability in sampling
    if shape[0] < 0.007:
        shape[0] = 0.007
    if shape[1] < 0.007:
        shape[1] = 0.007
    # sampling points in superellipse    
    point_eta = uniformSampledSuperellipse(shape[0], [1, scale[2]], threshold, num_limit, arclength)
    point_omega = uniformSampledSuperellipse(shape[1], [scale[0], scale[1]], threshold, num_limit, arclength)

    points_origin = []
    new_points = []
    for m in range(np.shape(point_omega)[1]):
        for n in range(np.shape(point_eta)[1]):
            point_temp = np.zeros(3)
            point_temp[0 : 2] = point_omega[:, m] * point_eta[0, n]
            point_temp[2] = point_eta[1, n]
            points_origin.append(point_temp)
            point_temp = np.dot(rotation, point_temp) + translation
            new_points.append(point_temp)    
    points_origin = np.array(points_origin)
    new_points = np.array(new_points)

    return new_points



def uniformSampledSuperellipse(epsilon, scale, threshold = 1e-2, num_limit = 10000, arclength = 0.02):

    # initialize array storing sampled theta
    theta = np.zeros(num_limit)
    theta[0] = 0

    for i in range(num_limit):
        dt = dtheta(theta[i], arclength, threshold, scale, epsilon)
        theta_temp = theta[i] + dt

        if theta_temp > np.pi / 4:
            theta[i + 1] = np.pi / 4
            break
        else:
            if i + 1 < num_limit:
                theta[i + 1] = theta_temp
            else:
                raise Exception(
                'Number of the sampled points exceed the preset limit', \
                num_limit,
                'Please decrease the sampling arclength.'
                )
    critical = i + 1

    for j in range(critical + 1, num_limit):
        dt = dtheta(theta[j], arclength, threshold, np.flip(scale), epsilon)
        theta_temp = theta[j] + dt
        
        if theta_temp > np.pi / 4:
            break
        else:
            if j + 1 < num_limit:
                theta[j + 1] = theta_temp
            else:
                raise Exception(
                'Number of the sampled points exceed the preset limit', \
                num_limit,
                'Please decrease the sampling arclength.'
                )
    num_pt = j
    theta = theta[0 : num_pt + 1]

    point_fw = angle2points(theta[0 : critical + 1], scale, epsilon)
    point_bw = np.flip(angle2points(theta[critical + 1: num_pt + 1], np.flip(scale), epsilon), (0, 1))
    point = np.concatenate((point_fw, point_bw), 1)
    point = np.concatenate((point, np.flip(point[:, 0 : num_pt], 1) * np.array([[-1], [1]]), 
                           point[:, 1 : num_pt + 1] * np.array([[-1], [-1]]),
                           np.flip(point[:, 0 : num_pt], 1) * np.array([[1], [-1]])), 1)

    return point

def dtheta(theta, arclength, threshold, scale, epsilon):
    # calculation the sampling step size
    if theta < threshold:
        dt = np.abs(np.power(arclength / scale[1] +np.power(theta, epsilon), \
             (1 / epsilon)) - theta)
    else:
        dt = arclength / epsilon * ((np.cos(theta) ** 2 * np.sin(theta) ** 2) /
             (scale[0] ** 2 * np.cos(theta) ** (2 * epsilon) * np.sin(theta) ** 4 +
             scale[1] ** 2 * np.sin(theta) ** (2 * epsilon) * np.cos(theta) ** 4)) ** (1 / 2)
    
    return dt

def angle2points(theta, scale, epsilon):

    point = np.zeros((2, np.shape(theta)[0]))
    point[0] = scale[0] * np.sign(np.cos(theta)) * np.abs(np.cos(theta)) ** epsilon
    point[1] = scale[1] * np.sign(np.sin(theta)) * np.abs(np.sin(theta)) ** epsilon

    return point