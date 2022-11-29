import numpy as np
import ast
from plotly.subplots import make_subplots
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R

from itertools import cycle


import pdb
# palette = cycle(['black', 'red', 'green', 'orange', 'yellow', 'purple', 'grey'])     
palette = cycle(['black', 'red', 'green'])     
palette2 = cycle(['black', 'red', 'green'])     



def visualize_cube(mesh_xy, mesh_xz, mesh_yz, mesh_xy_neg, mesh_xz_neg, mesh_yz_neg, superquadric):
    fig = go.Figure()


    new_points = showSuperquadrics(superquadric[:,0:2], superquadric[:,2:5], superquadric[:,5:8], superquadric[:,8:12])
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

    fig.add_trace(
        go.Mesh3d(
            name = 'superquadric, alpha',
            showlegend=True,
            x=x.flatten(), 
            y=y.flatten(), 
            z=z.flatten(), 
            alphahull= 0, 
            color=next(palette), 
            opacity=0.70,
            showscale= False))

    fig.add_trace(
        go.Scatter3d(
            x = mesh_xy[:,0],
            y = mesh_xy[:,1],
            z = mesh_xy[:,2],
            mode = 'markers'))
    fig.add_trace(
        go.Scatter3d(
            x = mesh_xz[:,0],
            y = mesh_xz[:,1],
            z = mesh_xz[:,2],
            mode = 'markers'))
    fig.add_trace(
        go.Scatter3d(
            x = mesh_yz[:,0],
            y = mesh_yz[:,1],
            z = mesh_yz[:,2],
            mode = 'markers'))
    fig.add_trace(
        go.Scatter3d(
            x = mesh_xy_neg[:,0],
            y = mesh_xy_neg[:,1],
            z = mesh_xy_neg[:,2],
            mode = 'markers'))
    fig.add_trace(
        go.Scatter3d(
            x = mesh_xz_neg[:,0],
            y = mesh_xz_neg[:,1],
            z = mesh_xz_neg[:,2],
            mode = 'markers'))
    fig.add_trace(
        go.Scatter3d(
            x = mesh_yz_neg[:,0],
            y = mesh_yz_neg[:,1],
            z = mesh_yz_neg[:,2],
            mode = 'markers'))

    fig.update_layout(scene_aspectmode='data')  
    fig.show()




def visualize_superquadric_cylinder(pointcloud, superquadrics, top, bottom, surface):
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


    fig.add_trace(
        go.Scatter3d(
            x = top[:,0],
            y = top[:,1],
            z = top[:,2],
            mode = 'markers'))
        
    fig.add_trace(
        go.Scatter3d(
            x = bottom[:,0],
            y = bottom[:,1],
            z = bottom[:,2],
            mode = 'markers'))

    fig.add_trace(
        go.Scatter3d(
            x = surface[:,0],
            y = surface[:,1],
            z = surface[:,2],
            mode = 'markers'))


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