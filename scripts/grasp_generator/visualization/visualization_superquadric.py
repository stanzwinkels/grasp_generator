

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
    def __init__(self, superquadrics):        
        self.alpha = superquadrics[:, 0]
        self.beta = superquadrics[:, 1]
        self.x = superquadrics[:, 2]
        self.y = superquadrics[:, 3]
        self.z = superquadrics[:, 4]
        self.translation = superquadrics[:, 9:12]
        self.quaternions = superquadrics[:, 5:9]

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
                # color='grey',
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


def visualize_superquadric_cylinder(pointcloud, superquadrics, regions):
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
    for region in regions:                
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
        if value != 0: 
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


palette_visualize_gt = cycle(['black', 'red', 'green'])
palette_visualize_gt2 = cycle(['black', 'red', 'green'])

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
                            color=next(palette_visualize_gt),
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
                            color=next(palette_visualize_gt2),
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





def visualize_grasp_point(grasp_point, point, full_pointcloud, ground_truth): 
    fig = go.Figure()

    for value in set(ground_truth): 
        segmented_pointcloud = full_pointcloud[ground_truth == value]
        fig.add_trace(
            go.Scatter3d(
                name = 'pointcloud',
                showlegend=True,
                x=segmented_pointcloud[:,0], 
                y=segmented_pointcloud[:,1], 
                z=segmented_pointcloud[:,2],
                mode = 'markers', 
                marker=dict(size=1, 
                            color=next(palette_visualize_gt),
                            opacity=0.7)))

    # fig.add_trace(
    #     go.Scatter3d(
    #         name = 'full pointcloud',
    #         showlegend=True,
    #         x=full_pointcloud[:,0], 
    #         y=full_pointcloud[:,1], 
    #         z=full_pointcloud[:,2],
    #         mode = 'markers', 
    #         marker=dict(size=1, 
    #                     color=next(palette2),
    #                     opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'grasp point',
            showlegend=True,
            x=[grasp_point[0]], 
            y=[grasp_point[1]], 
            z=[grasp_point[2]],
            mode = 'markers', 
            marker=dict(size=8, 
                        color='green',
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'point',
            showlegend=True,
            x=[point[0]], 
            y=[point[1]], 
            z=[point[2]],
            mode = 'markers', 
            marker=dict(size=8, 
                        color='red',
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


def visualize_scene_pointcloud(scene_pointcloud, left_lines, right_lines, middle_lines, hand_lines, gpd_left_line, gpd_right_line, gpd_middle_line, gpd_hand_line, reasoning_left_line, reasoning_right_line, reasoning_middle_line, reasoning_hand_line):
    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(
            name = 'partial pointcloud',
            showlegend=True,
            x=scene_pointcloud[:,0], 
            y=scene_pointcloud[:,1], 
            z=scene_pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color="blue",
                        opacity=0.7)))

    for left_line in left_lines:
        fig.add_trace(
            go.Scatter3d(
                name = 'Hand',
                showlegend=True,
                x=left_line[:,0], 
                y=left_line[:,1], 
                z=left_line[:,2],
                mode = 'lines', 
                marker=dict(size=1, 
                            color="orange",
                            opacity=0.7)))

    for right_line in right_lines:
        fig.add_trace(
            go.Scatter3d(
                name = 'Hand',
                showlegend=True,
                x=right_line[:,0], 
                y=right_line[:,1], 
                z=right_line[:,2],
                mode = 'lines', 
                marker=dict(size=1, 
                            color="orange",
                            opacity=0.7)))

    for middle_line in middle_lines:
        fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=middle_line[:,0], 
            y=middle_line[:,1], 
            z=middle_line[:,2],
            mode = 'lines', 
            marker=dict(size=1, 
                        color="orange",
                        opacity=0.7)))

    for hand_line in hand_lines:
        fig.add_trace(
            go.Scatter3d(
                name = 'Hand',
                showlegend=True,
                x=hand_line[:,0], 
                y=hand_line[:,1], 
                z=hand_line[:,2],
                mode = 'lines', 
                marker=dict(size=1, 
                            color="orange",
                            opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=gpd_left_line[:,0], 
            y=gpd_left_line[:,1], 
            z=gpd_left_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=gpd_right_line[:,0], 
            y=gpd_right_line[:,1], 
            z=gpd_right_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=gpd_middle_line[:,0], 
            y=gpd_middle_line[:,1], 
            z=gpd_middle_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=gpd_hand_line[:,0], 
            y=gpd_hand_line[:,1], 
            z=gpd_hand_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=reasoning_left_line[:,0], 
            y=reasoning_left_line[:,1], 
            z=reasoning_left_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=reasoning_right_line[:,0], 
            y=reasoning_right_line[:,1], 
            z=reasoning_right_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=reasoning_middle_line[:,0], 
            y=reasoning_middle_line[:,1], 
            z=reasoning_middle_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Hand',
            showlegend=True,
            x=reasoning_hand_line[:,0], 
            y=reasoning_hand_line[:,1], 
            z=reasoning_hand_line[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
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




def visualize_grasps_pointcloud(scene_pointcloud, left_lines, right_lines, middle_lines, hand_lines, left_line_gpd, right_line_gpd, middle_line_gpd, hand_line_gpd, left_line_reasoning, right_line_reasoning, middle_line_reasoning, hand_line_reasoning, grasp_pose_gpd ,grasp_pose_reasoning):
    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(
            name = 'partial pointcloud',
            showlegend=True,
            x=scene_pointcloud[:,0], 
            y=scene_pointcloud[:,1], 
            z=scene_pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color="blue",
                        opacity=0.7)))

    # for left_line in left_lines:
    #     fig.add_trace(
    #         go.Scatter3d(
    #             name = 'Hand',
    #             showlegend=True,
    #             x=left_line[:,0], 
    #             y=left_line[:,1], 
    #             z=left_line[:,2],
    #             mode = 'lines', 
    #             marker=dict(size=1, 
    #                         color="orange",
    #                         opacity=0.7)))

    # for right_line in right_lines:
    #     fig.add_trace(
    #         go.Scatter3d(
    #             name = 'Hand',
    #             showlegend=True,
    #             x=right_line[:,0], 
    #             y=right_line[:,1], 
    #             z=right_line[:,2],
    #             mode = 'lines', 
    #             marker=dict(size=1, 
    #                         color="orange",
    #                         opacity=0.7)))

    # for middle_line in middle_lines:
    #     fig.add_trace(
    #     go.Scatter3d(
    #         name = 'Hand',
    #         showlegend=True,
    #         x=middle_line[:,0], 
    #         y=middle_line[:,1], 
    #         z=middle_line[:,2],
    #         mode = 'lines', 
    #         marker=dict(size=1, 
    #                     color="orange",
    #                     opacity=0.7)))

    # for hand_line in hand_lines:
    #     fig.add_trace(
    #         go.Scatter3d(
    #             name = 'Hand',
    #             showlegend=True,
    #             x=hand_line[:,0], 
    #             y=hand_line[:,1], 
    #             z=hand_line[:,2],
    #             mode = 'lines', 
    #             marker=dict(size=1, 
    #                         color="orange",
    #                         opacity=0.7)))


    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=left_line_gpd[:,0], 
            y=left_line_gpd[:,1], 
            z=left_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=right_line_gpd[:,0], 
            y=right_line_gpd[:,1], 
            z=right_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=middle_line_gpd[:,0], 
            y=middle_line_gpd[:,1], 
            z=middle_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=hand_line_gpd[:,0], 
            y=hand_line_gpd[:,1], 
            z=hand_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))


    fig.add_trace(
        go.Scatter3d(
            name = 'Reasoning',
            showlegend=True,
            x=left_line_reasoning[:,0], 
            y=left_line_reasoning[:,1], 
            z=left_line_reasoning[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Reasoning',
            showlegend=True,
            x=right_line_reasoning[:,0], 
            y=right_line_reasoning[:,1], 
            z=right_line_reasoning[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Reasoning',
            showlegend=True,
            x=middle_line_reasoning[:,0], 
            y=middle_line_reasoning[:,1], 
            z=middle_line_reasoning[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Reasoning',
            showlegend=True,
            x=hand_line_reasoning[:,0], 
            y=hand_line_reasoning[:,1], 
            z=hand_line_reasoning[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="green",
                        opacity=0.7)))


    fig.add_trace(
        go.Scatter3d(
            name = 'Reasoning',
            showlegend=True,
            x=[grasp_pose_gpd.position.x], 
            y=[grasp_pose_gpd.position.y], 
            z=[grasp_pose_gpd.position.z],
            mode = 'markers', 
            marker=dict(size=3, 
                        color="black",
                        opacity=0.7)))



    fig.add_trace(
        go.Scatter3d(
            name = 'Reasoning',
            showlegend=True,
            x=[grasp_pose_reasoning.position.x], 
            y=[grasp_pose_reasoning.position.y], 
            z=[grasp_pose_reasoning.position.z],
            mode = 'markers', 
            marker=dict(size=3, 
                        color="black",
                        opacity=0.7)))

    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False,
    )

    fig.update_layout(
        plot_bgcolor="white",
        paper_bgcolor="white",
        showlegend=True)
    fig.show()
    return


       

def  visualize_grasp_gpd_pointcloud(scene_pointcloud, left_line_gpd, right_line_gpd, middle_line_gpd, hand_line_gpd, grasp_pose_gpd):
    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(
            name = 'partial pointcloud',
            showlegend=True,
            x=scene_pointcloud[:,0], 
            y=scene_pointcloud[:,1], 
            z=scene_pointcloud[:,2],
            mode = 'markers', 
            marker=dict(size=1, 
                        color="blue",
                        opacity=0.7)))
    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=left_line_gpd[:,0], 
            y=left_line_gpd[:,1], 
            z=left_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=right_line_gpd[:,0], 
            y=right_line_gpd[:,1], 
            z=right_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=middle_line_gpd[:,0], 
            y=middle_line_gpd[:,1], 
            z=middle_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'GPD',
            showlegend=True,
            x=hand_line_gpd[:,0], 
            y=hand_line_gpd[:,1], 
            z=hand_line_gpd[:,2],
            mode = 'lines', 
            marker=dict(size=2, 
                        color="red",
                        opacity=0.7)))

    fig.add_trace(
        go.Scatter3d(
            name = 'Reasoning',
            showlegend=True,
            x=[grasp_pose_gpd.position.x], 
            y=[grasp_pose_gpd.position.y], 
            z=[grasp_pose_gpd.position.z],
            mode = 'markers', 
            marker=dict(size=3, 
                        color="black",
                        opacity=0.7)))


    fig.update_scenes(
        xaxis_visible=False, 
        yaxis_visible=False,
        zaxis_visible=False,
    )

    fig.update_layout(
        plot_bgcolor="white",
        paper_bgcolor="white",
        showlegend=True)
    fig.show()
    return