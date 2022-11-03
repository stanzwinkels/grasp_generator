#!/usr/bin/env python

'''
Author: Stan Zwinkels
Context: This file is made to visualize the segmented full point cloud. 

'''

# Ros imports
import rospy
import numpy as np

import pdb
import numpy as np
import ast
import rospkg

import plotly.graph_objects as go
from itertools import cycle

palette = cycle(['black', 'red', 'green', 'orange', 'yellow', 'purple', 'grey'])     

from grasp_generator.utils.standard_functions import (
    save_pointcloud,
    load_pointcloud
)



class Main:
    def __init__(
        self, debug=False, save=False, load=True, superquadric_visualize=True
    ):
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")

        self.debug = debug
        self.save = save
        self.load = load
        self.superquadric_visualize = superquadric_visualize
        self.grasp_visualize = False

    def run(self):
        if self.load:
            self.partial_pointcloud = load_pointcloud(
                self.package_path + "/data/full_pointcloud")

        self.visualize()

        pdb.set_trace()


    def visualize(self): 
        fig = go.Figure()
        fig.add_trace(
            go.Scatter3d(
                name = 'pointcloud',
                showlegend=True,
                x=self.partial_pointcloud[:,0], 
                y=self.partial_pointcloud[:,1], 
                z=self.partial_pointcloud[:,2],
                mode = 'markers', 
                marker=dict(size=1, 
                            color=next(palette),
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
            )
        fig.show()


if __name__ == "__main__":
    rospy.init_node("visualize_full_pointcloud")

    start = Main()
    start.run()