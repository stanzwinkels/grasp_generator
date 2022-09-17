#!/usr/bin/env python

# Ros
from fileinput import close
from functools import partial
from turtle import pd
import rospy

# Clients
from grasp_generator.server_clients.detect_client import detect_grasp_client
from grasp_generator.server_clients.grasp_client import GraspClient
from grasp_generator.server_clients.aruco_client import ArucoClient
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient

# Utils
from grasp_generator.utils.utils_aruco_marker import detect_aruco_marker, convert_aruco_center, match_mult_aruco
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    tiago_pose,
    save_pointcloud,
    load_pointcloud,
    transform_pose,
)
from grasp_generator.utils.superquadric_functions import (
    grasp_quadric_distance,
    superquadric_overlapping,
)
from grasp_generator.utils.reasoning import PrologFunctionTask

# Libaries
import numpy as np
import ast
import os
import tkinter as tk
import open3d as o3d
from tkFileDialog import askopenfilename
import rospkg
import pdb
# Messages
from sensor_msgs.msg import PointCloud2

# Visualization
from grasp_generator.visualization.visualization_superquadric import *

# selection menu
from grasp_generator.utils.menu import start


class Main:
    def __init__(
        self, debug=False, save=False, load=False, superquadric_visualize=True
    ):
        self._nr_products = 1

        self.debug = debug
        self.save = save
        self.load = load
        self.superquadric_visualize = superquadric_visualize
        self.grasp_visualize = False

        self.topic_pointcloud_rgb = "/camera/depth/color/points"
        self.topic_pointcloud_tiago = "/xtion/depth_registered/points"
        self.topic_aruco_detection = "/aruco_marker_publisher/markers"

        self.aruco_id = 444

        self.task = "Pour"
        self.product = "Limonade"

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")

    def run(self):
        # 1. Aruco pose generation
        aruco_poses = detect_aruco_marker(
            self._nr_products, self.topic_aruco_detection)  
        aruco_poses = convert_aruco_center(aruco_poses)    

        aruco_client = ArucoClient()  
        aruco_client.run(aruco_poses[self.aruco_id])  

        for i in aruco_poses:
            aruco_poses[i].pose.pose = transform_pose(aruco_poses[i].pose.pose, "map", "xtion_rgb_optical_frame") 


        if self.load:
            partial_pointcloud = load_pointcloud(
                self.package_path + "/data/" + self.product)

        if not self.load:
            pointcloud = rospy.wait_for_message(
                self.topic_pointcloud_tiago, PointCloud2
                ) 
            grasp_poses = detect_grasp_client(pointcloud)
            partial_pointcloud = point_cloud_filter(pointcloud, self.debug)

            grasp_list = match_mult_aruco(aruco_poses, grasp_poses, threshold_dist = 0.15)

            if self.save:
                save_pointcloud(
                    partial_pointcloud,
                    self.product,
                    self.package_path + "/data/" + self.product + "/")
        

        # 2. superquadric modelling
        pointcloud_msg = point_cloud_to_message(partial_pointcloud)
        multiquadric = MultiquadricClient()
        superquadrics = multiquadric.run(pointcloud_msg)
        superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))
        if self.superquadric_visualize:
            visualize_superquadric(partial_pointcloud, superquadrics)
        filt_superquadrics, score = superquadric_overlapping(superquadrics)
        if self.superquadric_visualize:
            visualize_superquadric(partial_pointcloud, filt_superquadrics)


        # 3. prolog reasoning
        prolog_reasoning = PrologFunctionTask()
        request_ID = prolog_reasoning.shape_selection(
            filt_superquadrics, self.product, self.task)
        # request_ID = ast.literal_eval(request_ID[0])


        if not self.load:
            # 4. primitive - grasp matching
            closest_primitives, distances = grasp_quadric_distance(
                filt_superquadrics, grasp_list)
        
            # 5. grasp performance
            final_grasp = grasp_list[np.where(distances == min(distances[closest_primitives==1]))[0][0]]
            grasp_pose = tiago_pose(final_grasp[1])
            grasp_client = GraspClient("right")
            result = grasp_client.run(grasp_pose)

        print("end of task")


if __name__ == "__main__":
    rospy.init_node("main")

    start = Main()
    start.run()
