#!/usr/bin/env python

# Ros
from functools import partial
import rospy

# Clients
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient

# Utils
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    save_pointcloud,
    load_pointcloud,
    load_data,
    create_dict, 
    filter_full_pointcloud, 
    accuracy_overlap_partial

)
from grasp_generator.utils.superquadric_functions import (
    superquadric_overlapping,
    point_cloud_segmentation,
)
from grasp_generator.utils.reasoning import PrologShapeTask

# Libaries
import numpy as np
import ast
import rospkg
import pdb
import os
import json
from pyquaternion import Quaternion


# Messages
from sensor_msgs.msg import PointCloud2

# Visualization
from grasp_generator.visualization.visualization_superquadric import (
    visualize_superquadric,
    visualize_superquadric_segmentation, 
    visualize_pointclouds,
    visualize_gt_pred
)

# selection menu
from grasp_generator.utils.menu import start


class Main:
    def __init__(
        self, debug=False, save=False, load=True, superquadric_visualize=True
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

        self.directory_map_orientation_camera = self.package_path + "/data/parameter_tuning/bottle/poses/map_orientation_camera.json"
        self.directory_map_position_camera = self.package_path + "/data/parameter_tuning/bottle/poses/map_position_camera.json"
        self.directory_map_orientation_product = self.package_path + "/data/parameter_tuning/bottle/poses/map_orientation_product.json"
        self.directory_map_position_product = self.package_path + "/data/parameter_tuning/bottle/poses/map_position_product.json"
        self.directory_camera_position_product = self.package_path + "/data/parameter_tuning/bottle/poses/camera_position_product.json"
        self.directory_product_orientation_camera = self.package_path + "/data/parameter_tuning/bottle/poses/product_orientation_camera.json"


    def run(self):
        if self.load:
            partial_pointcloud, name, _ = load_pointcloud(
                self.package_path + "/data/parameter_tuning/bottle")


        if self.load:
            full_pointcloud, _, colors_gt = load_pointcloud(
                self.package_path + "/data/full_pointcloud")

        if not self.load:
            pointcloud = rospy.wait_for_message(
                self.topic_pointcloud_tiago, PointCloud2
                ) 
            partial_pointcloud = point_cloud_filter(pointcloud, self.debug)

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
        # if self.superquadric_visualize:
        #     visualize_superquadric(partial_pointcloud, superquadrics)
        filt_superquadrics, score = superquadric_overlapping(superquadrics)
        # if self.superquadric_visualize:
        #     visualize_superquadric(partial_pointcloud, filt_superquadrics)

        # 3. prolog reasoning
        prolog_reasoning = PrologShapeTask()
        request_ID = prolog_reasoning.shape_selection(
            filt_superquadrics, self.product, self.task)

        request_ID = 2

        # 5. Apply segmentation based on the created geometric shape. 
        closest_primitive, distances = point_cloud_segmentation(filt_superquadrics, partial_pointcloud)
        if self.superquadric_visualize:
            visualize_superquadric_segmentation(closest_primitive, partial_pointcloud, filt_superquadrics)

        # 6 Rotate partial pointcloud back to the original full pointcloud
        object_name, ext = os.path.splitext(name)
        
        camera_position_product = json.load(open(self.directory_camera_position_product))
        product_orientation_camera = json.load(open(self.directory_product_orientation_camera))
        camera_position_product = create_dict(camera_position_product)[object_name]
        product_orientation_camera = create_dict(product_orientation_camera)[object_name]
        product_orientation_camera = Quaternion(product_orientation_camera[0], product_orientation_camera[1], product_orientation_camera[2], product_orientation_camera[3])

        org_pointcloud = []
        org_cam_partial_pointcloud = partial_pointcloud - camera_position_product
        for point in org_cam_partial_pointcloud:
            obj_frame_pointcloud = product_orientation_camera.rotate(point)
            map_frame = obj_frame_pointcloud.tolist()
            org_pointcloud.append(map_frame)
        org_partial_pointcloud = np.array(org_pointcloud)

        # 1. have ground truth of the full point cloud. 
        pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 
        partial_pointcloud_color = ground_truth[:len(org_partial_pointcloud)]            ## PREDICTED BY SHAPE PRIMITIVE FRAMEWORK! ##
        closest_primitive[closest_primitive != request_ID] = 1  #undesired grasp area
        closest_primitive[closest_primitive == request_ID] = 0  #desired gras area

        ground_truth = np.logical_not(ground_truth).astype(int)         # reverse
        partial_pred = closest_primitive
        accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, org_partial_pointcloud, partial_pred)
        
        # visualize the ground truth and predicted pointcloud coincide. 
        visualize_gt_pred(pointcloud_gt, ground_truth, org_partial_pointcloud, partial_pred)
        visualize_pointclouds(org_partial_pointcloud, pointcloud_gt)



if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version1")

    start = Main()
    start.run()
