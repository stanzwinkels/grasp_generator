#!/usr/bin/env python

# Ros
import rospy

# Clients
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient
from grasp_generator.server_clients.semantic_client import SemanticClient

# Utils
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    load_pointcloud,
    region_cylinder
)

from grasp_generator.utils.superquadric_functions import (
    superquadric_overlapping,
    point_cloud_segmentation,
    point_cloud_segmentation_cylinder
)

from grasp_generator.utils.reasoning import PrologShapeTask

# Libaries
import numpy as np
import rospkg
import pdb
from pandas  import *
import os


# Visualization
from grasp_generator.visualization.visualization_superquadric import (
    visualize_superquadric,
    SingleSuperQuadric,
    visualize_superquadric_true_segmentation
    )

class Main:
    def __init__(
        self, debug=False, save=False, load=True, superquadric_visualize=True
    ):
        self._nr_products = 1
        self.save_to_excel = False
        self.debug = debug
        self.save = save
        self.load = load
        self.superquadric_visualize = superquadric_visualize
        self.grasp_visualize = False
        self.topic_pointcloud_rgb = "/camera/depth/color/points"
        self.topic_pointcloud_tiago = "/xtion/depth_registered/points"
        # self.topic_aruco_detection = "/aruco_marker_publisher/markers"
        self.task = "Pour"
        self.product = "Limonade"
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")
    

    def run(self):
        if self.load:
            partial_pointcloud, full_name, _ = load_pointcloud(
                self.package_path + "/data/real_data/limonade")

        multiquadric = MultiquadricClient()
        superquadrics = multiquadric.run(point_cloud_to_message(partial_pointcloud))

        superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))
        superquadrics, score = superquadric_overlapping(superquadrics)

        singlesuperquadric = SingleSuperQuadric(superquadrics)
        superquadric_pointcloud = singlesuperquadric.coordinates()
        visualize_superquadric(partial_pointcloud, superquadric_pointcloud)
        
        semantic_classification = SemanticClient()
        semantic_shape = semantic_classification.run(superquadrics[:,:5])

        prolog_reasoning = PrologShapeTask()
        request_ID, grasp_region = prolog_reasoning.shape_selection(superquadrics, self.product, self.task, semantic_shape)

        if grasp_region[0] == "All": 
            pointsegmentation, distances = point_cloud_segmentation(superquadrics, partial_pointcloud, request_ID)
        if not (grasp_region[0] == "All"): 
            cylinder_pointsegmentations, labels = region_cylinder(grasp_region, superquadrics, request_ID)            
            pointsegmentation, distances = point_cloud_segmentation_cylinder(superquadrics, partial_pointcloud, labels, cylinder_pointsegmentations, request_ID)
        visualize_superquadric_true_segmentation(partial_pointcloud, superquadric_pointcloud, pointsegmentation)

        pdb.set_trace()


if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version1")
    main = Main()
    main.run()


