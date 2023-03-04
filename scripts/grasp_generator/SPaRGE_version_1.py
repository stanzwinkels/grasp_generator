#!/usr/bin/env python

# Ros
import rospy
from sensor_msgs.msg import PointCloud2

# Clients
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient
from grasp_generator.server_clients.semantic_client import SemanticClient
from grasp_generator.server_clients.detect_client import detect_grasp_client
from grasp_generator.server_clients.grasp_client import GraspClient

# Utils
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    tiago_pose

)

from grasp_generator.utils.superquadric_functions import (
    superquadric_overlapping,
    point_cloud_segmentation
)

from grasp_generator.utils.reasoning import Reasoning_version1

# Libaries
import numpy as np
import rospkg
from scipy.spatial import KDTree
import pdb

class Main:
    def __init__(
        self, save=False, load=True, superquadric_visualize=True
    ):
        self.topic_pointcloud_tiago = "/xtion/depth_registered/points"
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")
        self.task = 'Handover'


    def run(self):
        raw_pointcloud = rospy.wait_for_message(
            self.topic_pointcloud_tiago, PointCloud2
            )
        scene_pointcloud, object_pointcloud = point_cloud_filter(raw_pointcloud)
        ### Data-driven approach ###
        grasp_poses = detect_grasp_client(raw_pointcloud)
        grasp_positions = np.array([[g.position.x, g.position.y, g.position.z] for g in grasp_poses.grasps])
        
        ### Primitive shape algorithm ###
        # Primitive recovery algorithm
        multiquadric = MultiquadricClient()
        superquadrics = multiquadric.run(point_cloud_to_message(object_pointcloud))
        superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))

        # Overlap removal
        superquadrics, score = superquadric_overlapping(superquadrics)
        superquadrics = superquadrics[:2]

        # Shape classification
        semantic_classification = SemanticClient()
        semantic_shape = semantic_classification.run(superquadrics[:,:5])
 
        ### Logic-based approach ###
        # Prolog reasoning
        prolog_reasoning = Reasoning_version1()
        request_ID = prolog_reasoning.shape_selection(superquadrics, self.task, semantic_shape)

        ### Primitives to pointcloud segmentation ###
        pointsegmentation, distances = point_cloud_segmentation(superquadrics, object_pointcloud, request_ID)

        # Grasp matching 
        object_pointcloudtree = KDTree(object_pointcloud)
        _, ids = object_pointcloudtree.query(grasp_positions)
        id_grasp_pose = np.where(pointsegmentation[ids]==0)[0][0]
        final_grasp_pose = tiago_pose(grasp_poses.grasps[id_grasp_pose], offset=-0.15, pre_grasp=-0.15)

        # Execute grasp TIAGo
        grasp_client = GraspClient("right")
        result = grasp_client.run(final_grasp_pose)

if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version1")

    main = Main()
    main.run()