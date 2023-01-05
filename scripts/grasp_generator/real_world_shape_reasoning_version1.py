#!/usr/bin/env python

# Ros
import rospy

# Clients
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient
from grasp_generator.server_clients.semantic_client import SemanticClient
from grasp_generator.server_clients.grasp_client import GraspClient


# Utils
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    load_pointcloud,
    region_cylinder,
    multiple_filtered_grasp_poses,
    single_grasp_poses,
    tiago_pose,
    save_pointcloud_world
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
from scipy.spatial import KDTree
import time

# Messages
from sensor_msgs.msg import PointCloud2
from grasp_generator.server_clients.detect_client import detect_grasp_client
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter

# Visualization
from grasp_generator.visualization.visualization_superquadric import (
    visualize_superquadric,
    SingleSuperQuadric,
    visualize_superquadric_true_segmentation,
    visualize_grasps_pointcloud,
    visualize_grasp_gpd_pointcloud
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
        pointcloud = rospy.wait_for_message(
            self.topic_pointcloud_tiago, PointCloud2
            )

        scene_pointcloud, partial_pointcloud = point_cloud_filter(pointcloud, self.debug)
        grasp_poses = detect_grasp_client(pointcloud)
        grasp_positions = np.array([[g.position.x, g.position.y, g.position.z] for g in grasp_poses.grasps])

        partial_tree = KDTree(partial_pointcloud)
        dist, ids = partial_tree.query(grasp_positions)

        index_filtered = [i for i in range(len(dist)) if dist[i] < 0.01]     # Threshold 0.005
        ids_partial_filtered = ids[index_filtered]
        filtered_grasps = [grasp_poses.grasps[x] for x in index_filtered]
        gpd_final_grasp = filtered_grasps[0]
        grasp_pose_gpd = tiago_pose(gpd_final_grasp)

        left_lines, right_lines, middle_lines, hand_lines = multiple_filtered_grasp_poses(filtered_grasps)
        left_line_gpd, right_line_gpd, middle_line_gpd, hand_line_gpd = single_grasp_poses(gpd_final_grasp)
        visualize_grasp_gpd_pointcloud(scene_pointcloud, left_line_gpd, right_line_gpd, middle_line_gpd, hand_line_gpd, grasp_pose_gpd)

        grasp_gpd = False
        if grasp_gpd:
            query_grasp = raw_input("Perform grasp with Right (r) or Left (l), enter r/l: ")
            if query_grasp == 'l':
                grasp_client = GraspClient("left")
            else:
                grasp_client = GraspClient("right")
            result = grasp_client.run(grasp_pose_gpd)

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

        grasp_id = np.array(np.where(ids == (ids_partial_filtered[(pointsegmentation == 0)[ids_partial_filtered] == True])[0]))
        reasoning_final_grasp = grasp_poses.grasps[grasp_id[0][0]]

        left_line_reasoning, right_line_reasoning, middle_line_reasoning, hand_line_reasoning = single_grasp_poses(reasoning_final_grasp)
        visualize_grasps_pointcloud(scene_pointcloud, left_lines, right_lines, middle_lines, hand_lines, left_line_gpd, right_line_gpd, middle_line_gpd, hand_line_gpd, left_line_reasoning, right_line_reasoning, middle_line_reasoning, hand_line_reasoning, grasp_pose_gpd, grasp_pose_reasoning )

        grasp_reasoning = True
        if grasp_reasoning: 
            grasp_pose_reasoning = tiago_pose(reasoning_final_grasp)        
            grasp_client = GraspClient("right")
            result = grasp_client.run(grasp_pose_reasoning)

        logask = raw_input("Save current pointcoud? (y/n): ")
        if logask == 'y':
            time_stamp = time.time()
            save_pointcloud_world("scene_pointcloud", scene_pointcloud, time_stamp)
            save_pointcloud_world("partial_pointcloud", partial_pointcloud, time_stamp)
        else: 
            print("not saving\n")


if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version1")
    main = Main()
    main.run()


        # if self.load:
        #     partial_pointcloud, full_name, _ = load_pointcloud(
        #         self.package_path + "/data/real_data/limonade")
