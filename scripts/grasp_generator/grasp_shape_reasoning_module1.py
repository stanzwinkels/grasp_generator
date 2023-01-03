#!/usr/bin/env python

# Ros
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

# Clients
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient
from grasp_generator.server_clients.semantic_client import SemanticClient
from grasp_generator.server_clients.detect_client import detect_grasp_client


# Utils
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    load_pointcloud,
    filter_full_pointcloud, 
    accuracy_overlap_partial,
    transform_partial_pointcloud_origin,
    region_cylinder, 
    create_dict,
    filter_full_pointcloud,
    gpd_grasp_succes,
    reasoning_grasp_succes,
    multiple_grasp_poses,
    single_grasp_poses
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
import time
from scipy.spatial import KDTree
import json
from pyquaternion import Quaternion
import time

# Messages
import open3d as o3d
from tf.transformations import quaternion_from_matrix

# Visualization
from grasp_generator.visualization.visualization_superquadric import (
    visualize_superquadric,
    visualize_superquadric_cylinder,
    visualize_superquadric_segmentation,
    visualize_gt_pred,
    visualize_pointclouds,
    SingleSuperQuadric,
    visualize_superquadric_true_segmentation,
    visualize_grasp_point,
    visualize_scene_pointcloud
    )

# selection menu
from grasp_generator.utils.menu import start





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
    

    def convertCloudFromOpen3dToRos(self, open3d_cloud, frame_id="odom"):
        FIELDS_XYZ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        FIELDS_XYZRGB = FIELDS_XYZ + \
            [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

        # Bit operations
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        
        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "xtion_rgb_optical_frame"

        # Set "fields" and "cloud_data"
        points=np.asarray(open3d_cloud.points)
        if not open3d_cloud.colors: # XYZ only
            fields=FIELDS_XYZ
            cloud_data=points
        else: # XYZ + RGB
            fields=FIELDS_XYZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.
            colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
            colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
            cloud_data=np.c_[points, colors]
        # create ros_cloud
        return pc2.create_cloud(header, fields, cloud_data)

    def run(self):
        if self.load:
            full_pointcloud, full_name, colors_gt = load_pointcloud(
                self.package_path + "/data/full_pointcloud")

        object_name, ext = os.path.splitext(full_name)
        package_path = self.rospack.get_path("grasp_generator")
        directory_product = package_path + "/data/GPD_data/"+object_name+"/"
        directory_scene = package_path + "/data/GPD_data/scene/"+object_name+"/"

        count = 1
        resulting_scores = []
        time_scores = []

        for filename in os.listdir(directory_product):
            print("\n")
            print("STARTING TO COUNT", count)
            filename_ext, ext = os.path.splitext(filename)

            source_partial = o3d.io.read_point_cloud(directory_product+filename)
            partial_pointcloud = np.asarray(source_partial.points)
            
            source_scene =  o3d.io.read_point_cloud(directory_scene+"full_" + filename)
            scene_pointcloud = np.asarray(source_scene.points)
            cloud_message = self.convertCloudFromOpen3dToRos(source_scene, frame_id="odom")

            grasp_poses = detect_grasp_client(cloud_message)
            grasp_positions, ids_grasp, gpd_predicted_area, predicted_grasps_GPD = gpd_grasp_succes(grasp_poses, partial_pointcloud, full_pointcloud, package_path, filename_ext, colors_gt)

            # print("predicted GPD: ",gpd_predicted_area)
            # print("Matched GPD: ", predicted_grasps_GPD)
            # print("SET GPD", set(predicted_grasps_GPD))

            try: 
                print("Opening file number " + str(count) + " with name: " + str(filename))
                multiquadric = MultiquadricClient()
                superquadrics = multiquadric.run(point_cloud_to_message(partial_pointcloud))
                
                superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))
                superquadrics, score = superquadric_overlapping(superquadrics)

                singlesuperquadric = SingleSuperQuadric(superquadrics)
                superquadric_pointcloud = singlesuperquadric.coordinates()
                # visualize_superquadric(partial_pointcloud, superquadric_pointcloud)
                
                semantic_classification = SemanticClient()
                semantic_shape = semantic_classification.run(superquadrics[:,:5])

                prolog_reasoning = PrologShapeTask()
                request_ID, grasp_region = prolog_reasoning.shape_selection(superquadrics, self.product, self.task, semantic_shape)

                if grasp_region[0] == "All": 
                    pointsegmentation, distances = point_cloud_segmentation(superquadrics, partial_pointcloud, request_ID)
                if not (grasp_region[0] == "All"): 
                    cylinder_pointsegmentations, labels = region_cylinder(grasp_region, superquadrics, request_ID)            
                    pointsegmentation, distances = point_cloud_segmentation_cylinder(superquadrics, partial_pointcloud, labels, cylinder_pointsegmentations, request_ID)
                
                # origin_partial_pointcloud = transform_partial_pointcloud_origin(self.package_path, partial_pointcloud, filename)
                # pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 

                correct_grasps = []
                correct_grasps_id = []
                for id, index_id in enumerate(ids_grasp): 
                    # desired_color = 'desired grasp = 0, undesired grasp = 1' 
                    if pointsegmentation[index_id] == 0:
                        correct_grasps.append(index_id)
                        correct_grasps_id.append(id)

                # Error: found_positions does not exist
                if grasp_positions.any(): 
                    found_positions = []
                    for correct_grasp_id in correct_grasps_id: 
                        found_positions.append([grasp_poses.grasps[correct_grasp_id].position.x, grasp_poses.grasps[correct_grasp_id].position.y, grasp_poses.grasps[correct_grasp_id].position.z])
                    found_positions = np.array(found_positions)

                    reasoning_predicted_area, gt_predicted_area, id_grasps_reasoning, index_partial_pointcloud = reasoning_grasp_succes(correct_grasps, correct_grasps_id, found_positions, full_pointcloud, package_path, filename_ext, colors_gt)
                
                    print("PREDICTED AREA BY SEGMENTATION", pointsegmentation[index_partial_pointcloud])
                    ############ RESULTS #############
                    print("Matched GPD: ", predicted_grasps_GPD)
                    print("SET GPD", set(predicted_grasps_GPD))
                    print("GPD predicted color: ", gpd_predicted_area)
                    print("Reasoning predicted: ", gt_predicted_area)
                    visualize_superquadric_true_segmentation(partial_pointcloud, superquadric_pointcloud, pointsegmentation)


                # Add here
                # Visualize generated grasps
                    # 1. plots generated grasps
                    # 2. visualize scene_pointcloud
                    # 3. visualize selected grasp GPD
                    # 4. visualize select grasp reasoning version

                    left_lines, right_lines, middle_lines, hand_lines = multiple_grasp_poses(grasp_poses)
                    gpd_left_line, gpd_right_line, gpd_middle_line, gpd_hand_line = single_grasp_poses(grasp_poses.grasps[0])
                    reasoning_left_line, reasoning_right_line, reasoning_middle_line, reasoning_hand_line = single_grasp_poses(grasp_poses.grasps[id_grasps_reasoning])
                    visualize_scene_pointcloud(scene_pointcloud, left_lines, right_lines, middle_lines, hand_lines, gpd_left_line, gpd_right_line, gpd_middle_line, gpd_hand_line, reasoning_left_line, reasoning_right_line, reasoning_middle_line, reasoning_hand_line)

                    pdb.set_trace()


                # scene_pointcloud
                # GPD: grasp_poses.grasps[0]
                # Reasoning: grasp_poses.grasps[id_grasps_reasoning]

                else: 
                    print("NO GRASP PREDICTED AT DESIRED LOCATION....")
                
                count +=1


            # Add here method that appends



                # visualize_superquadric_cylinder(partial_pointcloud, superquadric_pointcloud, cylinder_pointsegmentations)
                # visualize_superquadric_segmentation(partial_pointcloud, superquadric_pointcloud, pointsegmentation)
                # visualize_gt_pred(pointcloud_gt, ground_truth, origin_partial_pointcloud, pointsegmentation)
                # visualize_pointclouds(origin_partial_pointcloud, pointcloud_gt)
                
                # accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, origin_partial_pointcloud, pointsegmentation)
                # resulting_scores.append([accuracy,true_positive_rate])
                # count += 1

            except: 
                print("\n")
                print("COUNT: " + str(count) + " FAILED TO OPEN: " + str(filename_ext))
                print("\n")
                count += 1


if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version1")

    main = Main()
    main.run()


