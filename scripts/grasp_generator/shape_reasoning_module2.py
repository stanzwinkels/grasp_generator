#!/usr/bin/env python

# Ros
import rospy

# Clients
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient
from grasp_generator.server_clients.semantic_client import SemanticClient

# Utils
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    save_pointcloud,
    load_pointcloud,
    filter_full_pointcloud, 
    accuracy_overlap_partial,
    transform_partial_pointcloud_origin,
    region_cylinder,
    cylinder_reasoning,
    save_to_excel
)


from grasp_generator.utils.superquadric_functions import (
    superquadric_overlapping,
    point_cloud_segmentation,
    point_cloud_segmentation_cylinder
)

from grasp_generator.utils.reasoning import PrologFunctionTask

# Libaries
import numpy as np
import ast
import rospkg
import pdb
from pandas  import *
from pyquaternion import Quaternion
import os
# Messages
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import pickle
import joblib


# Visualization
from grasp_generator.visualization.visualization_superquadric import (
    visualize_superquadric,
    visualize_superquadric_cylinder,
    visualize_superquadric_segmentation,
    visualize_gt_pred,
    visualize_pointclouds,
    SingleSuperQuadric,
    visualize_superquadric_true_segmentation
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
    

    def run(self):
        if self.load:
            full_pointcloud, full_name, colors_gt = load_pointcloud(
                self.package_path + "/data/full_pointcloud")


        object_name, ext = os.path.splitext(full_name)
        package_path = self.rospack.get_path("grasp_generator")
        directory_product = package_path + "/data/test_data/"+object_name+"/"

        count = 1
        for filename in os.listdir(directory_product):
            source = o3d.io.read_point_cloud(directory_product+filename)
            partial_pointcloud = np.asarray(source.points)
            colors_pointcloud = np.asarray(source.colors)
            name = filename
            try: 
                print("Opening file number " + str(count) + " with name: " + str(name))

                # 2. superquadric modelling
                pointcloud_msg = point_cloud_to_message(partial_pointcloud)
                multiquadric = MultiquadricClient()
                superquadrics = multiquadric.run(pointcloud_msg)
                superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))

                superquadric = SingleSuperQuadric(superquadrics[:, 0:2], superquadrics[:, 2:5], superquadrics[:, 9:12], superquadrics[:, 5:9])
                superquadric_points = superquadric.coordinates()
                # visualize_superquadric(partial_pointcloud, superquadric_points)

                filt_superquadrics, score = superquadric_overlapping(superquadrics)

                superquadric_filt = SingleSuperQuadric(filt_superquadrics[:, 0:2], filt_superquadrics[:, 2:5], filt_superquadrics[:, 9:12], filt_superquadrics[:, 5:9])
                superquadric_points_filt = superquadric_filt.coordinates()
                visualize_superquadric(partial_pointcloud, superquadric_points_filt)
                
                correct_filt_superquadrics = np.copy(filt_superquadrics)
                correct_filt_superquadrics[:, 2:5] = cylinder_reasoning(correct_filt_superquadrics[:, 0:2], correct_filt_superquadrics[:, 2:5])    

                semantic_classification = SemanticClient()
                shape = semantic_classification.run(correct_filt_superquadrics[0,:5])
                print("SHAPE", shape)

                # prolog_reasoning = PrologFunctionTask()
                # request_ID, region, forms = prolog_reasoning.shape_selection(correct_filt_superquadrics, self.product, self.task)
                print(filt_superquadrics)  
                print("\n")
                print(correct_filt_superquadrics)
                # df = concat([DataFrame(forms), DataFrame(correct_filt_superquadrics)], axis=1)        
                # df = df.drop(columns=[5,6,7,8])
                # df.iloc[:,3:6] = df.iloc[:,3:6]*2
                # df.insert(loc=0, column='Name', value=name)
                # df = df.set_axis(['Name','form','Shape0', 'Shape1', 'Dim1', 'Dim2', 'Dim3', 'Pos1', 'Pos2', 'Pos3'], axis=1, inplace=False)
                # print(df.to_string())

                # if self.save_to_excel:
                #     save_to_excel('data_shapes.xlsx', object_name, df)
                count += 1
                print("\n")

                # if region == "All": 
                #     closest_primitive, distances = point_cloud_segmentation(filt_superquadrics, partial_pointcloud)
                #     visualize_superquadric_segmentation(partial_pointcloud, superquadric_points_filt, closest_primitive)           
                #     origin_partial_pointcloud = transform_partial_pointcloud_origin(self.package_path, partial_pointcloud, name)
                #     pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 
                #     closest_primitive[closest_primitive != request_ID] = 0 #undesired grasp area
                #     closest_primitive[closest_primitive == request_ID] = 1  #desired grasp area
                #     # ground_truth = np.logical_not(ground_truth).astype(int)         # reverse
                #     partial_pred = closest_primitive
                #     accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, origin_partial_pointcloud, partial_pred)
                #     visualize_gt_pred(pointcloud_gt, ground_truth, origin_partial_pointcloud, partial_pred)
                #     visualize_pointclouds(origin_partial_pointcloud, pointcloud_gt)
                        

                # if not (region == "All"): 
                #     region, label = region_cylinder(filt_superquadrics[request_ID-1, 2:5], filt_superquadrics[request_ID-1, 9:12], filt_superquadrics[request_ID-1, 5:9], region, filt_superquadrics, request_ID)            
                #     visualize_superquadric_cylinder(partial_pointcloud, superquadric_points_filt, region)

                #     closest_primitive, distances = point_cloud_segmentation_cylinder(filt_superquadrics, partial_pointcloud, label, region, request_ID)
                #     visualize_superquadric_segmentation(partial_pointcloud, superquadric_points_filt, closest_primitive)
                #     visualize_superquadric_true_segmentation(partial_pointcloud, superquadric_points_filt, closest_primitive)

                #     origin_partial_pointcloud = transform_partial_pointcloud_origin(self.package_path, partial_pointcloud, name)
                
                #     # 5. have ground truth of the full point cloud. 
                #     pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 
                #     closest_primitive[closest_primitive != request_ID] = 0 #undesired grasp area
                #     closest_primitive[closest_primitive == request_ID] = 1  #desired grasp area

                #     # ground_truth = np.logical_not(ground_truth).astype(int)         # reverse
                #     partial_pred = closest_primitive
                #     accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, origin_partial_pointcloud, partial_pred)
                    
                #     # 6. visualize the ground truth and predicted pointcloud 
                #     visualize_gt_pred(pointcloud_gt, ground_truth, origin_partial_pointcloud, partial_pred)
                #     visualize_pointclouds(origin_partial_pointcloud, pointcloud_gt)



            except: 
                print("\n")
                print("COUNT: " + str(count) + " FAILED TO OPEN: " + str(name))
                print("\n")
                count += 1

if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version2")

    main = Main()
    main.run()
