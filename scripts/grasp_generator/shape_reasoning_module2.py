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
            try: 
                print("Opening file number " + str(count) + " with name: " + str(filename))
                multiquadric = MultiquadricClient()
                superquadrics = multiquadric.run(point_cloud_to_message(partial_pointcloud))
                superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))
                superquadrics, score = superquadric_overlapping(superquadrics)
                        
                singlesuperquadric = SingleSuperQuadric(superquadrics)
                superquadric_pointcloud = singlesuperquadric.coordinates()
                visualize_superquadric(partial_pointcloud, superquadric_pointcloud)
                
                semantic_classification = SemanticClient()
                semantic_shape = semantic_classification.run(superquadrics[:,:5])

                prolog_reasoning = PrologFunctionTask()
                request_ID, grasp_region = prolog_reasoning.shape_selection(superquadrics, self.product, self.task, semantic_shape)

                df = concat([DataFrame(semantic_shape), DataFrame(superquadrics)], axis=1)        
                df = df.drop(columns=[5,6,7,8])
                df.iloc[:,3:6] = df.iloc[:,3:6]*2
                df.insert(loc=0, column='Name', value=filename)
                df = df.set_axis(['Name','semantic_shape','Shape0', 'Shape1', 'Dim1', 'Dim2', 'Dim3', 'Pos1', 'Pos2', 'Pos3'], axis=1, inplace=False)
                print(df.to_string())
                count += 1

                if grasp_region == "All": 
                    closest_primitive, distances = point_cloud_segmentation(superquadrics, partial_pointcloud)
                    visualize_superquadric_segmentation(partial_pointcloud, superquadrics, closest_primitive)           
                    origin_partial_pointcloud = transform_partial_pointcloud_origin(self.package_path, partial_pointcloud, filename)
                    pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 
                    closest_primitive[closest_primitive != request_ID] = 0 #undesired grasp area
                    closest_primitive[closest_primitive == request_ID] = 1  #desired grasp area
                    # ground_truth = np.logical_not(ground_truth).astype(int)         # reverse
                    accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, origin_partial_pointcloud, closest_primitive)
                    visualize_gt_pred(pointcloud_gt, ground_truth, origin_partial_pointcloud, closest_primitive)
                    visualize_pointclouds(origin_partial_pointcloud, pointcloud_gt)
                        

                if not (grasp_region == "All"): 
                    # superquadrics[:, 2:5] = cylinder_reasoning(superquadrics)  # but also include rotation as well!    

                    cylinder_pointsegmentation, label = region_cylinder(superquadrics[request_ID-1,:], grasp_region, superquadrics, request_ID)            
                    visualize_superquadric_cylinder(partial_pointcloud, superquadric_pointcloud, cylinder_pointsegmentation)

                    # closest_primitive, distances = point_cloud_segmentation_cylinder(superquadrics, partial_pointcloud, label, cylinder_pointsegmentation, request_ID)
                    # visualize_superquadric_segmentation(partial_pointcloud, superquadric_pointcloud, closest_primitive)
                    # visualize_superquadric_true_segmentation(partial_pointcloud, superquadric_pointcloud, closest_primitive)
                    # origin_partial_pointcloud = transform_partial_pointcloud_origin(self.package_path, partial_pointcloud, filename)
                
                    # # 5. have ground truth of the full point cloud. 
                    # pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 
                    # closest_primitive[closest_primitive != request_ID] = 0 #undesired grasp area
                    # closest_primitive[closest_primitive == request_ID] = 1  #desired grasp area

                    # # ground_truth = np.logical_not(ground_truth).astype(int)         # reverse
                    # accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, origin_partial_pointcloud, closest_primitive)
                    
                    # # 6. visualize the ground truth and predicted pointcloud 
                    # visualize_gt_pred(pointcloud_gt, ground_truth, origin_partial_pointcloud, closest_primitive)
                    # visualize_pointclouds(origin_partial_pointcloud, pointcloud_gt)


            except: 
                print("\n")
                print("COUNT: " + str(count) + " FAILED TO OPEN: " + str(filename))
                print("\n")
                count += 1


                # singlesuperquadric = SingleSuperQuadric(superquadrics)
                # superquadric_pointcloud = singlesuperquadric.coordinates()
                # # visualize_superquadric(partial_pointcloud, superquadric_pointcloud)



    
                




                # if self.save_to_excel:
                #     save_to_excel('data_shapes.xlsx', object_name, df)

 


if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version2")

    main = Main()
    main.run()
