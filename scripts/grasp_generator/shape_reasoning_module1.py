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
    load_pointcloud,
    filter_full_pointcloud, 
    accuracy_overlap_partial,
    transform_partial_pointcloud_origin,
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
import time

# Messages
import open3d as o3d

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
        resulting_scores = []
        time_scores = []
        for filename in os.listdir(directory_product):
            source = o3d.io.read_point_cloud(directory_product+filename)
            partial_pointcloud = np.asarray(source.points)
            colors_pointcloud = np.asarray(source.colors)
            try: 
                print("Opening file number " + str(count) + " with name: " + str(filename))
                start_time = time.time()
                time_recovery_primitive = time.time()
                multiquadric = MultiquadricClient()
                superquadrics = multiquadric.run(point_cloud_to_message(partial_pointcloud))
                time_recovery_primitive = (time.time() - time_recovery_primitive)
                
                time_overlap_removal = time.time()
                superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))
                superquadrics, score = superquadric_overlapping(superquadrics)
                time_overlap_removal = (time.time() - time_overlap_removal)

                # singlesuperquadric = SingleSuperQuadric(superquadrics)
                # superquadric_pointcloud = singlesuperquadric.coordinates()
                # visualize_superquadric(partial_pointcloud, superquadric_pointcloud)
                
                time_shape_classification = time.time()
                semantic_classification = SemanticClient()
                semantic_shape = semantic_classification.run(superquadrics[:,:5])
                time_shape_classification = (time.time() - time_shape_classification)

                # df = concat([DataFrame(semantic_shape), DataFrame(superquadrics)], axis=1)        
                # df = df.drop(columns=[5,6,7,8])
                # df.iloc[:,3:6] = df.iloc[:,3:6]*2
                # df.insert(loc=0, column='Name', value=filename)
                # df = df.set_axis(['Name','semantic_shape','Shape0', 'Shape1', 'Dim1', 'Dim2', 'Dim3', 'Pos1', 'Pos2', 'Pos3'], axis=1, inplace=False)
                # print(df.to_string())

                time_reasoning = time.time()
                prolog_reasoning = PrologShapeTask()
                request_ID, grasp_region = prolog_reasoning.shape_selection(superquadrics, self.product, self.task, semantic_shape)
                time_reasoning = (time.time() - time_reasoning)

                time_segmentation = time.time()
                if grasp_region[0] == "All": 
                    pointsegmentation, distances = point_cloud_segmentation(superquadrics, partial_pointcloud, request_ID)
                if not (grasp_region[0] == "All"): 
                    cylinder_pointsegmentations, labels = region_cylinder(grasp_region, superquadrics, request_ID)            
                    pointsegmentation, distances = point_cloud_segmentation_cylinder(superquadrics, partial_pointcloud, labels, cylinder_pointsegmentations, request_ID)
                time_segmentation = (time.time() - time_segmentation)
                total_time = (time.time() - start_time)
                
                # origin_partial_pointcloud = transform_partial_pointcloud_origin(self.package_path, partial_pointcloud, filename)
                # pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 

                # visualize_superquadric_cylinder(partial_pointcloud, superquadric_pointcloud, cylinder_pointsegmentations)
                # visualize_superquadric_true_segmentation(partial_pointcloud, superquadric_pointcloud, pointsegmentation)
                # visualize_superquadric_segmentation(partial_pointcloud, superquadric_pointcloud, pointsegmentation)
                # visualize_gt_pred(pointcloud_gt, ground_truth, origin_partial_pointcloud, pointsegmentation)
                # visualize_pointclouds(origin_partial_pointcloud, pointcloud_gt)
                
                # accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, origin_partial_pointcloud, pointsegmentation)
                # resulting_scores.append([accuracy,true_positive_rate])
                count += 1
                comp_total_time = time_recovery_primitive + time_overlap_removal + time_shape_classification + time_reasoning + time_segmentation
                time_scores.append([filename, time_recovery_primitive, time_overlap_removal, time_shape_classification, time_reasoning, time_segmentation, total_time, comp_total_time])

            except: 
                print("\n")
                print("COUNT: " + str(count) + " FAILED TO OPEN: " + str(filename))
                print("\n")
                count += 1

            df_data = np.array(resulting_scores)
            scores_df = DataFrame(df_data)
            scores_df.to_csv('resulting_scores_version1.csv', index=False)

            df_time = np.array(time_scores)
            times_df = DataFrame(df_time)
            times_df.to_csv('time_scores', index=False)



if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version1")

    main = Main()
    main.run()


