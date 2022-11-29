#!/usr/bin/env python

# Ros
import rospy

# Clients
from grasp_generator.server_clients.multiquadric_client import MultiquadricClient

# Utils
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter
from grasp_generator.utils.standard_functions import (
    point_cloud_to_message,
    save_pointcloud,
    load_pointcloud,
    filter_full_pointcloud, 
    accuracy_overlap_partial,
    transform_partial_pointcloud_origin,
    cylinder_reasoning,
    cube_segmentation,
    surface_cylinder

)
from grasp_generator.utils.superquadric_functions import (
    superquadric_overlapping,
    point_cloud_segmentation,
)
from grasp_generator.utils.reasoning import PrologShapeTask

# Libaries
import numpy as np
import rospkg
import pdb
from pandas  import *
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

# from grasp_generator.visualization.visualization_mesh_cube_cylinder import (
#     visualize_cube
# )

# selection menu
from grasp_generator.utils.menu import start


class Main:
    def __init__(
        self, debug=False, save=False, load=True, superquadric_visualize=True
    ):
        self.debug = debug
        self.save = save
        self.load = load
        self.superquadric_visualize = superquadric_visualize

        self.topic_pointcloud_rgb = "/camera/depth/color/points"
        self.topic_pointcloud_tiago = "/xtion/depth_registered/points"
        # self.topic_aruco_detection = "/aruco_marker_publisher/markers"

        self.task = "Pour"
        self.product = "Limonade"

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")


    def run(self):
        if self.load:
            partial_pointcloud, name, _ = load_pointcloud(
                self.package_path + "/data/test_data")

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
        

        # 1. superquadric modelling
        pointcloud_msg = point_cloud_to_message(partial_pointcloud)
        multiquadric = MultiquadricClient()
        superquadrics = multiquadric.run(pointcloud_msg)
        superquadrics = np.reshape(superquadrics.quadrics, (-1, 12))

        # if self.superquadric_visualize:
        #     visualize_superquadric(partial_pointcloud, superquadrics)
        filt_superquadrics, score = superquadric_overlapping(superquadrics)
        if self.superquadric_visualize:
            visualize_superquadric(partial_pointcloud, filt_superquadrics)

        # 2. prolog reasoning
        prolog_reasoning = PrologShapeTask()
        request_ID, forms = prolog_reasoning.shape_selection(
            filt_superquadrics, self.product, self.task)

        pdb.set_trace()
        surface_cylinder(filt_superquadrics[request_ID, 2:5], filt_superquadrics[request_ID, 5:8], filt_superquadrics[request_ID, 8:12])
        pdb.set_trace()

        df = concat([DataFrame(forms), DataFrame(filt_superquadrics)], axis=1)        
        df = df.drop(columns=[5,6,7,8])
        df.iloc[:,3:6] = df.iloc[:,3:6]*2
        df = df.set_axis(['form','Shape0', 'Shape1', 'Dim1', 'Dim2', 'Dim3', 'Pos1', 'Pos2', 'Pos3'], axis=1, inplace=False)
        print(df.to_string())
        pdb.set_trace()

        # 3. Segmentation of partial pointcloud 
        closest_primitive, distances = point_cloud_segmentation(filt_superquadrics, partial_pointcloud)
        if self.superquadric_visualize:
            visualize_superquadric_segmentation(closest_primitive, partial_pointcloud, filt_superquadrics)
        origin_partial_pointcloud = transform_partial_pointcloud_origin(self.package_path, partial_pointcloud, name)

        # 4. have ground truth of the full point cloud. 
        pointcloud_gt, ground_truth = filter_full_pointcloud(full_pointcloud, colors_gt) 
        closest_primitive[closest_primitive != request_ID] = 0 #undesired grasp area
        closest_primitive[closest_primitive == request_ID] = 1  #desired grasp area

        ground_truth = np.logical_not(ground_truth).astype(int)         # reverse
        partial_pred = closest_primitive
        accuracy, true_positive_rate = accuracy_overlap_partial(pointcloud_gt, ground_truth, origin_partial_pointcloud, partial_pred)
        

        # 5. visualize the ground truth and predicted pointcloud 
        visualize_gt_pred(pointcloud_gt, ground_truth, origin_partial_pointcloud, partial_pred)
        visualize_pointclouds(origin_partial_pointcloud, pointcloud_gt)



if __name__ == "__main__":
    rospy.init_node("shape_reasoning_version1")

    main = Main()
    main.run()



# PUT THIS IN ANOTHER PYTHON FILE!!!
        # 2.1 reasoning about the height etc. 
        # height, diameters = cylinder_reasoning(filt_superquadrics[0,0:2], filt_superquadrics[0,2:5])
        # pdb.set_trace()

        # mesh_xy, mesh_xz, mesh_yz, mesh_xy_neg, mesh_xz_neg, mesh_yz_neg = cube_segmentation(filt_superquadrics[0,2:5], filt_superquadrics[0,5:8], Quaternion(filt_superquadrics[0,8:12]))
        # visualize_cube(mesh_xy, mesh_xz, mesh_yz, mesh_xy_neg, mesh_xz_neg, mesh_yz_neg, superquadric)
        
        # visualize both. 