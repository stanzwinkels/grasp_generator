#!/usr/bin/env python

# Ros
import rospy

# Utils
from grasp_generator.utils.standard_functions import (
    load_pointcloud,
    filter_full_pointcloud, 
    accuracy_overlap)

# Libaries
import numpy as np
import rospkg
import pdb

class Main:
    def __init__(
        self, debug=False, save=False, load=True, superquadric_visualize=True
    ):
        self.load = load
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")


    def run(self):
        if self.load:
            pointcloud_gt, name, colors_gt = load_pointcloud(
                self.package_path + "/data/full_pointcloud")

        if self.load:
            partial_pointcloud, name, _ = load_pointcloud(
                self.package_path + "/data/full_pointcloud_color")


        pointcloud_gt, ground_truth = filter_full_pointcloud(pointcloud_gt, colors_gt) 
        partial_pointcloud_color = ground_truth[:len(partial_pointcloud)]            ## PREDICTED BY SHAPE PRIMITIVE FRAMEWORK! ##
        accuracy, true_positive_rate = accuracy_overlap(pointcloud_gt, ground_truth, partial_pointcloud, partial_pointcloud_color)
        
        # visualize_pointclouds(full_pointcloud, orig_pointcloud)


if __name__ == "__main__":
    rospy.init_node("ground_truth")

    start = Main()
    start.run()
