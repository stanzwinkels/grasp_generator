#!/usr/bin/env python

# Ros
import rospy

# Utils
from grasp_generator.utils.standard_functions import (
    load_pointcloud,
    filter_full_pointcloud
    )

# Libaries
import numpy as np
import rospkg
import pdb


def accuracy_overlap(pointcloud_gt, ground_truth, partial_pointcloud, partial_pointcloud_color):
    tree = KDTree(pointcloud_gt)
    dist, ids = tree.query(partial_pointcloud, k=1)
    tp, fp, tn, fn = [], [], [], []
    accuracy = []

    for idx_color, color in enumerate(partial_pointcloud_color):
        if color == 0: 
            if color == ground_truth[ids[idx_color]]:
                tp.append(True)         # the predicted value is black, and it should be black
            else:
                fp.append(True)         # the predicted value is black, but it should have been red
        elif color == 1: 
            if color == ground_truth[ids[idx_color]]:
                tn.append(True)         # the predicted value is red, and it should be red
            else: 
                fn.append(True)         # the predicted value is red, but it should be black

    accuracy = float(sum(tp)+sum(tn))/len(partial_pointcloud_color)
    true_positive_rate = float(sum(tp))/(sum(tp)+sum(fp))
    print("Accuracy = " + str(accuracy*100) + "%")
    print("True positive rate = " + str(true_positive_rate*100) + "%")
    return accuracy, true_positive_rate


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
