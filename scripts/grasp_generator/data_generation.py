#!/usr/bin/env python

# Ros
import rospy

# Utils
from grasp_generator.utils.point_cloud_filtering import point_cloud_filter
from grasp_generator.utils.standard_functions import (
    save_pointcloud,
    load_pointcloud
)

# Libaries
import os
import rospkg
import csv
import pdb
import pickle

# Messages
from sensor_msgs.msg import PointCloud2
from gazebo_msgs.srv import GetModelState

# Visualization
from grasp_generator.visualization.visualization_superquadric import visualize_superquadric


class Main:
    def __init__(
        self, debug=True, save=True,
    ):
        self.debug = debug
        self.save = save
        self.running = True

        self.topic_pointcloud_rgb = "/camera/depth/color/points"
        self.topic_pointcloud_tiago = "/xtion/depth_registered/points"

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")

        self.product = "wrench2"

    def product_pose(self):
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(self.product, 'map')
        return resp_coordinates.pose

    def run(self):
        # directory = self.package_path + "/data/simulation_data/" + self.product
        # if not os.path.exists(directory):
        #     os.makedirs(directory)

        while self.running: 
            pointcloud = rospy.wait_for_message(
                self.topic_pointcloud_tiago, PointCloud2
                )

            product_c = self.product_pose()
            orientation = [product_c.orientation.w, product_c.orientation.x, product_c.orientation.y, product_c.orientation.z]
            position = [product_c.position.x, product_c.position.y, product_c.position.z]

            partial_pointcloud = point_cloud_filter(pointcloud, self.debug)

            if self.save:        
                saved_name = save_pointcloud(
                    partial_pointcloud,
                    self.product,
                    self.package_path + "/data/parameter_tuning/")
                
                csv_orientation = {saved_name: orientation} 
                csv_position = {saved_name: position}

                with open(self.package_path + "/data/parameter_tuning/poses/orientation.pkl", 'a+') as f:
                    pickle.dump(csv_orientation, f)
            
                with open(self.package_path + "/data/parameter_tuning/poses/position.pkl", 'a+') as g:
                    pickle.dump(csv_position, g)
        
            # lst = os.listdir(directory) # your directory path
            # number_files = len(lst)
            # print("currently " + str(number_files-1) + " pointclouds stored.")
            # print("\n")


            logask = raw_input(" ----- ROTATE OBJECT!? (y/n): ------")
            if logask == 'y': 
                self.running = True
            else: 
                self.running = False

        print("SCRIPT ENDED")


if __name__ == "__main__":
    rospy.init_node("main")

    start = Main()
    start.run()
