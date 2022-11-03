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
import json
import tf2_ros
import numpy as np
from pyquaternion import Quaternion

# Messages
from sensor_msgs.msg import PointCloud2
from gazebo_msgs.srv import GetModelState
import geometry_msgs.msg

# Visualization
from grasp_generator.visualization.visualization_superquadric import (
    visualize_pointclouds
)


def save_data(location, data): 
    a = []
    if not os.path.isfile(location):
        a.append(data)
        with open(location, mode='w') as f:
            f.write(json.dumps(a, indent=2))
    else:
        with open(location) as feedsjson:
            feeds = json.load(feedsjson)

        feeds.append(data)
        with open(location, mode='w') as f:
            f.write(json.dumps(feeds, indent=2))


class Main:
    def __init__(
        self, debug=False, save=True,
    ):
        self.debug = debug
        self.save = save
        self.running = True
        self.load = True

        self.topic_pointcloud_rgb = "/camera/depth/color/points"
        self.topic_pointcloud_tiago = "/xtion/depth_registered/points"

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")

        self.product = "bottle1"

    def product_pose(self):
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(self.product, '/map')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        product_frame = geometry_msgs.msg.TransformStamped()
        product_frame.header.frame_id = "map"
        product_frame.header.stamp = rospy.Time.now()
        product_frame.child_frame_id = 'product_type' 
        product_frame.transform.translation = resp_coordinates.pose.position
        product_frame.transform.rotation = resp_coordinates.pose.orientation
        broadcaster.sendTransform(product_frame) 
        return resp_coordinates.pose

    def run(self):
        while self.running: 
            pointcloud = rospy.wait_for_message(
                self.topic_pointcloud_tiago, PointCloud2
                )

            partial_pointcloud = point_cloud_filter(pointcloud, self.debug)

            map_pose_to_product = self.product_pose()
            map_orientation_to_product = Quaternion(map_pose_to_product.orientation.w, map_pose_to_product.orientation.x, map_pose_to_product.orientation.y, map_pose_to_product.orientation.z)
            map_position_to_product = [map_pose_to_product.position.x, map_pose_to_product.position.y, map_pose_to_product.position.z]

            tf_buffer = tf2_ros.Buffer()
            tf2_listener = tf2_ros.TransformListener(tf_buffer)

            cam_object_transform = tf_buffer.lookup_transform("xtion_rgb_optical_frame", "product_type", rospy.Time(0), rospy.Duration(10))            
            camera_position_to_product = [cam_object_transform.transform.translation.x, cam_object_transform.transform.translation.y, cam_object_transform.transform.translation.z]
            camera_orientation_to_product = Quaternion(cam_object_transform.transform.rotation.w, cam_object_transform.transform.rotation.x, cam_object_transform.transform.rotation.y, cam_object_transform.transform.rotation.z)
            product_orientation_to_camera = camera_orientation_to_product.inverse
            
            map_pose_to_camera = tf_buffer.lookup_transform("map", "xtion_rgb_optical_frame", rospy.Time(0), rospy.Duration(10))            
            map_position_to_camera = [map_pose_to_camera.transform.translation.x, map_pose_to_camera.transform.translation.y, map_pose_to_camera.transform.translation.z]
            map_orientation_to_camera = Quaternion(map_pose_to_camera.transform.rotation.w, map_pose_to_camera.transform.rotation.x, map_pose_to_camera.transform.rotation.y, map_pose_to_camera.transform.rotation.z)
      

            # HERE TRANSFORM TO ORIGINAL POSE #
            new_pointcloud = []
            org_cam_partial_pointcloud = partial_pointcloud - camera_position_to_product
            for point in org_cam_partial_pointcloud:
                obj_frame_pointcloud = product_orientation_to_camera.rotate(point)
                map_frame = obj_frame_pointcloud.tolist()
                new_pointcloud.append(map_frame)
            new_pointcloud = np.array(new_pointcloud)
            #########

            if self.load:
                full_pointcloud = load_pointcloud(
                    self.package_path + "/data/full_pointcloud/")

            visualize_pointclouds(new_pointcloud, full_pointcloud[0])


            if self.save:        
                saved_name = save_pointcloud(
                    partial_pointcloud,
                    self.product,
                    self.package_path + "/data/parameter_tuning/bottle")
                
                map_orientation_to_product1 = list([map_orientation_to_product.w, map_orientation_to_product.x, map_orientation_to_product.y, map_orientation_to_product.z])
                map_orientation_to_camera1 = [map_orientation_to_camera.w, map_orientation_to_camera.x, map_orientation_to_camera.y, map_orientation_to_camera.z]
                product_orientation_to_camera1 = [product_orientation_to_camera.w, product_orientation_to_camera.x, product_orientation_to_camera.y, product_orientation_to_camera.z]

                csv_map_orientation_product = {saved_name: map_orientation_to_product1} 
                csv_map_position_product = {saved_name: map_position_to_product}
                csv_map_orientation_camera ={saved_name: map_orientation_to_camera1}
                csv_map_position_camera = {saved_name: map_position_to_camera}
                csv_product_orientation_camera = {saved_name: product_orientation_to_camera1}
                csv_camera_position_product  = {saved_name: camera_position_to_product}


                save_data(self.package_path + "/data/parameter_tuning/bottle/poses/map_orientation_product.json", csv_map_orientation_product)
                save_data(self.package_path + "/data/parameter_tuning/bottle/poses/map_position_product.json", csv_map_position_product)
                save_data(self.package_path + "/data/parameter_tuning/bottle/poses/map_orientation_camera.json", csv_map_orientation_camera)
                save_data(self.package_path + "/data/parameter_tuning/bottle/poses/map_position_camera.json", csv_map_position_camera)
                save_data(self.package_path + "/data/parameter_tuning/bottle/poses/product_orientation_camera.json", csv_product_orientation_camera)
                save_data(self.package_path + "/data/parameter_tuning/bottle/poses/camera_position_product.json", csv_camera_position_product)


            logask = raw_input(" ----- ROTATE OBJECT!? (y/n): ------")
            if logask == 'y': 
                self.running = True
            else: 
                self.running = False

        print("SCRIPT ENDED")


if __name__ == "__main__":
    rospy.init_node("data_generation")

    start = Main()
    start.run()