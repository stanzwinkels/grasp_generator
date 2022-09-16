# utilities
from functools import partial
import numpy as np
import open3d as o3d
import time
import os
import tkinter as tk
from tkFileDialog import askopenfilename

# messages
from geometry_msgs.msg import Pose, PoseStamped, Point32
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_from_matrix

def load_pointcloud(location):
    root = tk.Tk()
    root.withdraw()
    root.filename = askopenfilename(initialdir=location, title="Select file",
                                            filetypes=[("Pointcloud", "*.ply")])
    source = o3d.io.read_point_cloud(root.filename)
    partial_pointcloud = np.asarray(source.points)
    return partial_pointcloud


def save_pointcloud(pointcloud, product, location): 
    logask = raw_input("Save current pointcoud? (y/n): ")
    if logask == 'y': 
        t = time.localtime()
        timestamp = time.strftime('%b-%d-%Y_%H%M%S', t)
        product_name = product + "-" + timestamp + '.ply'
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud)
        o3d.io.write_point_cloud(location+product_name, pcd,  write_ascii=True, compressed=False, print_progress=True)
    else: 
        print("not saving\n")



# converts pointcloud to a correct message
def point_cloud_to_message(point_cloud):
    point_cloud_msg = []
    for i in range(len(point_cloud)):
        point = Point32()
        point.x, point.y, point.z = point_cloud[i][0], point_cloud[i][1], point_cloud[i][2] 
        point_cloud_msg.append(point)
    return point_cloud_msg


def rotate_vector(vector, quaternion):
    vector.append(0.0)
    return quaternion_multiply(quaternion_multiply(quaternion, vector),quaternion_conjugate(quaternion))[:3]



def tiago_pose(final_pose, offset=-0.15, pre_grasp=-0.15):
    vector = [offset+ pre_grasp, 0, 0]

    quat = quaternion_rotation_matrix(final_pose)
    quat = quaternion_multiply(quat, [0.7071, 0, 0, 0.7071])
    vector = rotate_vector(vector, [quat[0], quat[1], quat[2], quat[3]])

    robot_pose = Pose()
    robot_pose.position.x = final_pose.position.x + vector[0]
    robot_pose.position.y = final_pose.position.y + vector[1]
    robot_pose.position.z = final_pose.position.z + vector[2]

    robot_pose.orientation.x = quat[0] 
    robot_pose.orientation.y = quat[1] 
    robot_pose.orientation.z = quat[2] 
    robot_pose.orientation.w = quat[3]
    return robot_pose


def quaternion_rotation_matrix(pose):
    grasp_approach = np.array([pose.approach.x, pose.approach.y, pose.approach.z])
    grasp_binormal = np.array([pose.binormal.x, pose.binormal.y, pose.binormal.z])
    grasp_axis = np.array([pose.axis.x, pose.axis.y, pose.axis.z])

    rotation_matrix = np.eye(4)
    rotation_matrix[0:3,0] = grasp_approach
    rotation_matrix[0:3,1] = grasp_binormal
    rotation_matrix[0:3,2] = grasp_axis
    quat = quaternion_from_matrix(rotation_matrix)
    return quat