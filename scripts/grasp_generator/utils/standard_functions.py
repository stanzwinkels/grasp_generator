# ros
import rospy
import tf2_ros
import tf2_geometry_msgs

# utilities
import numpy as np
import open3d as o3d
import time
import os
import tkinter as tk
from tkFileDialog import askopenfilename
import pickle
import pdb
from scipy.spatial import KDTree

# messages
from geometry_msgs.msg import Pose, PoseStamped, Point32
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_from_matrix


def create_dict(list_values): 
    result = {}
    for d in list_values:
        result.update(d)
    return result

def load_data(location):
    data = {}
    with open(location, "rb") as f: 
        while True:
            try:
                data.update(pickle.load(f))
            except EOFError:
                break
    return data


def load_pointcloud(location):
    root = tk.Tk()
    root.withdraw()
    root.filename = askopenfilename(initialdir=location, title="Select file",
                                            filetypes=[("Pointcloud", "*.ply")])
    source = o3d.io.read_point_cloud(root.filename)
    partial_pointcloud = np.asarray(source.points)
    colors_pointcloud = np.asarray(source.colors)
    return partial_pointcloud, os.path.basename(root.filename), colors_pointcloud



def save_pointcloud(pointcloud, product, location): 
    logask = raw_input("Save current pointcoud? (y/n): ")
    if logask == 'y': 
        t = time.localtime()
        timestamp = time.strftime('%b-%d-%Y_%H%M%S', t)
        product_name = product + "-" + timestamp 
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud)
        o3d.io.write_point_cloud(location+product_name + '.ply', pcd,  write_ascii=True, compressed=False, print_progress=True)
        return product_name
    else: 
        print("not saving\n")
    return 


def filter_full_pointcloud(pointcloud_gt, colors_gt): 
    black = np.array([0, 0 ,0])
    red = np.array([1,0,0])
    white = np.array([1,1,1])

    
    ground_truth = np.empty(len(pointcloud_gt))
    for idx, color in enumerate(colors_gt):
        if (color == black).all():
            ground_truth[idx] = 0
        elif (color == red).all():
            ground_truth[idx] = 1
        else: 
            ground_truth[idx] = -1
    pointcloud_gt = pointcloud_gt[ground_truth >= 0]
    ground_truth = ground_truth[ground_truth >= 0]
    return pointcloud_gt, ground_truth

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

def accuracy_overlap_partial(pointcloud_gt, ground_truth, partial_pointcloud, partial_pointcloud_label):
    tree = KDTree(pointcloud_gt)
    dist, ids = tree.query(partial_pointcloud, k=1)
    tp, fp, tn, fn = [], [], [], []
    accuracy = []

    for idx_label, ID in enumerate(partial_pointcloud_label):
        if ID == 0: 
            if ID == ground_truth[ids[idx_label]]:
                tp.append(True)         # the predicted value is black, and it should be black
            else:
                fp.append(True)         # the predicted value is black, but it should have been red
        elif ID == 1: 
            if ID == ground_truth[ids[idx_label]]:
                tn.append(True)         # the predicted value is red, and it should be red
            else: 
                fn.append(True)         # the predicted value is red, but it should be black

    accuracy = (float(sum(tp)+sum(tn))/len(partial_pointcloud_label))*100
    true_positive_rate = (float(sum(tp))/(sum(tp)+sum(fp)))*100
    print("Accuracy = " + str(accuracy) + "%")
    print("True positive rate = " + str(true_positive_rate) + "%")
    return accuracy, true_positive_rate




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

def transform_pose(pose, from_frame, to_frame):
    """
    Function to transform coordinates one frame to another frame 

    Parameters
    ----------
    position    : message, 
        Position of object to be translated in the correct frame.
    orientation : message
        Orientation of object to be rotate in the correct frame.
    from_frame  : str, source_frame
    to_frame    : str, resulting_Frame

    Return
    ---------
    output_pose_stamped.pose : message
        Object coordinates in the new reference frame.
    """
    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    start_pose = PoseStamped()
    start_pose.pose = pose
    start_pose.header.frame_id = from_frame
    start_pose.header.stamp = rospy.Time(0)

    output_pose_stamped = tf_buffer.transform(start_pose, to_frame, rospy.Duration(2))    
    return output_pose_stamped.pose


