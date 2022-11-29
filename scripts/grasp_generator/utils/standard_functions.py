# ros
import rospy
import tf2_ros
import tf2_geometry_msgs

# utilities
import numpy as np
import open3d as o3d
import time
import json
import os
import tkinter as tk
from tkFileDialog import askopenfilename
import pickle
import pdb
from scipy.spatial import KDTree
from pyquaternion import Quaternion

# messages
from geometry_msgs.msg import Pose, PoseStamped, Point32
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_from_matrix


def region_cylinder(dimension, position, quaternion, region, superquadrics, Id): 
    """Create a simple cylinder from the superquadric dimensions in the correct orientation
            Addition: Based on the reasoner define which regions are suited to grasp!"""
    
    quaternion = Quaternion(quaternion[3], quaternion[0],quaternion[1],quaternion[2])

    t = np.arange(0, 2*np.pi, 0.1)              # default 0.1: how many steps in a circle
    x_dim = np.linspace(0, dimension[0], 10)    
    y_dim = np.linspace(0, dimension[1], 10)

    X_coord = np.array([])
    Y_coord = np.array([])
    for i in range(len(x_dim)): 
        X_coord = np.append(X_coord, x_dim[i]*np.cos(t))
        Y_coord = np.append(Y_coord, y_dim[i]*np.sin(t))
    z = np.full(len(X_coord), dimension[2])
    disk_top = np.array([X_coord, Y_coord, z])
    disk_bottom = np.array([X_coord, Y_coord, -z])


    z_dim = np.arange(-dimension[2]*0.98, dimension[2]*0.98, 0.001)
    x_outside = np.array([])
    y_outside = np.array([])
    z_outside = np.array([])
    for i in range(len(z_dim)): 
        x_outside = np.append(x_outside, x_dim[-1]*np.cos(t))
        y_outside = np.append(y_outside, y_dim[-1]*np.sin(t))
        z_outside = np.append(z_outside, np.full(len(t), z_dim[i]))
    disk_surface = np.array([x_outside, y_outside, z_outside])

    new_disk_top = np.empty((len(disk_top[0]),3))
    new_disk_bottom = np.empty((len(disk_bottom[0]),3))
    new_disk_surface = np.empty((len(disk_surface[0]),3))

    for idx, _ in enumerate(disk_top[0]): 
        new_disk_top[idx] = (quaternion.rotate(np.array([disk_top[0,idx],disk_top[1,idx],disk_top[2,idx]])))+position

    for idx, _ in enumerate(disk_bottom[0]): 
        new_disk_bottom[idx] = (quaternion.rotate(np.array([disk_bottom[0,idx],disk_bottom[1,idx],disk_bottom[2,idx]])))+position

    for idx, _ in enumerate(disk_surface[0]): 
        new_disk_surface[idx] = (quaternion.rotate(np.array([disk_surface[0,idx],disk_surface[1,idx],disk_surface[2,idx]]))) + position



    if region == "All":
        label = np.array([True,True,True])       # top, bottom, surface

    elif region == "Round":
        label = np.array([False,False,True])       # surface

    elif region == "Flat":
        bottom_grasp = True
        top_grasp = True
        if len(superquadrics) > 1:
            # check all the other shapes with this shape....
            reference_dim = superquadrics[Id-1, 2:5]
            reference_quat = Quaternion(superquadrics[Id-1, 5:9][3], superquadrics[Id-1, 5:9][0], superquadrics[Id-1, 5:9][1], superquadrics[Id-1, 5:9][2])
            reference_quat = reference_quat.inverse
            reference_pos = superquadrics[Id-1, 9:12]

            for j in [x for x in range(len(superquadrics)) if x != (Id-1)]:
                compare_pos = superquadrics[j, 9:12]                
                point = reference_pos - compare_pos
                new_center = reference_quat.rotate(point)
                value = (new_center[0]**2/reference_dim[0]**2) + (new_center[1]**2/reference_dim[1]**2) - 1

                if value < 1 and reference_dim[2]*0.5 < new_center[2]: 
                    bottom_grasp = False
                    print("TOP GRASP NOT POSSIBLE")
                elif value < 1 and new_center[2] < -reference_dim[2]*0.5:       # assumption that I make, a product has to be from the center half a dimension distance. otherwise it is a side object!
                    top_grasp = False
                    print("BOTTOM GRASP NOT POSSIBLE")

        if bottom_grasp and top_grasp: 
            label = np.array([True,True,False])       # top, bottom, 
            print("bottom yes, top yes")

        elif not bottom_grasp and top_grasp: 
            label = np.array([True,False,False])
            print("bottom not, top yes")    # top
            
        elif bottom_grasp and not top_grasp: 
            label = np.array([False,True,False])       # bottom
            print("bottom yes, top not")

        elif not bottom_grasp and not top_grasp: 
            label = np.array([False,False,False])
            print("bottom not, top not")
        
    return np.array([new_disk_top, new_disk_bottom, new_disk_surface]), label



def rotate_points(points, quaternion): 
    for idx, point in enumerate(points): 
        points[idx] = quaternion.rotate(point)
    return points

def rotate_points_X_Y_Z(X, Y, Z, quaternion): 
    points = np.empty((len(X),3))
    for idx, x in enumerate(X): 
        points[idx] = quaternion.rotate(np.array([x,Y[idx],Z[idx]]))
    return points

def cube_segmentation(dim, origin, quaternion): 
    x = np.arange(-dim[0], dim[0], 0.01)
    y = np.arange(-dim[1], dim[1], 0.01)
    z = np.arange(-dim[2], dim[2], 0.01)

    xy_x_mesh_grid, xy_y_mesh_grid = np.meshgrid(x, y)
    xz_x_mesh_grid, xz_z_mesh_grid = np.meshgrid(x, z)
    yz_y_mesh_grid, yz_z_mesh_grid = np.meshgrid(y, z)

    mesh_xy = rotate_points_X_Y_Z(xy_x_mesh_grid.flatten(), xy_y_mesh_grid.flatten(), dim[2]*np.ones(len(xy_y_mesh_grid.flatten())), quaternion)+origin
    mesh_xz = rotate_points_X_Y_Z(xz_x_mesh_grid.flatten(), dim[1]*np.ones(len(xz_x_mesh_grid.flatten())), xz_z_mesh_grid.flatten(), quaternion)+origin
    mesh_yz = rotate_points_X_Y_Z(dim[0]*np.ones(len(yz_y_mesh_grid.flatten())), yz_y_mesh_grid.flatten(), yz_z_mesh_grid.flatten(), quaternion)+origin

    mesh_xy_neg = rotate_points_X_Y_Z(xy_x_mesh_grid.flatten(), xy_y_mesh_grid.flatten(), -dim[2]*np.ones(len(xy_y_mesh_grid.flatten())), quaternion)+origin
    mesh_xz_neg = rotate_points_X_Y_Z(xz_x_mesh_grid.flatten(), -dim[1]*np.ones(len(xz_x_mesh_grid.flatten())), xz_z_mesh_grid.flatten(), quaternion)+origin
    mesh_yz_neg = rotate_points_X_Y_Z(-dim[0]*np.ones(len(yz_y_mesh_grid.flatten())), yz_y_mesh_grid.flatten(), yz_z_mesh_grid.flatten(), quaternion)+origin
    return mesh_xy, mesh_xz, mesh_yz, mesh_xy_neg, mesh_xz_neg, mesh_yz_neg



def cylinder_reasoning(EPS, DIM):
    for idx, eps in enumerate(EPS):
        if eps[0] > eps[1]: 
            # height = dim[1]
            # diameters = dim[0], dim[2]]
            DIM[idx] = np.array([DIM[idx, 0], DIM[idx,2], DIM[idx,1]])
        else: 
            # height = dim[2]
            # diameters = [dim[0], dim[1]]
            DIM[idx] = np.array([DIM[idx,0], DIM[idx,1], DIM[idx,2]])
    return DIM


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



def load_single_pointcloud(location):
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


def transform_partial_pointcloud_origin(package_path, partial_pointcloud, name):
    """
    Transform a partial pointcloud back around the location (0, 0, 0). The translation and rotation are canceled using the stored transformations in Json. 
    """
    object_name, ext = os.path.splitext(name)

    directory_camera_position_product = package_path + "/data/test_data/camera_position_product.json"
    directory_product_orientation_camera = package_path + "/data/test_data/product_orientation_camera.json"
    
    camera_position_product = json.load(open(directory_camera_position_product))
    product_orientation_camera = json.load(open(directory_product_orientation_camera))
    camera_position_product = create_dict(camera_position_product)[object_name]
    product_orientation_camera = create_dict(product_orientation_camera)[object_name]
    product_orientation_camera = Quaternion(product_orientation_camera[0], product_orientation_camera[1], product_orientation_camera[2], product_orientation_camera[3])

    org_pointcloud = []
    org_cam_partial_pointcloud = partial_pointcloud - camera_position_product
    for point in org_cam_partial_pointcloud:
        obj_frame_pointcloud = product_orientation_camera.rotate(point)
        map_frame = obj_frame_pointcloud.tolist()
        org_pointcloud.append(map_frame)
    origin_partial_pointcloud = np.array(org_pointcloud)
    return origin_partial_pointcloud

def accuracy_overlap_partial(pointcloud_gt, ground_truth, partial_pointcloud, partial_pointcloud_label):
    tree = KDTree(pointcloud_gt)
    dist, ids = tree.query(partial_pointcloud, k=1)
    tp, fp, tn, fn = [], [], [], []
    accuracy = []
    
    for idx_label, ID in enumerate(partial_pointcloud_label):
        if ID == 1: 
            if ID == ground_truth[ids[idx_label]]:
                tp.append(True)         # the predicted value is black, and it should be black
            else:
                fp.append(True)         # the predicted value is black, but it should have been red
        elif ID == 0: 
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