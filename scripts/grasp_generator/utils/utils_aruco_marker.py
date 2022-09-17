# Ros
import rospy
import numpy as np

# Messages
from aruco_msgs.msg import MarkerArray
from grasp_generator.utils.standard_functions import rotate_vector

def detect_aruco_marker(objects, topic="/aruco_marker_publisher/markers"):
    """
    Function to wait for aruco_marker detection to obtain the coordinates + ID. 

    Return
    ---------
    aruco_poses : dict
        Detected aruco_cube marker in dictionary

    Raises
    ---------
        Cannot detect multiple aruco markers. 
    """

    detected_id = []
    aruco_poses = {}
    
    print("--- Searching for aruco marker ---")
    while len(detected_id) != objects:
        aruco_marker = rospy.wait_for_message(topic, MarkerArray)
        for marker in aruco_marker.markers:
            if marker.id not in detected_id:
                detected_id.append(marker.id)
                aruco_poses[marker.id] = marker
    print("Aruco marker", aruco_poses.keys(), "detected")
    return aruco_poses


def convert_aruco_center(aruco_poses):
    """
    Function transforms aruco center of the object based on the object size.
    """ 
    data = rospy.get_param('products')
    for i in aruco_poses:
        product_info =  [v for k, v in data.items() if v['id'] == i]
        offset = [product_info[0]['dist_to_center']['x'], product_info[0]['dist_to_center']['y'], product_info[0]['dist_to_center']['z']]

        aruco_loc = [aruco_poses[i].pose.pose.position.x, aruco_poses[i].pose.pose.position.y, aruco_poses[i].pose.pose.position.z]
        ort_a = aruco_poses[i].pose.pose.orientation
        quaternion = [ort_a.x, ort_a.y, ort_a.z, ort_a.w]
        new_center = (rotate_vector(offset, quaternion) + aruco_loc)
        aruco_poses[i].pose.pose.position.x = new_center[0]
        aruco_poses[i].pose.pose.position.y = new_center[1]
        aruco_poses[i].pose.pose.position.z = new_center[2]
    return aruco_poses

def match_mult_aruco(aruco_poses, grasp_poses, threshold_dist):
    centers, points, grasp_list = [], [], []
    for i in aruco_poses:
        centers.append([aruco_poses[i].pose.pose.position.x, aruco_poses[i].pose.pose.position.y, aruco_poses[i].pose.pose.position.z])
    for i in range(len(grasp_poses.grasps)):
        points.append([grasp_poses.grasps[i].position.x, grasp_poses.grasps[i].position.y, grasp_poses.grasps[i].position.z])
    
    for idx, point in enumerate(points):
        dist = []
        point = np.array(point)
        for center in centers:
            center = np.array(center)
            dist.append(np.sqrt(np.sum(np.square(point-center))))
        if min(dist) < threshold_dist:
            grasp_list.append([dist.index(min(dist)), grasp_poses.grasps[idx]])
    return grasp_list