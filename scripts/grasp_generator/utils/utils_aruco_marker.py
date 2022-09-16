# Ros
import rospy

# Messages
from aruco_msgs.msg import MarkerArray


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

