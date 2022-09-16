#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
from grasp_generator.srv import PoseAruco, PoseArucoRequest, PoseArucoResponse

def pose_aruco(aruco):    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    aruco_frame = geometry_msgs.msg.TransformStamped()
    aruco_frame.header.frame_id = "map"
    aruco_frame.header.stamp = rospy.Time.now()
    aruco_frame.child_frame_id = str(aruco.marker.id) 
    aruco_frame.transform.translation = aruco.marker.pose.pose.position
    aruco_frame.transform.rotation = aruco.marker.pose.pose.orientation
    broadcaster.sendTransform(aruco_frame) 
    return PoseArucoResponse(aruco.marker.id)

def aruco_pose_server():
    s = rospy.Service("pose_aruco", PoseAruco, pose_aruco)

if __name__ == "__main__":
    rospy.init_node("aruco_server")
    aruco_pose_server()
    rospy.spin()
