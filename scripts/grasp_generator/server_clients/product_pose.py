#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState

import tf2_ros
import geometry_msgs.msg

def product_pose(product):
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resp_coordinates = model_coordinates(product, 'map')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    product_frame = geometry_msgs.msg.TransformStamped()
    product_frame.header.frame_id = "map"
    product_frame.header.stamp = rospy.Time.now()
    product_frame.child_frame_id = 'product_type' 
    product_frame.transform.translation = resp_coordinates.pose.position
    product_frame.transform.rotation = resp_coordinates.pose.orientation
    broadcaster.sendTransform(product_frame) 

    return resp_coordinates.pose
    
if __name__ == '__main__':
    rospy.init_node("product_detection")
    print("Server running.... ")
    pose = product_pose("bottle2")
    print(pose)
    print("finished")
    rospy.spin()

