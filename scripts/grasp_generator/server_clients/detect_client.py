#!/usr/bin/env python

import rospy
from grasp_generator.srv import DetectGrasps, DetectGraspsRequest


def detect_grasp_client(point_cloud):
    rate = rospy.Rate(1)
    try:
        detect_grasp = rospy.ServiceProxy("/detect_grasps/detect_object", DetectGrasps)
        rospy.wait_for_service("/detect_grasps/detect_object", 5)
        rospy.loginfo("server starting: generating grasp poses")
        grasping_poses = detect_grasp(point_cloud)
        rospy.loginfo("server ending: "+ str(len(grasping_poses.grasp_configs.grasps)) + " grasping poses found")
        rate.sleep()
        return grasping_poses.grasp_configs
    
    except rospy.ServiceException as e:
        print("Service call failed")
    return 