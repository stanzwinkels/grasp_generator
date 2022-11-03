#!/usr/bin/env python  
import rospy

from grasp_generator.srv import PoseAruco, PoseArucoRequest

class ArucoClient():
    def run(self, aruco):
        rate = rospy.Rate(1)
        try:
            pose_aruco = rospy.ServiceProxy("/pose_aruco", PoseAruco)
            rospy.loginfo("server starting: creating aruco pose.")
            rospy.wait_for_service("/pose_aruco", 15)
            resp = pose_aruco(aruco)
            rospy.loginfo("server ended: created an aruco pose.")
            rate.sleep()
            return    

        except rospy.ServiceException as e:
            print("Aruco_server: Service call failed")       
        return 