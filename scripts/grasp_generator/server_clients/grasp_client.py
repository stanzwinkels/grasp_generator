#!/usr/bin/env python

import rospy
import actionlib
from grasp_generator.msg import GraspAction
from grasp_generator.msg import GraspGoal


class GraspClient(object):
    def __init__(self, side="right"):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient('/grasp_server_' + side, GraspAction)
        rospy.loginfo('Waiting for server...')
        self.client.wait_for_server()

    def run(self, pose):
        goal = GraspGoal()
        goal.pose = pose
        rospy.loginfo('server starting: performing grasp')
        self.client.send_goal(goal)
        self.client.wait_for_result()
        raw_result = self.client.get_result()
        rospy.loginfo("server ended: did the grasp succeed? " + str(raw_result.success))
        result = raw_result.success
        return result