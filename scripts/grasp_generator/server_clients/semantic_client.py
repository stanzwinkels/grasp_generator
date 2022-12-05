#!/usr/bin/env python3.6
import rospy
import numpy as np
from grasp_generator.srv import (
    Semantic,
    SemanticRequest,
    SemanticResponse,
)

class SemanticClient:
    def run(self, superquadric):
        rate = rospy.Rate(1)
        try:
            semantic = rospy.ServiceProxy("/semantic_server", Semantic)
            rospy.loginfo("waiting for semantic server")
            rospy.wait_for_service("/semantic_server", 15)
            response = semantic(superquadric)
            shapes = ['cuboid', 'cylinder', 'sphere']
            rospy.loginfo("superquadric shapes classified.")
            rate.sleep()
            return shapes[response.shape]

        except rospy.ServiceException as e:
            print("Semantic classification client: Service call failed")
        return

if __name__ == "__main__":
    rospy.init_node("semantic_client")
    superquadric = np.array([0.1, 1.0, 1, 1, 1])


    semantic_client = SemanticClient()
    result = semantic_client.run(superquadric)
    print("DONE", result)