#!/usr/bin/env python3.6
import rospy
import numpy as np
from grasp_generator.srv import (
    Semantic,
    SemanticRequest,
    SemanticResponse,
)

import pdb

class SemanticClient:
    def run(self, superquadric):
        rate = rospy.Rate(1)
        superquadric = superquadric.flatten()
        try:
            semantic = rospy.ServiceProxy("/semantic_server", Semantic)
            rospy.loginfo("waiting for semantic server")
            rospy.wait_for_service("/semantic_server", 15)
            response = semantic(superquadric)
            response = list(response.shape)
            shapes = ['cuboid', 'cylinder', 'sphere']
            for id, resp in enumerate(response): 
                response[id] = shapes[int(resp)]            
            rospy.loginfo("superquadric shapes classified.")
            rate.sleep()
            return response

        except rospy.ServiceException as e:
            print("Semantic classification client: Service call failed")
        return

if __name__ == "__main__":
    rospy.init_node("semantic_client")

    superquadric = np.array([[1.0179203 , 0.97124332  ,0.02187232 , 0.02107734 , 0.02758151],     # sphere
                                [1.30289576 , 1.05399751 , 0.02726866  ,0.03386318 , 0.12026305],   # cylinder
                                [1.20289576 , 1.05399751 , 0.02726866  ,0.03386318 , 0.02026305],   # sphere
                                [1.20289576 , 0.15399751 , 0.02726866  ,0.03386318 , 0.02026305],   # cuboid
                                [0.3289576 , 0.15399751 , 0.02726866  ,0.03386318 , 0.12026305]])  # cuboid

    superquadric = superquadric.flatten()
    semantic_client = SemanticClient()
    result = semantic_client.run(superquadric)
    print("DONE", result)