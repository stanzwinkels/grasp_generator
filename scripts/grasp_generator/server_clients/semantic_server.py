#!/usr/bin/env python3.6
import rospy
from grasp_generator.srv import (
    Semantic,
    SemanticRequest,
    SemanticResponse,
)
import numpy as np
import pickle
from sklearn.svm import SVC
import pdb

def SemanticServer(superquadric):
    model = pickle.load(open('model_svm.pkl', 'rb'))
    result = model.predict([list(superquadric.quadrics)])
    return SemanticResponse(int(result))

def semantic_sever():
    s = rospy.Service("semantic_server", Semantic, SemanticServer)

if __name__ == "__main__":
    rospy.init_node("semantic_server")
    print("~/Server running.... ")    
    semantic_sever()
    rospy.spin()
