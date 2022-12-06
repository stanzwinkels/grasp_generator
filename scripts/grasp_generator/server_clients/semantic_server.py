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
from sklearn.preprocessing import StandardScaler

import pdb

def SemanticServer(superquadric):
    superquadric = np.reshape(superquadric.quadrics, (-1, 5))
    standard_scalar = pickle.load(open('standard_scalar.pkl', 'rb'))
    superquadric = standard_scalar.transform(superquadric)
    model = pickle.load(open('model_mlp.pkl', 'rb'))
    result = model.predict(superquadric)
    return SemanticResponse(result)

def semantic_sever():
    s = rospy.Service("semantic_server", Semantic, SemanticServer)

if __name__ == "__main__":
    rospy.init_node("semantic_server")
    print("~/semantic server running.... ")    
    semantic_sever()
    rospy.spin()
