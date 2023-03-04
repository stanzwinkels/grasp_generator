#!/usr/bin/env python3.6
import rospy
from grasp_generator.srv import (
    QuadricDetect,
    QuadricDetectRequest,
    QuadricDetectResponse,
)
import numpy as np

from grasp_generator.tools_superquadric.multi_superquadric_generation import hierarchical_ems

def multiquadric(points):
    pointcloud = []
    for point in points.pointcloud:
        pointcloud.append([point.x, point.y, point.z])
    pointcloud = np.array(pointcloud)
    list_quadrics = hierarchical_ems(pointcloud)
    value_quadrics = []

    for i in range(len(list_quadrics)):
        value_quadrics = np.concatenate(
            (
                value_quadrics,
                list_quadrics[i]._shape,
                list_quadrics[i]._scale,
                list_quadrics[i].quat,
                list_quadrics[i]._translation,
            )
        )
    return QuadricDetectResponse(value_quadrics)

def multi_quadric_sever():
    s = rospy.Service("multiquadric_server", QuadricDetect, multiquadric)

if __name__ == "__main__":
    rospy.init_node("multiquadric_server")
    print("Server running.... ")
    multi_quadric_sever()
    rospy.spin()
