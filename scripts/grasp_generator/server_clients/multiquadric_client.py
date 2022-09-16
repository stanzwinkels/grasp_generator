#!/usr/bin/env python3.6
import rospy

from grasp_generator.srv import (
    QuadricDetect,
    QuadricDetectRequest,
    QuadricDetectResponse,
)


class MultiquadricClient:
    def run(self, point_cloud):
        rate = rospy.Rate(1)

        try:
            multiquadric = rospy.ServiceProxy("/multiquadric_server", QuadricDetect)
            rospy.loginfo("waiting for quadric server")
            rospy.wait_for_service("/multiquadric_server", 15)
            response = multiquadric(point_cloud)
            rospy.loginfo("quadric shapes generated.")
            rate.sleep()
            return response

        except rospy.ServiceException as e:
            print("Multiquadric client: Service call failed")
        return


if __name__ == "__main__":
    rospy.init_node("multiquadric_client")
