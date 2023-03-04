#!/usr/bin/env python

# ros
import rospy
import actionlib
import pdb
# import messages
from grasp_generator.msg import GraspAction, GraspResult, GraspFeedback
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped 

from copy import deepcopy


# For arm control
import moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import tf

# utilities
from grocery_store_utils.srv import *
from grasp_generator.utils.standard_functions import rotate_vector 
from grasp_generator.utils.gripper_control import GripperControl

from grasp_generator.visualization.visualization_grasps import (
    visualize_grasp
)


class GraspActionClass(object):

    _feedback = GraspFeedback()
    _result = GraspResult()

    def __init__(self, side="left"):
        # Initialize action server
        self._action_name = "grasp_server_" + side
        self._side = side
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.as_cb, auto_start = False)
        self._as.start()

        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("arm_right_torso")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity

        # Transform pose
        self._tl = tf.TransformListener()
        print("yes")


    def as_cb(self, pose):       
        # Prerequisites (open gripper and detached and remove objects)

        gripper = GripperControl(self._side)
        gripper.run('open')
        eef_link = self._group.get_end_effector_link()
        self._scene.remove_attached_object(link=eef_link)
        self._scene.remove_world_object()

        # 0.1. prerequisite: Add table as collision object in MoveIt
        coll_obj = CollisionObject()
        coll_obj.header.frame_id = "base_link"
        coll_obj.operation = coll_obj.ADD
        box = SolidPrimitive()
        box.type = box.BOX
        box.dimensions = [1.50, 1.50, 0.53]     # 0.57 #0.74
        coll_obj.id = "table"
        coll_obj.primitives.append(box)

        cube_pose = Pose()
        cube_pose.position.x = 1.3
        cube_pose.position.y = 0.0
        cube_pose.position.z = 0.37
        cube_pose.orientation.w = 1.0
        coll_obj.primitive_poses.append(cube_pose)
        self._scene.add_object(coll_obj)
        rospy.sleep(1)
        print("TABLE ADDED!")

        # 2. Go to pre-grasp position
        goal = PoseStamped()
        goal.pose = pose.pose
        goal.header.frame_id = "xtion_rgb_optical_frame"
        visualize_grasp(goal.pose, "xtion_rgb_optical_frame", "final_grasp_camera")       
        goal = self._tl.transformPose("base_footprint", goal)     

        vector = rotate_vector([0.10,0,0], [goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w])
        new_grasp = deepcopy(goal)
        new_grasp.pose.position.x += vector[0] 
        new_grasp.pose.position.y += vector[1]
        new_grasp.pose.position.z += vector[2]

        # 1. Go to prior position
        if self._side == "right": 
            neutral_goal = PoseStamped()
            neutral_goal.pose.position.x = 0.2
            neutral_goal.pose.position.y = -0.3
            neutral_goal.pose.position.z = 1.2
            neutral_goal.pose.orientation.x = 0
            neutral_goal.pose.orientation.y = 0
            neutral_goal.pose.orientation.z = 0
            neutral_goal.pose.orientation.w = 0
            neutral_goal.header.frame_id = "base_footprint"
            self._group.set_pose_target(neutral_goal)
            succeeded = self._group.go(wait=True)
            if not succeeded:
                self._result.success=succeeded
                return self._as.set_succeeded(self._result)

        visualize_grasp(goal.pose, "base_footprint", "pre_grasp_pose")
        visualize_grasp(new_grasp.pose, "base_footprint", "grasp_pose")  
       
        self._group.set_pose_target(goal)
        succeeded = False
        while succeeded == False: 
            succeeded = self._group.go(wait=True)
            print(succeeded)

        self._group.set_pose_target(new_grasp)
        succeeded = False
        while succeeded == False: 
            succeeded = self._group.go(wait=True)
            print(succeeded)

        # 3. close gripper and attach collision object
        gripper.run('close')
        rospy.loginfo("closed gripper")
        grasping_group = "gripper_{}".format(self._side)
        touch_links = self._robot.get_link_names(group=grasping_group)
        eef_link = self._group.get_end_effector_link()


        # 4. Go to posterior coordinate
        self._group.set_pose_target(neutral_goal)
        succeeded = self._group.go(wait=True)
        if not succeeded:
            self._result.success=succeeded
            return self._as.set_succeeded(self._result)

        # 5. return arm to tuck
        client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        client.wait_for_server()
        rospy.loginfo("...connected.")
        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(1.0)
        rospy.loginfo("Tuck arm...")
        goal = PlayMotionGoal()
        goal.motion_name = 'home_'+ self._side
        goal.skip_planning = False
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(20.0))
        rospy.loginfo("Arm tucked.")

        self._result.success=succeeded
        return self._as.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("grasp_server")
    rospy.loginfo("grasp servers started")

    GraspActionClass(side="right")
    # GraspActionClass(side="left")
    rospy.spin()
