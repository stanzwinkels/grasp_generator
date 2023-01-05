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



# For arm control
import moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import tf

# utilities
from grocery_store_utils.srv import *
from grasp_generator.utils.standard_functions import rotate_vector 
from grasp_generator.utils.gripper_control import GripperControl


class GraspActionClass(object):

    _feedback = GraspFeedback()
    _result = GraspResult()

    def __init__(self, side="left"):
        # Initialize action server
        self._action_name = "grasp_server_" + side
        self._side = side
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.as_cb, auto_start = False)
        self._as.start()

        # Initialize grocery store collision object server clients
        rospy.wait_for_service('add_collision_object', timeout=2)
        rospy.wait_for_service('remove_collision_object', timeout=2)
        rospy.wait_for_service('get_grasp_pose', timeout=2)
        self._add_co = rospy.ServiceProxy('add_collision_object', addCollObjByAruco)
        self._remove_co = rospy.ServiceProxy('remove_collision_object', removeCollObj)
        self._get_grasp_pose = rospy.ServiceProxy('get_grasp_pose', getGraspPose)
        
        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("arm_{}".format(self._side))
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity

        # Transform pose
        self._tl = tf.TransformListener()

    def as_cb(self, pose):       
        # Prerequisites (open gripper and detached and remove objects)
        gripper = GripperControl(self._side)
        gripper.run('open')
        eef_link = self._group.get_end_effector_link()
        self._scene.remove_attached_object(link=eef_link)
        self._scene.remove_world_object()

        # 0.1. prerequisite: Add table as collision object in MoveIt
        coll_obj = CollisionObject()
        coll_obj.header.frame_id = self._robot.get_planning_frame()
        coll_obj.operation = coll_obj.ADD
        box = SolidPrimitive()
        box.type = box.BOX
        box.dimensions = [1.50, 1.50, 0.8]
        coll_obj.id = "table"
        coll_obj.primitives.append(box)

        cube_pose = Pose()
        cube_pose.position.x = 1.25
        cube_pose.position.y = 0.0
        cube_pose.position.z = 0.4
        cube_pose.orientation.w = 1.0
        coll_obj.primitive_poses.append(cube_pose)
        self._scene.add_object(coll_obj)
        rospy.sleep(1)
        print("TABLE ADDED!")

        pdb.set_trace()
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

        if self._side == "left": 
            neutral_goal = PoseStamped()
            neutral_goal.pose.position.x = 0.2
            neutral_goal.pose.position.y = 0.3
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


        # 2. Go to pre-grasp position
        goal = PoseStamped()
        goal.pose = pose.pose
        goal.header.frame_id = "xtion_rgb_optical_frame"
        self._group.set_pose_target(goal)
        succeeded = self._group.go(wait=True)

        if not succeeded:
            self._result.success=succeeded
            return self._as.set_succeeded(self._result)

        # 2.2 Go to grasp position linearly
        vector = rotate_vector([0.10,0,0], [goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w])
        goal.pose.position.x += vector[0]
        goal.pose.position.y += vector[1]
        goal.pose.position.z += vector[2]
        goal_map_frame = self._tl.transformPose("map", goal)
        (plan, _) = self._group.compute_cartesian_path([goal_map_frame.pose], 0.01, 0.0)
        succeeded = self._group.execute(plan, wait=True)

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
