#!/usr/bin/env python

'''
Author: Stan Zwinkels
Context: This file is made specifically for visualizing in RVIZ and in 3d plot. 

'''

# Ros imports
import rospy
import numpy as np
import pdb

# Visualization messages
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker as VisMarker
from visualization_msgs.msg import MarkerArray as VisArrayMarker


# Draw 3D plots
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from tf.transformations import quaternion_from_matrix

import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R
from itertools import cycle

import numpy as np
palette = cycle(['black', 'red', 'green', 'orange', 'yellow', 'purple', 'grey'])     


# Visualize TIAGo robot hand in RVIZ
def pub_grasp_pose(grasp, frame_id):
    afford_pub = rospy.Publisher("robot_hand", VisMarker, queue_size=100, latch= True)
    grasp_marker = VisMarker()
    grasp_marker.header.frame_id = frame_id
    grasp_marker.type = VisMarker.MESH_RESOURCE
    grasp_marker.mesh_resource = "package://retail_store_simulation/models/gripper.dae"
    grasp_marker.action = VisMarker.ADD
    grasp_marker.scale.x = 1.0
    grasp_marker.scale.y = 1.0
    grasp_marker.scale.z = 1.0
    grasp_marker.color.a = 1.0
    grasp_marker.color.r = 1.0
    grasp_marker.color.g = 0.0
    grasp_marker.color.b = 0.0
    afford_pub.publish(grasp_marker)
    return



def visualize_grasp(grasp_pose, frame_id, name):
    grasp = rospy.Publisher(name, VisArrayMarker, queue_size=100, latch= True)
    array_marker = VisArrayMarker()

    pose = Pose()
    pose.position.x = grasp_pose.position.x
    pose.position.y = grasp_pose.position.y
    pose.position.z = grasp_pose.position.z

    marker = VisMarker()
    marker.header.frame_id = frame_id # 
    marker.header.stamp = rospy.Time()
    marker.ns = "points"
    marker.id = 0
    marker.type = VisMarker.SPHERE
    marker.action = VisMarker.ADD
    marker.pose.orientation.w = 1.0
    # marker.points = [grasp_pose.position]   
    marker.pose.position = grasp_pose.position
    marker.scale.x = 0.1 # forward direction
    marker.scale.y = 0.1 # hand closing direction
    marker.scale.z = 0.1 # hand vertical direction
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration()
    array_marker.markers.append(marker)
    grasp.publish(array_marker)   
    return



def plot_affordance(poly, name, frame, color=[1,0,0,1]):
    grasp = rospy.Publisher(name, VisArrayMarker, queue_size=100, latch= True)
    array_marker = VisArrayMarker()

    for idx, poses in enumerate(poly):
        pose = Pose()
        pose.position.x = poses[0]
        pose.position.y = poses[1]
        pose.position.z = poses[2]

        marker = VisMarker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time()
        marker.ns = "point_affordance"
        marker.id = idx
        marker.type = VisMarker.SPHERE_LIST
        marker.action = VisMarker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [pose.position]   
        marker.scale.x = 0.01 # forward direction
        marker.scale.y = 0.01# hand closing direction
        marker.scale.z = 0.01 # hand vertical direction
        marker.color.a = color[0]
        marker.color.r = color[1]
        marker.color.g = color[2]
        marker.color.b = color[3]
        marker.lifetime = rospy.Duration()
        array_marker.markers.append(marker)
    grasp.publish(array_marker)   
    return



def plot_points(grasp_poses):
    grasp = rospy.Publisher("points_something", VisArrayMarker, queue_size=100, latch= True)
    array_marker = VisArrayMarker()

    for idx, poses in enumerate(grasp_poses):
        if poses[0] == 0:
            pose = Pose()
            pose.position = poses[1].position
            marker = VisMarker()
            marker.header.frame_id = "xtion_rgb_optical_frame"
            marker.header.stamp = rospy.Time()
            marker.ns = "points_new"
            marker.id = idx
            marker.type = VisMarker.SPHERE_LIST
            marker.action = VisMarker.ADD
            marker.pose.orientation.w = 1.0
            marker.points = [pose.position]   
            marker.scale.x = 0.01 # forward direction
            marker.scale.y = 0.01# hand closing direction
            marker.scale.z = 0.01 # hand vertical direction
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.lifetime = rospy.Duration()
            array_marker.markers.append(marker)
        elif poses[0] == 1:
            pose = Pose()
            pose.position = poses[1].position
            marker = VisMarker()
            marker.header.frame_id = "xtion_rgb_optical_frame"
            marker.header.stamp = rospy.Time()
            marker.ns = "points_new"
            marker.id = idx
            marker.type = VisMarker.SPHERE_LIST
            marker.action = VisMarker.ADD
            marker.pose.orientation.w = 1.0
            marker.points = [pose.position]   
            marker.scale.x = 0.01 # forward direction
            marker.scale.y = 0.01# hand closing direction
            marker.scale.z = 0.01 # hand vertical direction
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration()
            array_marker.markers.append(marker)
    grasp.publish(array_marker)   
    return



# Visualize grasping pose in RVIZ
def publish_hand(grasp):
    hand_pub = rospy.Publisher("visualize_grasp", VisArrayMarker, queue_size=100, latch= True)

    grasp_position = np.array([grasp.position.x, grasp.position.y, grasp.position.z])
    grasp_approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
    grasp_binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
    grasp_axis = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])
    
    rotation_matrix = np.eye(4)
    rotation_matrix[0:3,0] = grasp_approach
    rotation_matrix[0:3,1] = grasp_binormal
    rotation_matrix[0:3,2] = grasp_axis

    array_marker = VisArrayMarker()

    # Change this yaml parameters automatically!
    hand_depth = 0.06
    hand_height = 0.02
    outer_diameter = 0.12
    finger_width = 0.01
    
    hw = 0.5 * outer_diameter - 0.5 * finger_width
    left_bottom = grasp_position - hw * grasp_binormal
    right_bottom = grasp_position + hw * grasp_binormal
    left_top = left_bottom + hand_depth * grasp_approach
    right_top = right_bottom + hand_depth * grasp_approach
    left_center = left_bottom + 0.5*(left_top - left_bottom)
    right_center = right_bottom + 0.5*(right_top - right_bottom)
    base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*grasp_approach
    approach_center = base_center - 0.04*grasp_approach

    finger_lwh = [hand_depth, finger_width, hand_height]
    approach_lwh = [0.08, finger_width, hand_height]

    left_finger = create_finger_marker(left_center, finger_lwh, 10, rotation_matrix) 
    right_finger = create_finger_marker(right_center, finger_lwh, 20, rotation_matrix)
    approach = create_finger_marker(approach_center, approach_lwh, 30, rotation_matrix)
    base = create_hand_base_marker(left_bottom, right_bottom, 0.02, hand_height, 1, rotation_matrix)

    array_marker.markers.append(left_finger)
    array_marker.markers.append(right_finger)
    array_marker.markers.append(approach)
    array_marker.markers.append(base)

    hand_pub.publish(array_marker)    
    return 


def create_finger_marker(center, lwh, id, rot_matrix):
    marker = VisMarker()
    marker.header.frame_id = "xtion_rgb_optical_frame"
    marker.header.stamp = rospy.Time()
    marker.ns = "finger"
    marker.id = id
    marker.type = VisMarker.CUBE
    marker.action = VisMarker.ADD
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]

    quat = quaternion_from_matrix(rot_matrix)
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.scale.x = lwh[0] # forward direction
    marker.scale.y = lwh[1] # hand closing direction
    marker.scale.z = lwh[2] # hand vertical direction
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()
    return marker

def create_hand_base_marker(start, end, length, height, id, rot_matrix):
    center = start + 0.5 * (end - start)

    marker = VisMarker()
    marker.header.frame_id = "xtion_rgb_optical_frame"
    marker.header.stamp = rospy.Time()
    marker.ns = "hand_base"
    marker.id = id
    marker.type = VisMarker.CUBE
    marker.action = VisMarker.ADD
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]

    quat = quaternion_from_matrix(rot_matrix)
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.scale.x = length # forward direction
    marker.scale.y = np.linalg.norm(end - start) # hand closing direction
    marker.scale.z = height # hand vertical direction
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()
    
    return marker


def visualize_cylindrical(width, depth, height):
    grasp = rospy.Publisher("cylinder", VisMarker, queue_size=100, latch= True)
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    marker = VisMarker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time()
    marker.ns = "cylinder"
    marker.id = 5
    marker.type = VisMarker.CYLINDER
    marker.action = VisMarker.ADD
    marker.pose.orientation.w = 1.0
    marker.pose.position = pose.position
    marker.scale.x = width # forward direction
    marker.scale.y = depth # hand closing direction
    marker.scale.z = height # hand vertical direction
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()
    grasp.publish(marker)   
    return

def grasp_points(grasp_poses, good_points, bad_points):
    grasp = rospy.Publisher("visualize_points", VisArrayMarker, queue_size=100, latch= True)
    array_marker = VisArrayMarker()

    marker = VisMarker()
    marker.header.frame_id = "xtion_rgb_optical_frame"
    marker.header.stamp = rospy.Time()
    marker.ns = "points"
    marker.id = 11101111
    marker.type = VisMarker.SPHERE_LIST
    marker.action = VisMarker.ADD
    marker.pose.orientation.w = 1.0
    marker.points = [grasp_poses[good_points[0]][1].position]

    # use orientation of hand frame
    marker.scale.x = 0.01 # forward direction
    marker.scale.y = 0.01# hand closing direction
    marker.scale.z = 0.01 # hand vertical direction
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration()
    array_marker.markers.append(marker)

    # good_points.pop(0)

    for idx in good_points:
        marker = VisMarker()
        marker.header.frame_id = "xtion_rgb_optical_frame"
        marker.header.stamp = rospy.Time()
        marker.ns = "points"
        marker.id = idx
        marker.type = VisMarker.SPHERE_LIST
        marker.action = VisMarker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [grasp_poses[idx][1].position]

        # use orientation of hand frame
        marker.scale.x = 0.01 # forward direction
        marker.scale.y = 0.01# hand closing direction
        marker.scale.z = 0.01 # hand vertical direction
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()
        array_marker.markers.append(marker)

    for idx in bad_points:
        marker = VisMarker()
        marker.header.frame_id = "xtion_rgb_optical_frame"
        marker.header.stamp = rospy.Time()
        marker.ns = "points"
        marker.id = idx
        marker.type = VisMarker.SPHERE_LIST
        marker.action = VisMarker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [grasp_poses[idx][1].position]

        # use orientation of hand frame
        marker.scale.x = 0.01 # forward direction
        marker.scale.y = 0.01# hand closing direction
        marker.scale.z = 0.01 # hand vertical direction
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()
        array_marker.markers.append(marker)

    grasp.publish(array_marker)   
    return

def affordance_box(coordinates, position, id):
    afford_pub = rospy.Publisher("visualize_affordance", VisMarker, queue_size=100, latch= True)

    marker = VisMarker()
    marker.header.frame_id = "xtion_rgb_optical_frame"
    marker.header.stamp = rospy.Time()
    marker.ns = "object_box"
    marker.id = id
    marker.type = VisMarker.CUBE
    marker.action = VisMarker.ADD
    marker.pose.position.x = position.x 
    marker.pose.position.y = position.y 
    marker.pose.position.z = position.z 
    marker.pose.orientation.x = coordinates[0]
    marker.pose.orientation.y = coordinates[1]
    marker.pose.orientation.z = coordinates[2]
    marker.pose.orientation.w = coordinates[3]

    marker.scale.x = 0.05 # forward direction
    marker.scale.y = 0.05# hand closing direction
    marker.scale.z = 0.1 # hand vertical direction
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()

    afford_pub.publish(marker)        
    return marker


    ###################### PLOTTING STUFFFFF IN 3D ################################
def plot_axis_object(quaternion, object_location, plot_object=False, grasps=None):
    
    axis_x = [1,0,0]
    axis_y = [0,1,0]
    axis_z = [0,0,1]
 
    ax_x = (rotate_vector(axis_x, quaternion))
    ax_y = (rotate_vector(axis_y, quaternion))
    ax_z = (rotate_vector(axis_z, quaternion))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if grasps is not None:
        x_pos, y_pos, z_pos, x_app, y_app, z_app = [],[],[],[],[],[]
        for index, integer in enumerate(grasps):
            x_pos.append(integer.position.x)
            y_pos.append(integer.position.y)
            z_pos.append(integer.position.z)
            x_app.append(integer.approach.x)
            y_app.append(integer.approach.y)
            z_app.append(integer.approach.z)

        ax.scatter(x_pos, y_pos, z_pos)

        print('yes')

    if plot_object:
        p1 = [-0.025, -0.025, -0.1]
        p2 = [-0.025, -0.025, 0]
        p3 = [0.025, -0.025, 0]
        p4 = [0.025, -0.025, -0.1]
        p5 = [-0.025, 0.025, 0]
        p6 = [0.025, 0.025, 0]
        p7 = [-0.025, 0.025, -.1]
        p8 = [0.025, 0.025, -0.1]
        p1_new = (rotate_vector(p1, quaternion))
        p2_new = (rotate_vector(p2, quaternion))
        p3_new = (rotate_vector(p3, quaternion))
        p4_new = (rotate_vector(p4, quaternion))
        p5_new = (rotate_vector(p5, quaternion))
        p6_new = (rotate_vector(p6, quaternion))        
        p7_new = (rotate_vector(p7, quaternion))
        p8_new = (rotate_vector(p8, quaternion))


    ax.set_xlim([object_location.x-1, object_location.x+1])
    ax.set_ylim([object_location.y-1, object_location.y+1])
    ax.set_zlim([object_location.z-1, object_location.z+1])

    # ax.quiver(object_location.x, object_location.y, object_location.z, axis_x[0], axis_x[1], axis_x[2], color="r", linestyle='dashed')
    # ax.quiver(object_location.x, object_location.y, object_location.z, axis_y[0], axis_y[1], axis_y[2], color="g", linestyle='dashed')
    # ax.quiver(object_location.x, object_location.y, object_location.z, axis_z[0], axis_z[1], axis_z[2], color="b", linestyle='dashed')

    ax.quiver(0,0,0, axis_x[0], axis_x[1], axis_x[2], color="r", linestyle='dashed')
    ax.quiver(0,0,0, axis_y[0], axis_y[1], axis_y[2], color="g", linestyle='dashed')
    ax.quiver(0,0,0, axis_z[0], axis_z[1], axis_z[2], color="b", linestyle='dashed')
    ax.quiver(object_location.x, object_location.y, object_location.z, ax_x[0], ax_x[1], ax_x[2], color="r")
    ax.quiver(object_location.x, object_location.y, object_location.z, ax_y[0], ax_y[1], ax_y[2], color="g")
    ax.quiver(object_location.x, object_location.y, object_location.z, ax_z[0], ax_z[1], ax_z[2], color="b")


    if plot_object:
        ax.scatter(p1[0] + object_location.x, p1[1] + object_location.y, p1[2] + object_location.z, color = 'y')
        ax.scatter(p2[0] + object_location.x, p2[1] + object_location.y, p2[2] + object_location.z, color = 'y')
        ax.scatter(p3[0] + object_location.x, p3[1] + object_location.y, p3[2] + object_location.z, color = 'y')
        ax.scatter(p4[0] + object_location.x, p4[1] + object_location.y, p4[2] + object_location.z, color = 'y')
        ax.scatter(p5[0] + object_location.x, p5[1] + object_location.y, p5[2] + object_location.z, color = 'y')
        ax.scatter(p6[0] + object_location.x, p6[1] + object_location.y, p6[2] + object_location.z, color = 'y')
        ax.scatter(p7[0] + object_location.x, p7[1] + object_location.y, p7[2] + object_location.z, color = 'y')
        ax.scatter(p8[0] + object_location.x, p8[1] + object_location.y, p8[2] + object_location.z, color = 'y')


        ax.scatter(p1_new[0] + object_location.x, p1_new[1] + object_location.y, p1_new[2] + object_location.z, color = 'c')
        ax.scatter(p2_new[0] + object_location.x, p2_new[1] + object_location.y, p2_new[2] + object_location.z, color = 'c')
        ax.scatter(p3_new[0] + object_location.x, p3_new[1] + object_location.y, p3_new[2] + object_location.z, color = 'c')
        ax.scatter(p4_new[0] + object_location.x, p4_new[1] + object_location.y, p4_new[2] + object_location.z, color = 'c')
        ax.scatter(p5_new[0] + object_location.x, p5_new[1] + object_location.y, p5_new[2] + object_location.z, color = 'black')
        ax.scatter(p6_new[0] + object_location.x, p6_new[1] + object_location.y, p6_new[2] + object_location.z, color = 'black')
        ax.scatter(p7_new[0] + object_location.x, p7_new[1] + object_location.y, p7_new[2] + object_location.z, color = 'black')
        ax.scatter(p8_new[0] + object_location.x, p8_new[1] + object_location.y, p8_new[2] + object_location.z, color = 'black')

        ax.plot([p1_new[0] + object_location.x, p2_new[0] + object_location.x],[p1_new[1] + object_location.y, p2_new[1] + object_location.y],[p1_new[2] + object_location.z, p2_new[2] + object_location.z], color='black')
        ax.plot([p2_new[0] + object_location.x, p3_new[0] + object_location.x],[p2_new[1] + object_location.y, p3_new[1] + object_location.y],[p2_new[2] + object_location.z, p3_new[2] + object_location.z], color='black')
        ax.plot([p3_new[0] + object_location.x, p4_new[0] + object_location.x],[p3_new[1] + object_location.y, p4_new[1] + object_location.y],[p3_new[2] + object_location.z, p4_new[2] + object_location.z], color='black')
        ax.plot([p4_new[0] + object_location.x, p1_new[0] + object_location.x],[p4_new[1] + object_location.y, p1_new[1] + object_location.y],[p4_new[2] + object_location.z, p1_new[2] + object_location.z], color='black')
        ax.plot([p5_new[0] + object_location.x, p2_new[0] + object_location.x],[p5_new[1] + object_location.y, p2_new[1] + object_location.y],[p5_new[2] + object_location.z, p2_new[2] + object_location.z], color='green')
        ax.plot([p6_new[0] + object_location.x, p3_new[0] + object_location.x],[p6_new[1] + object_location.y, p3_new[1] + object_location.y],[p6_new[2] + object_location.z, p3_new[2] + object_location.z], color='green')
        ax.plot([p7_new[0] + object_location.x, p1_new[0] + object_location.x],[p7_new[1] + object_location.y, p1_new[1] + object_location.y],[p7_new[2] + object_location.z, p1_new[2] + object_location.z], color='blue')
        ax.plot([p8_new[0] + object_location.x, p4_new[0] + object_location.x],[p8_new[1] + object_location.y, p4_new[1] + object_location.y],[p8_new[2] + object_location.z, p4_new[2] + object_location.z], color='blue')

    ax.set_xlabel("X", color='red', size=18)
    ax.set_ylabel("Y", color='green', size=18)
    ax.set_zlabel("z", color='b', size=18)
    plt.show() 
    return