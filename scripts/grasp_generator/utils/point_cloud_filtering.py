#!/usr/bin/env python

"""
This file filters the raw point cloud into a scene point cloud and an object point cloud. 
"""

# Ros
import rospy
import ros_numpy

# Libraries
import numpy as np
import open3d as o3d
import trimesh

def point_cloud_filter(point_cloud_raw):
    """
    Function to capture a pointcloud of a single object. (scene specific)

    Return
    ---------
    pointcloud : array
        object pointcloud, x, y, z
    """

    point_cloud_xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud_raw)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(np.array(point_cloud_xyz))
    # o3d.visualization.draw_geometries([point_cloud])

    # 1. Crop raw point cloud
    lb = point_cloud.get_min_bound()
    ub = point_cloud.get_max_bound()

    ub[-1] = 0.8
    ub[0] =  0.1
    ub[1] =  2.0
    lb[0] =  -1.2

    min_bound = lb
    max_bound = ub

    box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    point_cloud_crop = point_cloud.crop(box)
    # o3d.visualization.draw_geometries([point_cloud_crop])

    # 2. remove table perceived pointcloud
    plane_model, inliers = point_cloud_crop.segment_plane(
        # distance_threshold=rospy.get_param('plane_segmentation/distance_threshold'), 
        distance_threshold=0.01, 

        ransac_n=rospy.get_param('plane_segmentation/ransac_n'), 
        num_iterations=rospy.get_param('plane_segmentation/num_iterations')
    )
    partial_point_cloud = point_cloud_crop.select_down_sample(inliers, invert=True)
    # o3d.visualization.draw_geometries([partial_point_cloud])


    ###### Optional to create a mesh first #####
    # mesh = False
    # if mesh: 
    #     # 3. reduce the number of points
    #     object_cloud = partial_point_cloud.voxel_down_sample(
    #         voxel_size=rospy.get_param('mesh/voxel_size'))
    #     if debug:
    #         o3d.visualization.draw_geometries([object_cloud])

    #     # 4. create mesh of the object pointcloud
    #     alpha = rospy.get_param('mesh/alpha')
    #     tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(object_cloud)
    #     object_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
    #         object_cloud, 0.25, tetra_mesh, pt_map
    #     )
    #     if debug:
    #         o3d.visualization.draw_geometries([object_mesh])

    #     # 5. Sample points from the mesh to create a new pointcloud
    #     object_mesh.compute_vertex_normals()
    #     mesh_point_cloud = object_mesh.sample_points_uniformly(
    #         number_of_points=rospy.get_param('mesh/mesh_sampling_points'))
    #     if debug:
    #         o3d.visualization.draw_geometries([mesh_point_cloud])
    ################################################################


    partial_point_cloud = np.asarray(partial_point_cloud.points)
    point_cloud = np.asarray(point_cloud.points)
    point_cloud_crop = np.asarray(point_cloud_crop.points)

    return point_cloud_crop, partial_point_cloud
