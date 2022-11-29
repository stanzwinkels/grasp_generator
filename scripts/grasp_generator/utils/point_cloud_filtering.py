#!/usr/bin/env python

# Ros
import rospy
import ros_numpy

# Libraries
import numpy as np
import open3d as o3d
import trimesh


def point_cloud_filter(point_cloud_raw, debug):
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

    # if debug:
    #     o3d.visualization.draw_geometries([point_cloud])

    # 1. crop area perceived pointcloud
    lb = point_cloud.get_min_bound()
    ub = point_cloud.get_max_bound()

    ub[-1] = rospy.get_param('perception_area/max_depth_threshold')
    ub[0] = rospy.get_param('perception_area/min_depth_threshold')
    ub[1] =  rospy.get_param('perception_area/max_width_threshold')
    lb[0] = rospy.get_param('perception_area/min_width_threshold')

    min_bound = lb
    max_bound = ub

    box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    point_cloud = point_cloud.crop(box)
    # if debug:
    #     o3d.visualization.draw_geometries([point_cloud])

    # 2. remove table perceived pointcloud
    plane_model, inliers = point_cloud.segment_plane(
        distance_threshold=rospy.get_param('plane_segmentation/distance_threshold'), 
        ransac_n=rospy.get_param('plane_segmentation/ransac_n'), 
        num_iterations=rospy.get_param('plane_segmentation/num_iterations')
    )

    point_cloud = point_cloud.select_down_sample(inliers, invert=True)
    if debug:
        o3d.visualization.draw_geometries([point_cloud])


    mesh = False
    if mesh: 
        # 3. reduce the number of points
        object_cloud = point_cloud.voxel_down_sample(
            voxel_size=rospy.get_param('mesh/voxel_size'))
        if debug:
            o3d.visualization.draw_geometries([object_cloud])

        # 4. create mesh of the object pointcloud
        alpha = rospy.get_param('mesh/alpha')
        tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(object_cloud)
        object_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            object_cloud, 0.25, tetra_mesh, pt_map
        )
        if debug:
            o3d.visualization.draw_geometries([object_mesh])

        # 5. Sample points from the mesh to create a new pointcloud
        object_mesh.compute_vertex_normals()
        point_cloud = object_mesh.sample_points_uniformly(
            number_of_points=rospy.get_param('mesh/mesh_sampling_points'))
        if debug:
            o3d.visualization.draw_geometries([point_cloud])

    point_cloud = np.asarray(point_cloud.points)

    return point_cloud
