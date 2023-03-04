#!/usr/bin/env python3.6

import rospy
import numpy as np

from grasp_generator.tools_superquadric.single_superquadric_generation import EMS_recovery
from sklearn.cluster import DBSCAN

import pdb

import pandas as pd

def hierarchical_ems(
    point,
    OutlierRatio=rospy.get_param('hierachical_ems/OutlierRatio'),               # prior outlier probability [0, 1) (default: 0.1)
    MaxIterationEM=rospy.get_param('hierachical_ems/MaxIterationEM'),           # maximum number of EM iterations (default: 20)
    ToleranceEM=rospy.get_param('hierachical_ems/ToleranceEM'),                 # absolute tolerance of EM (default: 1e-3)
    RelativeToleranceEM=rospy.get_param('hierachical_ems/RelativeToleranceEM'), # relative tolerance of EM (default: 1e-1)
    MaxOptiIterations=rospy.get_param('hierachical_ems/MaxOptiIterations'),     # maximum number of optimization iterations per M (default: 2)
    Sigma=rospy.get_param('hierachical_ems/Sigma'),                             # 0.3 initial sigma^2 (default: 0 - auto generate)
    MaxiSwitch=rospy.get_param('hierachical_ems/MaxiSwitch'),                   # maximum number of switches allowed (default: 2)
    AdaptiveUpperBound=rospy.get_param('hierachical_ems/AdaptiveUpperBound'),   # Introduce adaptive upper bound to restrict the volume of SQ (default: false)
    Rescale=rospy.get_param('hierachical_ems/Rescale'),                         # normalize the input point cloud (default: true)
    MaxLayer=rospy.get_param('hierachical_ems/MaxLayer'),                       # maximum depth
    Eps=rospy.get_param('hierachical_ems/Eps'),                                 # 0.03 IMPORTANT: varies based on the size of the input pointcoud (DBScan parameter)
    MinPoints=rospy.get_param('hierachical_ems/MinPoints'),                     # DBScan parameter required minimum points
    Max_superquadrics = rospy.get_param('hierachical_ems/Max_superquadrics')

):

    point_seg = {key: [] for key in list(range(0, MaxLayer+1))}
    point_outlier = {key: [] for key in list(range(0, MaxLayer+1))}
    point_seg[0] = [point]
    list_quadrics = []
    quadric_count = 1


    points_input = []
    points_outlier = []

    for h in range(MaxLayer):           # number of iterations.
        if quadric_count > Max_superquadrics:
            return list_quadrics
        for c in range(len(point_seg[h])):
            if quadric_count > Max_superquadrics:
                return list_quadrics

            quadric_count += 1
            x_raw, p_raw = EMS_recovery(
                point_seg[h][c],
                OutlierRatio,
                MaxIterationEM,
                ToleranceEM,
                RelativeToleranceEM,
                MaxOptiIterations,
                Sigma,
                MaxiSwitch,
                AdaptiveUpperBound,
                Rescale,
            )

            points_input.append(point_seg[h][c][p_raw > 0.1, :])
            points_outlier.append(point_seg[h][c][p_raw < 0.1, :])

            point_previous = point_seg[h][c]
            list_quadrics.append(x_raw)
            outlier = point_seg[h][c][p_raw < 0.1, :]
            point_seg[h][c] = point_seg[h][c][p_raw > 0.1, :]
            if np.sum(p_raw) < (0.8 * len(point_previous)):
                clustering = DBSCAN(eps=Eps, min_samples=MinPoints).fit(outlier)
                labels = list(set(clustering.labels_))
                labels = [item for item in labels if item >= 0]
                if len(labels) >= 1:
                    for i in range(len(labels)):
                        point_seg[h + 1].append(outlier[clustering.labels_ == i])
                point_outlier[h].append(outlier[clustering.labels_ == -1])
            else:
                point_outlier[h].append(outlier)

    # points_input = np.array(points_input)
    # points_outlier = np.array(points_outlier)
    # input_pd1 = pd.DataFrame(points_input[0])
    # input_pd2 = pd.DataFrame(points_input[1])
    # input_pd3 = pd.DataFrame(points_input[2])

    # output_pd1 = pd.DataFrame(points_outlier[0])
    # output_pd2 = pd.DataFrame(points_outlier[1])
    # output_pd3 = pd.DataFrame(points_outlier[2])

    # input_pd1.to_csv('input_points1.csv',index=False,header=False)  # save as csv file
    # input_pd2.to_csv('input_points2.csv',index=False,header=False)  # save as csv file
    # input_pd3.to_csv('input_points3.csv',index=False,header=False)  # save as csv file

    # output_pd1.to_csv('output_points1.csv',index=False,header=False)  # save as csv file
    # output_pd2.to_csv('output_points2.csv',index=False,header=False)  # save as csv file
    # output_pd3.to_csv('output_points3.csv',index=False,header=False)  # save as csv file
    return list_quadrics
