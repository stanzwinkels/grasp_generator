#!/usr/bin/env python3.6


import numpy as np
from grasp_generator.tools_superquadric.single_superquadric_generation import EMS_recovery
from sklearn.cluster import DBSCAN

def fixed_nr_ems(

    point,
    OutlierRatio = 0.9,             # prior outlier probability [0, 1) (default: 0.1)
    MaxIterationEM = 20,            # maximum number of EM iterations (default: 20)
    ToleranceEM = 0.001,            # absolute tolerance of EM (default: 1e-3)
    RelativeToleranceEM = 0.1,      # relative tolerance of EM (default: 1e-1)
    MaxOptiIterations = 2,          # maximum number of optimization iterations per M (default: 2)
    Sigma = 0.3,                    # 0.3 initial sigma^2 (default: 0 - auto generate)
    MaxiSwitch = 1,                 # maximum number of switches allowed (default: 2)
    AdaptiveUpperBound = True,      # Introduce adaptive upper bound to restrict the volume of SQ (default: false)
    Rescale = True,                 # normalize the input point cloud (default: true)
    MaxLayer = 3,                   # maximum depth
    Eps = 0.03,                     # 0.03 IMPORTANT: varies based on the size of the input pointcoud (DBScan parameter)
    MinPoints = 300,                # DBScan parameter required minimum points
    nr_superquadrics = 2
):


    point_seg = {key: [] for key in list(range(0, MaxLayer+1))}
    point_outlier = {key: [] for key in list(range(0, MaxLayer+1))}
    point_seg[0] = [point]
    list_quadrics = []
    quadric_count = 1

    for h in range(MaxLayer):           # number of iterations.
        if quadric_count > nr_superquadrics:
            return list_quadrics

        for c in range(len(point_seg[h])):
            if quadric_count > nr_superquadrics:
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

    return list_quadrics
