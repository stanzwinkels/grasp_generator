#!/usr/bin/env python3.6

# Libaries
import yaml 
import numpy as np
import rospkg
import open3d as o3d
import tkinter as tk
from tkinter.filedialog import askopenfilename
from math import sqrt, tanh
import pdb
from scipy.optimize import linear_sum_assignment
import os
import pickle
import joblib
import plotly
import plotly.express as px

# Superquadric generator
from grasp_generator.tools_superquadric.multi_superquadric_fixed_number import fixed_nr_ems


# Visualization
from grasp_generator.visualization.visualization_superquadric import visualize_superquadric

# Rotation
from pyquaternion import Quaternion


import optuna
from optuna.visualization import plot_optimization_history, plot_parallel_coordinate

  
def size_diff(size, superquadrics):
    sum_size_matrix = np.zeros([len(size), len(size)])
    n = len(size)
    select_matrix = np.array([[[0 for k in range(3)] for j in range(n)] for i in range(n)])

    for i in range(len(size)):
        for j in range(len(superquadrics)):
            cost = np.array([[np.abs(size[i][0] - superquadrics[j, 2]), np.abs(size[i][0] - superquadrics[j, 3]), np.abs(size[i][0] - superquadrics[j, 4])],
                [np.abs(size[i][1] - superquadrics[j, 2]), np.abs(size[i][1] - superquadrics[j, 3]), np.abs(size[i][1] - superquadrics[j, 4])],
                [np.abs(size[i][2] - superquadrics[j, 2]), np.abs(size[i][2] - superquadrics[j, 3]), np.abs(size[i][2] - superquadrics[j, 4])]])
            row_ind, col_ind = linear_sum_assignment(cost, maximize=True)
            select_matrix[i][j] = np.array(col_ind)
            sum_size_matrix[i][j] = cost[row_ind, col_ind].sum()
    row_ind, col_ind = linear_sum_assignment(sum_size_matrix, maximize=True)
    total_size_difference = sum_size_matrix[row_ind, col_ind].sum()
    total_size_diff_norm = tanh(np.abs(total_size_difference))
    return total_size_diff_norm, select_matrix[row_ind, col_ind], col_ind


def dist_difference(distance, superquadrics): 
    distQ1 = np.array([superquadrics[:,9][0] - superquadrics[:,9][1], superquadrics[:,10][0] - superquadrics[:,10][1], superquadrics[:,11][0] - superquadrics[:,11][1]])
    distQ2 = np.array([superquadrics[:,9][1] - superquadrics[:,9][0], superquadrics[:,10][1] - superquadrics[:,10][0], superquadrics[:,11][1] - superquadrics[:,11][0]])
    min_difference = np.array([np.abs((distance - distQ1)).sum(), np.abs((distance - distQ2)).sum()]).min()
    norm_min_difference = tanh(np.abs(min_difference))
    return norm_min_difference

def objective(trial, pointclouds, dimensions, distances):
    OutlierRatio = trial.suggest_float('OutlierRatio', 0.0, 1.0)
    MaxIterationEM = trial.suggest_int('MaxIterationEM', 10, 30, step=2)                # maximum number of EM iterations (default: 20)
    ToleranceEM = trial.suggest_float('ToleranceEM', 0.0001, 0.01)                      # absolute tolerance of EM (default: 1e-3)
    RelativeToleranceEM = trial.suggest_float('RelativeToleranceEM', 0.01, 1)           # relative tolerance of EM (default: 1e-1)
    MaxOptiIterations = 2                                                               # maximum number of optimization iterations per M (default: 2)
    Sigma = 0.3                                                                         # 0.3 initial sigma^2 (default: 0 - auto generate)
    MaxiSwitch = trial.suggest_int("MaxiSwitch", 1, 4, step=1)                          # maximum number of switches allowed (default: 2)
    AdaptiveUpperBound = trial.suggest_categorical("AdaptiveUpperBound", [True, False]) # Introduce adaptive upper bound to restrict the volume of SQ (default: false)
    Rescale = True                                                                      # normalize the input point cloud (default: true)
    MaxLayer = 3                                                                        # maximum depth
    Eps = trial.suggest_float('Eps',0.001, 0.1)                                         # 0.03 IMPORTANT: varies based on the size of the input pointcoud (DBScan parameter)
    MinPoints = trial.suggest_int('MinPoints', 50, 500, step=50),                       # DBScan parameter required minimum points

    sum_total_score = 0
    i = 0 
    count_scores = 0
    for i in range(len(pointclouds)):          # for pointcloud in iteration 30
        try: 
            list_quadrics = fixed_nr_ems(
                pointclouds[i], 
                OutlierRatio = OutlierRatio,             
                MaxIterationEM = MaxIterationEM,            
                ToleranceEM = ToleranceEM,            
                RelativeToleranceEM = RelativeToleranceEM,      
                MaxOptiIterations = MaxOptiIterations,          
                Sigma = Sigma,                    
                MaxiSwitch = MaxiSwitch,                 
                AdaptiveUpperBound = AdaptiveUpperBound,      
                Rescale = Rescale,                 
                MaxLayer = MaxLayer,                   
                Eps = Eps,                    
                MinPoints = MinPoints,               
                nr_superquadrics = 2)

            value_quadrics =[]
            for k in range(len(list_quadrics)):
                value_quadrics = np.concatenate(
                    (
                        value_quadrics,
                        list_quadrics[k]._shape,
                        list_quadrics[k]._scale,
                        list_quadrics[k].quat,
                        list_quadrics[k]._translation,
                    )
                )
            superquadrics = np.reshape(value_quadrics, (-1, 12))
            if len(superquadrics) == 2: 
                total_size_diff_norm, _, col_ind = size_diff(dimensions[i], superquadrics)
                total_dist_diff_norm = dist_difference(distances[i], superquadrics)
                total_score = (total_size_diff_norm + total_dist_diff_norm)/2.0
                sum_total_score += total_score
                print("iteration pointcloud: " + str(i) + "/" + str(len(pointclouds)))
                count_scores += 1
            else: 
                raise Exception('Number of generated quadrics not 2.')
        except: 
            print("failure: " + str(i) + "/" + str(len(pointclouds)))
            sum_total_score += 1

    normalized_score = sum_total_score/len(pointclouds)
    return normalized_score

def load_pointcloud(location):
    root = tk.Tk()
    root.withdraw()
    root.filename = askopenfilename(initialdir=location, title="Select file",
                                            filetypes=[("Pointcloud", "*.ply")])
    source = o3d.io.read_point_cloud(root.filename)
    partial_pointcloud = np.asarray(source.points)
    return partial_pointcloud

def load_yaml(product_type):
    product = products["products"]
    product_info = product[product_type]
    dimensions = np.array([[]])
    distance = np.array([[]])
    for keys in product_info.keys():
        if keys == "dist":
            x = product_info[keys]["x"] 
            y = product_info[keys]["y"]
            z = product_info[keys]["z"]
            distance = np.append(distance, [x,y,z])
        else:
            x = product_info[keys]["x"]
            y = product_info[keys]["y"]
            z = product_info[keys]["z"]
            dimensions = np.append(dimensions, [x,y,z])
    dimensions = np.reshape(dimensions, (-1, 3))
    return dimensions, distance

if __name__ == "__main__":
    data_orientations = {}
    pointclouds = []
    dimensions = []
    distances = []

    rospack = rospkg.RosPack()
    package_path = rospack.get_path("grasp_generator")
    directory_yaml = package_path + "/config/experiment_products.yaml"
    directory_orientation = package_path + "/data/parameter_tuning/poses/orientation.pkl"
    directory_product = package_path + "/data/parameter_tuning/mug/"

    with open(directory_yaml) as stream:
        products = yaml.safe_load(stream)

    with open(directory_orientation, "rb") as f: 
        while True:
            try:
                data_orientations.update(pickle.load(f))
            except EOFError:
                break

    for filename in os.listdir(directory_product):
        name, ext = os.path.splitext(filename)
        if ext == '.ply':    
            product_type = filename.split('-')[0]
            dimension, distance = load_yaml(product_type)
            quaternion = Quaternion(data_orientations[name][0], data_orientations[name][1], data_orientations[name][2], data_orientations[name][3])
            distance_rot = quaternion.rotate(distance)
            source = o3d.io.read_point_cloud(directory_product + filename)
            partial_pointcloud = np.asarray(source.points)
            
            pointclouds.append(partial_pointcloud)
            dimensions.append(dimension)
            distances.append(distance_rot)

    study = optuna.create_study(direction="minimize")
    study.optimize(lambda trial: objective(trial, pointclouds, dimensions, distances),
            n_trials=50)

    study_id = study._study_id
    best_params = study.best_params



    # save results: 
    save_figures = True
    if save_figures: 
        directory_fig = package_path + "/data/figures/"
        if not os.path.exists(directory_fig):
            os.makedirs(directory_fig)

        fig_opt = plot_optimization_history(study)
        fig_par = plot_parallel_coordinate(study)

        plotly.offline.plot(fig_opt, filename= directory_fig +"_optimization_cup.html", auto_open = False)
        plotly.offline.plot(fig_par, filename= directory_fig +"_parallel_coordinates_cup.html", auto_open = False)           


        joblib.dump(study, directory_fig+"study_cup.pkl")
