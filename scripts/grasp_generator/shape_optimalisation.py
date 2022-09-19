#!/usr/bin/env python3.6
from re import X
from grasp_generator.tools_superquadric.multi_superquadric_fixed_number import fixed_nr_ems

# Libaries
import numpy as np
import rospkg
import open3d as o3d
import tkinter as tk
from tkinter.filedialog import askopenfilename
from itertools import combinations
from math import sqrt
import pdb

# Visualization
from grasp_generator.visualization.visualization_superquadric import visualize_superquadric
from sklearn.model_selection import RandomizedSearchCV

import csv
import time

import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment


def load_pointcloud(location):
    root = tk.Tk()
    root.withdraw()
    root.filename = askopenfilename(initialdir=location, title="Select file",
                                            filetypes=[("Pointcloud", "*.ply")])
    source = o3d.io.read_point_cloud(root.filename)
    partial_pointcloud = np.asarray(source.points)
    return partial_pointcloud


def dist(x_vals, y_vals, z_vals):
  " Distance of pair combinations of x_vals & y_vals & z_vals (i.e. 1-2, 1-3, 2-3, etc.)"
  dist2 = lambda z: sqrt((z[0][0] - z[1][0]) ** 2.0 + (z[0][1]- z[1][1]) ** 2.0 + (z[0][2]- z[1][2]) ** 2.0)
  return list(map(dist2, combinations(zip(x_vals, y_vals, z_vals), 2)))

    
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
            # for z in range(len(row_ind)):
            #     print("row: ", row_ind[z], "  col: " ,col_ind[z], "  value: ", cost[z, col_ind[z]] )
            # print("\n")

    row_ind, col_ind = linear_sum_assignment(sum_size_matrix, maximize=True)
    total_size_difference = sum_size_matrix[row_ind, col_ind].sum()

    return total_size_difference, select_matrix[row_ind, col_ind], col_ind


def dist_difference(distance, superquadrics): 
    distQ = dist(superquadrics[:,9], superquadrics[:,10], superquadrics[:,11])
    cost = np.zeros([len(distance), len(distance)])
    for i in range(len(distance)): 
        for j in range(len(distQ)):
            cost[i][j] = np.abs(distance[i] - distQ[j]) 
    row_ind, col_ind = linear_sum_assignment(cost, maximize=True)
    total_dist_difference = cost[row_ind, col_ind].sum()

    return total_dist_difference, col_ind




class Main:
    def __init__(self, load=True, superquadric_visualize=True):
        self.load = load
        self.superquadric_visualize = superquadric_visualize

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path("grasp_generator")
        self.product = "Limonade"
        self.nr_superquadrics = 3


        self.distance = [0.05, 0.3, 0.2]
        self.size = np.array([
                        [1.,2.,3.], 
                        [3.5,2.1,1.1], 
                        [3,2,3]
                        ])

        self.OutlierRatio = [0.6,0.9]           # prior outlier probability [0, 1) (default: 0.1)
        self.MaxIterationEM = [15, 20]                # maximum number of EM iterations (default: 20)
        self.ToleranceEM = 0.001                # absolute tolerance of EM (default: 1e-3)
        self.RelativeToleranceEM = 0.1          # relative tolerance of EM (default: 1e-1)
        self.MaxOptiIterations = 2              # maximum number of optimization iterations per M (default: 2)
        self.Sigma = [0.3, 0.35]                        # 0.3 initial sigma^2 (default: 0 - auto generate)
        self.MaxiSwitch = 1                     # maximum number of switches allowed (default: 2)
        self.AdaptiveUpperBound = True          # Introduce adaptive upper bound to restrict the volume of SQ (default: false)
        self.Rescale = True                     # normalize the input point cloud (default: true)
        self.MaxLayer = 3                       # maximum depth
        self.Eps = [0.35, 0.03]                         # 0.03 IMPORTANT: varies based on the size of the input pointcoud (DBScan parameter)
        self.MinPoints = [200, 300]                    # DBScan parameter required minimum points


    def run(self):
        if self.load:
            partial_pointcloud = load_pointcloud(
                self.package_path + "/data/" + self.product)

        headerList = [
            'OutlierRatio',
            'MaxIterationEM',
            'ToleranceEM',
            'RelativeToleranceEM',
            'MaxOptiIterations',
            'Sigma',
            'MaxiSwitch',
            'AdaptiveUpperBound',
            'Rescale',
            'MaxLayer',
            'Eps',
            'MinPoints',
            'nr_superquadrics',
            'total_dist_difference',
            'total_size_difference',
            'total_score']

        total_list = []
        total_length = len(self.OutlierRatio)* len(self.MaxIterationEM)* len(self.Sigma)* len(self.Eps)* len(self.MinPoints)
        count = 1

        for a in range(len(self.OutlierRatio)): 
            for b in range(len(self.MaxIterationEM)):
                for c in range(len(self.Sigma)):
                    for d in range(len(self.Eps)):
                        for e in range(len(self.MinPoints)):

                            list_quadrics = fixed_nr_ems(
                                partial_pointcloud, 
                                OutlierRatio = self.OutlierRatio[a],             
                                MaxIterationEM = self.MaxIterationEM[b],            
                                ToleranceEM = self.ToleranceEM,            
                                RelativeToleranceEM = self.RelativeToleranceEM,      
                                MaxOptiIterations = self.MaxOptiIterations,          
                                Sigma = self.Sigma[c],                    
                                MaxiSwitch = self.MaxiSwitch,                 
                                AdaptiveUpperBound = self.AdaptiveUpperBound,      
                                Rescale = self.Rescale,                 
                                MaxLayer = self.MaxLayer,                   
                                Eps = self.Eps[d],                    
                                MinPoints = self.MinPoints[e],               
                                nr_superquadrics = self.nr_superquadrics)

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


                            total_dist_difference, _ = dist_difference(self.distance, superquadrics)
                            total_size_difference, _, col_ind = size_diff(self.size, superquadrics)
                            total_score = total_size_difference + total_dist_difference

                            print("   ***total distance difference", total_dist_difference)
                            print("   ***total size difference", total_size_difference)
                            print("   ***total error", total_score)

                            # for z in range(len(col_ind)):
                            #     print("object: ", z, " is matched with quadric: " ,col_ind[z])
                            # print("\n")

                            mylist = [
                                self.OutlierRatio[a],
                                self.MaxIterationEM[b],
                                self.ToleranceEM,
                                self.RelativeToleranceEM,
                                self.MaxOptiIterations,
                                self.Sigma[c],
                                self.MaxiSwitch,
                                self.AdaptiveUpperBound,
                                self.Rescale,
                                self.MaxLayer,
                                self.Eps[d],
                                self.MinPoints[e],
                                self.nr_superquadrics,
                                total_dist_difference, 
                                total_size_difference, 
                                total_score]

                            total_list.append(mylist)


                            print("Iteration ", count, "/", total_length)
                            count += 1


        t = time.localtime()
        timestamp = time.strftime('%b-%d-%Y_%H%M%S', t)
        with open(self.package_path + "/parameter_tuning/parameters-" + timestamp + ".csv", 'w', newline='') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            wr.writerow(headerList)
            for i in range(len(total_list)):
                wr.writerow(total_list[i])


        print("finished - uploaded data to ", self.package_path + "/parameter_tuning/parameters-" + timestamp + ".csv")





        # if self.superquadric_visualize:
        #     visualize_superquadric(partial_pointcloud, superquadrics)


if __name__ == "__main__":
    start = Main()
    start.run()
