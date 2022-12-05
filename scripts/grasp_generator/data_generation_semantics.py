#!/usr/bin/env python3.6
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from itertools import cycle
import pandas as pd
import matplotlib.pyplot as plt


from pick import pick
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
import seaborn as sns
from sklearn.metrics import classification_report
from sklearn.model_selection import GridSearchCV


from matplotlib.colors import ListedColormap

palette = cycle(['grey']) 

import pdb
from pyquaternion import Quaternion

class SingleSuperQuadric():
    def __init__(self, eps, dimension , position = np.array([0,0,0]), quaternion = np.array([1,0,0,0])):
        self.alpha = eps[0]
        self.beta = eps[1]
        self.x = dimension[0]
        self.y = dimension[1]
        self.z = dimension[2]
        self.position = position
        self.quaternion = Quaternion(quaternion)
        self.points = []

    def fexp(self, x,p):
        """a different kind of exponentiation"""
        return (np.sign(x) * (np.abs(x)**p))

    def tens_fld(self, A,B,C,P,Q):
        """this module plots superquadratic surfaces with the given parameters"""
        phi, theta = np.mgrid[0:np.pi:80j, 0:2*np.pi:80j]
        x = (A * (self.fexp(np.sin(phi),P)) * (self.fexp(np.cos(theta),Q))).flatten()
        y = (B * (self.fexp(np.sin(phi),P)) * (self.fexp(np.sin(theta),Q))).flatten()
        z = (C * (self.fexp(np.cos(phi),P))).flatten()
        points = np.column_stack((x,y,z))
        return points

    def transformation(self, points): 
        for idx, point in enumerate(points):
            points[idx] = self.quaternion.rotate(point) + self.position
        return points

    
    def visualize(self):
        points = self.tens_fld(self.x, self.y, self.z, self.alpha, self.beta)
        points = self.transformation(points)
        return points
    
    def plot(self):
        self.points = self.tens_fld(self.x, self.y, self.z, self.alpha, self.beta)
        self.points = self.transformation(self.points)

        fig = go.Figure()
        fig.add_trace(
            go.Mesh3d(
                name = 'eps1 = ' + str(round(self.alpha,2)) + ' eps2 = ' + str(round(self.beta,2)),
                showlegend=True,
                x=self.points[:,0], 
                y=self.points[:,1], 
                z=self.points[:,2], 
                alphahull= 0, 
                color = 'grey',
                opacity=0.70,
                showscale= False))
        fig.update_layout(scene_aspectmode='data')  
        fig.show()
        return

    
data_generation = True

if data_generation:
    range_dim = np.array([0.01, 0.15])
    range_eps = np.array([0.8, 1.2])
    len_data = 100

    np.random.seed(43)
    range_dim = np.random.uniform(range_dim[0], range_dim[1], size=(len_data, 3))
    range_eps = np.random.uniform(range_eps[0], range_eps[1], size=(len_data, 2))
    superquadrics = np.concatenate((range_eps, range_dim), axis=1)
    quaternion = np.array([1,0,0,0])

    appended_data = []

    count = 1
    for superquadric in superquadrics:
        quadric = SingleSuperQuadric(superquadric[:2], superquadric[2:5])
        points = quadric.plot()

        title = str(count) +'/'+  str(len(superquadrics)) + ': Please choose crresponding shape: ' 
        options = ['Cuboid [0.1 ; 0.1]', 'Cylinder [0.1 ; 1.0]', 'Sphere [1.0 ; 1.0]']
        option, index = pick(options, title, indicator='=>', default_index=2)
        
        data = np.append(superquadric, int(index))
        appended_data.append(data)
        print('Currently labelled: ' + str(count) +'/'+  str(len(superquadrics)))
        count += 1

    df_data = np.array(appended_data)
    df = pd.DataFrame(appended_data)
    df.to_csv('semantic_reasoning_additional_2.csv', index=False)