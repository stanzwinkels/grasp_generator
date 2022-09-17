#!/usr/bin/env python

# Libraries
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R


import pdb

def categorize_superquadric(superquadric):
    """
    Function to categorize superquadric functions into cylindrical and rectangular.
        - maybe add more categorizations: Handle, Cap, Scoup, Cut?
    """

    categorization = []
    theshold = 0.7
    for i in range(len(superquadric)):
        shape = superquadric[i][0:2]
        scale = superquadric[i][2:5]
        pos = superquadric[i][9:12]
        print("primitive_values("+ str(i+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +").")
        if all(shape < theshold) and (-shape[0]+theshold-shape[1]) >= 0:
            categorization.append("rectangular")
        elif (shape[0]< theshold ) and (shape[1] > (2-theshold)) and (shape[0] -theshold - shape[1]) >= 0:
            categorization.append("rectangular")
        elif ((-shape[0]+theshold-shape[1]) < 0) and (shape[0] -(2-theshold) - shape[1]) < 0 and shape[1] < (2-theshold):
            categorization.append("cylindrical")
        else:
            categorization.append("unknown")
    assert len(categorization) == len(superquadric)
    return categorization
    

def grasp_quadric_distance(superquadrics, grasps):
    """
    Function to match grasps with superquadric shapes.  
    """ 
    closest_primitive = np.zeros(len(grasps))
    distances = np.zeros(len(grasps))

    for j in range(len(grasps)): 
        grasp_point = [grasps[j][1].position.x, grasps[j][1].position.y, grasps[j][1].position.z]
        

        for i in range(len(superquadrics)):
            eps = superquadrics[i][0:2]
            scale = superquadrics[i][2:5]
            quaternion = superquadrics[i][5:9]
            translation = superquadrics[i][9:12]
            q = R.from_quat(quaternion)
            rotation_matrix = q.as_dcm()


            new_grasp_point =  np.dot(np.linalg.inv(rotation_matrix), (grasp_point - translation))           
            distance = radial_euclidean_distance(new_grasp_point, scale, eps)

            if distance < distances[j] or distances[j] == 0:
                closest_primitive[j] = (i+1)
                distances[j] = distance

    return closest_primitive, distances


def radial_euclidean_distance(point, scale, eps):
    """
    Function to check if a point lies within a specified superquadric function
    """
    F =  ((abs(point[0])/scale[0])**(2/eps[1]) + (abs(point[1])/scale[1])**(2/eps[1]))**(eps[1]/eps[0]) + (abs(point[2])/scale[2])**(2/eps[0])
    distance = np.abs(np.linalg.norm(point)) * np.abs(1 - F**(-eps[0]/2))
    return distance
    
    
    
def comp_superquadric(points, scale, eps):
    """
    Function to check if a point lies within a specified superquadric function

    Return
    ---------
    values : array
        score, < 1 inside superquadric, = 1 point lies on boundary, > 1, outside superquadric
    """

    values = []
    for i in range(len(points)):
        value = ((abs(points[i][0])/scale[0])**(2/eps[1]) + (abs(points[i][1])/scale[1])**(2/eps[1]))**(eps[1]/eps[0]) + (abs(points[i][2])/scale[2])**(2/eps[0])
        values.append(value)
    return np.array(values)


def superquadric_overlapping(superquadrics, threshold_max_overlap = 0.5):
    """
    Function compute the % of overlap of every generated superquadric function. 

    Return
    ---------
    new_superquadric : array
        filtered superquadric 
    score : 
        matrix with % of overlap for every superquadric
    """            
    score = np.zeros((len(superquadrics), len(superquadrics)))
    for i in range(len(superquadrics)):
        shape_orig = superquadrics[i][0:2]
        scale_orig = superquadrics[i][2:5]
        quaternion_orig = superquadrics[i][5:9]
        translation_orig = superquadrics[i][9:12]
        q_orig = R.from_quat(quaternion_orig)
        rotation_matrix_orig = q_orig.as_dcm()

        for j in [x for x in range(len(superquadrics)) if x != i]:
            shape = superquadrics[j][0:2]
            scale = superquadrics[j][2:5]
            quaternion = superquadrics[j][5:9]
            translation = superquadrics[j][9:12]
            q = R.from_quat(quaternion)
            rotation_matrix = q.as_dcm()
            new_points = showSuperquadrics(shape, scale, rotation_matrix, translation)

            for k in range(len(new_points)):
                new_points[k] =  np.dot(np.linalg.inv(rotation_matrix_orig), (new_points[k] - translation_orig))

            mesh = trimesh.convex.convex_hull(new_points)
            points = trimesh.sample.sample_surface_even(mesh, 5000)
            points = np.array(points[0])
            values = comp_superquadric(points, scale_orig, shape_orig)
            score[i][j] = (sum(i < 1.0001 for i in values))/float(len(values))
    max_values = np.max(score, axis=0)
    indexes = ([idx for idx in range(len(max_values)) if max_values[idx] > threshold_max_overlap])
    filt_superquadrics = [v for i, v in enumerate(superquadrics) if i not in indexes]                        
    print("\n score \n", score)
    return filt_superquadrics, score


def showSuperquadrics(shape, scale, rotation, translation, threshold = 1e-2, num_limit = 10000, arclength = 0.3):
    # avoid numerical instability in sampling
    if shape[0] < 0.007:
        shape[0] = 0.007
    if shape[1] < 0.007:
        shape[1] = 0.007
    # sampling points in superellipse    
    point_eta = uniformSampledSuperellipse(shape[0], [1, scale[2]], threshold, num_limit, arclength)
    point_omega = uniformSampledSuperellipse(shape[1], [scale[0], scale[1]], threshold, num_limit, arclength)

    points_origin = []
    new_points = []
    for m in range(np.shape(point_omega)[1]):
        for n in range(np.shape(point_eta)[1]):
            point_temp = np.zeros(3)
            point_temp[0 : 2] = point_omega[:, m] * point_eta[0, n]
            point_temp[2] = point_eta[1, n]
            points_origin.append(point_temp)
            point_temp = np.dot(rotation, point_temp) + translation
            new_points.append(point_temp)    
    points_origin = np.array(points_origin)
    new_points = np.array(new_points)

    return new_points



def uniformSampledSuperellipse(epsilon, scale, threshold = 1e-2, num_limit = 10000, arclength = 0.02):
    # initialize array storing sampled theta
    theta = np.zeros(num_limit)
    theta[0] = 0
    for i in range(num_limit):
        dt = dtheta(theta[i], arclength, threshold, scale, epsilon)
        theta_temp = theta[i] + dt
        if theta_temp > np.pi / 4:
            theta[i + 1] = np.pi / 4
            break
        else:
            if i + 1 < num_limit:
                theta[i + 1] = theta_temp
            else:
                raise Exception(
                'Number of the sampled points exceed the preset limit', \
                num_limit,
                'Please decrease the sampling arclength.'
                )
    critical = i + 1

    for j in range(critical + 1, num_limit):
        dt = dtheta(theta[j], arclength, threshold, np.flip(scale), epsilon)
        theta_temp = theta[j] + dt
        
        if theta_temp > np.pi / 4:
            break
        else:
            if j + 1 < num_limit:
                theta[j + 1] = theta_temp
            else:
                raise Exception(
                'Number of the sampled points exceed the preset limit', \
                num_limit,
                'Please decrease the sampling arclength.'
                )
    num_pt = j
    theta = theta[0 : num_pt + 1]

    point_fw = angle2points(theta[0 : critical + 1], scale, epsilon)
    point_bw = np.flip(angle2points(theta[critical + 1: num_pt + 1], np.flip(scale), epsilon), (0, 1))
    point = np.concatenate((point_fw, point_bw), 1)
    point = np.concatenate((point, np.flip(point[:, 0 : num_pt], 1) * np.array([[-1], [1]]), 
                           point[:, 1 : num_pt + 1] * np.array([[-1], [-1]]),
                           np.flip(point[:, 0 : num_pt], 1) * np.array([[1], [-1]])), 1)

    return point

def dtheta(theta, arclength, threshold, scale, epsilon):
    # calculation the sampling step size
    if theta < threshold:
        dt = np.abs(np.power(arclength / scale[1] +np.power(theta, epsilon), \
             (1 / epsilon)) - theta)
    else:
        dt = arclength / epsilon * ((np.cos(theta) ** 2 * np.sin(theta) ** 2) /
             (scale[0] ** 2 * np.cos(theta) ** (2 * epsilon) * np.sin(theta) ** 4 +
             scale[1] ** 2 * np.sin(theta) ** (2 * epsilon) * np.cos(theta) ** 4)) ** (1 / 2)
    
    return dt

def angle2points(theta, scale, epsilon):

    point = np.zeros((2, np.shape(theta)[0]))
    point[0] = scale[0] * np.sign(np.cos(theta)) * np.abs(np.cos(theta)) ** epsilon
    point[1] = scale[1] * np.sign(np.sin(theta)) * np.abs(np.sin(theta)) ** epsilon

    return point

