#!/usr/bin/env python

import numpy as np
import math

def inverse_kinematics(x, y, z):
    task_space = np.array([x, y, z])
    J_matrix = np.array([[ 0.66666667,  0,              0.33333333], 
                         [ -0.33333333, 0.57735027,    0.33333333], 
                         [ -0.33333333, -0.57735027,     0.33333333]])
    joint_space = np.dot(J_matrix, task_space) # J_matrix @ task_space gives syntax error
    return 255 * joint_space

#   numpy array as input
#   TODO create mobot object
def forward_kinematics(theta):
    x = theta[0] + theta[1] * math.cos(math.radians(120)) + theta[2] * math.cos(math.radians(240))
    y = theta[1] * math.sin(math.radians(120)) + theta[2] * math.sin(math.radians(240))
    z = theta[0] + theta[1] + theta[2]
    task_space = [x, y, z]
    return task_space

def calculate_ang_pos(theta):
    radius_wheel = 0.024 # TODO measure properly
    radius_mobot = 0.13 # TODO measure properly
    lin_pos = theta * radius_wheel
    ang_pos = (lin_pos[0] + lin_pos[1] + lin_pos[2]) / radius_mobot
    return ang_pos

'''   3 args as input
def forward_kinematics(t0, t1, t2):
    x = t0 + t1 * math.cos(math.radians(120)) + t2 * math.cos(math.radians(240))
    y = t1 * math.sin(math.radians(120)) + t2 * math.sin(math.radians(240))
    z = t0 + t1 + t2
    task_space = [x, y, z]
    return task_space
'''
    
#J_matrix = np.array([[ -2/3 , 0, 1/3], 
#                         [ 1/3  , 1 ,1/3], 
#                         [ 1/3  , 2 ,1/3]])
    
# J_matrix = np.array([[ 1 , -1/2, -1/2], 
#                     [ 0, math.sqrt(3)/2, -math.sqrt(3)/2], 
#                     [ 1/R, 1/R, 1/R]])
#                     
#print(np.linalg.inv(J_matrix))
    
