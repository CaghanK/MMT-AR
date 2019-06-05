#!/usr/bin/env python

from Mobot import inverse_kinematics
from Mobot import forward_kinematics

joint_space = inverse_kinematics(1, 1, 1)
print(joint_space)

task_space = forward_kinematics(joint_space[0], joint_space[1], joint_space[2])
print(task_space)

#----------> 
#inv_J = np.array([[ 1, -math.cos(math.radians(60)), -math.cos(math.radians(60))], 
#                         [ 0, -math.sin(math.radians(60)), math.sin(math.radians(60))], 
#                         [ 1, 1, 1]])
#J = np.linalg.inv(inv_J)
#print(J)
