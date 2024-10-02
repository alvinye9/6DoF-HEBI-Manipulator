# -*- coding: utf-8 -*-
"""
Created on Fri Feb 18 09:46:11 2022

@author: isaac.hagberg
"""

#Homogenous Transform Code
import numpy as np

def rot3(theta, axis):
    # function uses radians to create rotation matrix
    theta = 90 *np.pi/180
    c = np.cos(theta) 
    s = np.sin(theta)     
    if axis == 1: # basic rotation, x axis
        R = np.array([[1,   0,   0] ,
               [0 ,  c,  -s], 
               [0  , s  , c ]]) 
    elif axis == 2: # basic rotation, x axis
        R = [[  c,   0,   s,],
               [0,   1,   0], 
              [-s,   0,   c ]] 
    elif axis == 3: # basic rotation, x axis
        R = [ [ c,  -s ,  0],
               [s ,  c   ,0],  
               [0   ,0   ,1 ]] 
    else: # no rotation
        R = np.eye(3) 
    return R
a = rot3(90 *np.pi/180, 1)
print(a)

def getH_fromRp( R, p): 
    # R: 3x3 rotation matrix
    # X: [x;y;z] position 
    # assemble into transformation matrix
    # if np.shape(p) != (3,1):
    #     p = np.array([p])
        
    H = np.hstack((R,p))
    H = np.vstack((H,np.array([0,0,0,1])))
    # H = np.array([[    R     , p ],
    #      [np.array([0,0,0,1])] ])
    return H
# v_in = np.reshape(np.array([1,2,3]),(3,1))
# v_in = np.array([1,2,3])

# v_in = np.append(v_in, np.array([1]))
# np.reshape(v_in, (4,1))

def getvec_thruH(H_in_out , v_in):
    v_in = np.append(v_in, np.array([1]))
    v_in = np.reshape(v_in, (4,1))
    v = H_in_out @ v_in
    v_out = v[:3, -1]
    return(v_out)

# getvec_thruH(A, v_in)
# H_in_out = A
def H_inv(H):
    R = H[:3,:3]
    p = np.reshape((H[0:3, -1]), (3,1))
    
    p_inv = -np.transpose(R) @ p
    R_inv = np.transpose(R)
    H_inv = np.hstack((R_inv, p_inv))
    H_inv = np.vstack((H_inv, np.array([0,0,0,1])))
    H_inv = np.linalg.inv(H)
    print(H_inv @ H)
    return H_inv
# getvec_thruH(a,)

# p = np.array(np.transpose([[1,1,1]]))   
# getH_fromRp(a,p)

# R_A_O = rot3(90*np.pi/180, 3)
# p_A_O = np.array((2,2,0)).reshape(3,1)
# R = R_A_O
# p = p_A_O

# H_ = getH_fromRp( R_A_O, p_A_O)

A = np.array([[1,0,0,0],[0,2,0,0],[3,3,3,3],[0,0,0,1]])
H_inv(A)
A@np.linalg.inv(A)

"""
This demo calculates multiple things for different scenarios.
Here are the defined reference frames:
TAG:
                A y
                |
                |
                |tag center
                O---------> x
CAMERA:
                X--------> x
                | frame center
                |
                |
                V y
F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis
The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)
We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

#--- Define Tag
#set this below in line 177
id_to_find  = 1
marker_size  = 10 #- [cm]


#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.

    
    
    

R_T_C = rot3(roll, 1) * rot3(pitch,2) * rot3(yaw,3)

#depth data gives v_T_C
v_T_C = [1,2,1]
H_T_C = getH_fromRp(R_T_C, v_T_C)

H_C_T = H_inv(H_T_C)

H_C_0 = H_T_0 @ H_C_T

#now i can find the position of new objects

pos_object1 = getvec_thruH(H_C_0 , v_obj_camera)



# import module
import numpy as np
  
# create array
print("\nArray:")
arr = np.array([[1, 2, 3], 
                [4, 5, 6], 
                [7, 8, 9]])
print(arr)
print(type(arr))
  
# apply method
lis = arr.tolist()
  
# display list
print("\nList:")
print(lis)
print(type(lis))
    
