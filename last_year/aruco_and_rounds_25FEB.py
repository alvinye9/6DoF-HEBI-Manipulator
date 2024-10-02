#!/usr/bin/env python3

#Author: Isaac Hagberg
#5 MAY 2022

#Starting basis for code was from zptang (UMass Amherst)
## Detector for ArUco Markers with Intel RealSense Camera
import numpy as np
from decimal import *
getcontext().prec = 4

try:
    import pyrealsense2 as rs
    
    from std_msgs.msg import Float64MultiArray
    import rospy
except:
    print("WINDOWS DEVICE")
import matplotlib.pyplot as plt
# try:
#     from RoundDetection import FindRounds
# except:
#     print("be free my child")
import cv2
import time
import math
from RoundDetection_withES import FindRounds

#PUT THIS ONE BACK IN FOR THE REAL THING
#the parameters for the Evolutionary Strategy still need tuning
from TuningHoughTranform_copyForIntegration import *

#%%
flag_sender = 0
holder_of_old_detected_rounds = []
number_of_rounds = 0
ES_flag = 2
reset_threshold = 50 #make sure this is the same as the value in rounddetection_withES
old_circles = []
flag_HCO = 0   #after the first 5 frames where tag 1 is detected, the HCO matrix will no longer be calculated. This fixes the error of having multiple tags in the frame messing things up. Initially only have tag 1 in the frame!

buffer = 1 #this variable ensures that the depth projection is taken from a point on the rim or outside on the round
########################
ROS_ON = 1
#high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius
param = [30,150,17,12,15]
tag_position = [0,.515  ,0.1125] #this is tag number 1
#tag_position = [0,0,.1125]
p_T_0= np.array(tag_position)  #Tag Position Relative to the world (i.e., origin)
livestream_camera = 1

########################

#camera frame size is 640 x 480
frame_width = 640
frame_height = 480
radius_of_round = 0.03 #[m]


id_to_find  = 1
marker_size  = 0.05 #- [cm]
circles3D = []

#%%
# i = 1.0

# def talker(jeff):
#     pub = rospy.Publisher('finalTesting',Float64MultiArray,queue_size=20)
#     rospy.init_node('aruco_and_detector6', anonymous=True)
#     rate = rospy.Rate(100)  
#     my_msg = Float64MultiArray()  
# #    global i
# #    jeff = [3,4,5,i]

#     my_msg.data = jeff
#     #force everything to be a float
#     pub.publish(my_msg)
#     rate.sleep()
    
# def sendTagsROS(jeff):
#     pub = rospy.Publisher('TagPoints',Float64MultiArray,queue_size=20)
#     rospy.init_node('putDownPoints', anonymous=True)
#     rate = rospy.Rate(100)  
#     my_msg = Float64MultiArray()   
#     global i
#     jeff = [3,4,5,i]
#     my_msg.data = jeff
#     #force everything to be a float
#     pub.publish(my_msg)
#     rate.sleep()
# print("Warming Up...")
# def Ethan():
#     i = 0.34
#     while True:
#         pub = rospy.Publisher('finalTesting',Float64MultiArray,queue_size=20)
#         rospy.init_node('chuck_norris', anonymous=True)
#         print(i)
#         while True:
#             my_msg = Float64MultiArray()
#             rate = rospy.Rate(120)
#         #    d=[[1.2354567, 99.7890, 67.654236], [67.875, 90.6543, 76.5689], [65.3452, 45.873, 67.8956]]
#         #    d=[[float(d[i][j]) for j in range(len(d))] for i in range(len(d[0]))]
            
#             d= [23.234, 234.23, 234.345,234.23,423.34,234222.222343434,i]
#             d = np.array([[1,2,3],[4,5,6],[7,8,9]])
#             d = d.reshape(d.size).tolist()
#             i +=1.4
#             my_msg.data = d
#             pub.publish(my_msg)
#             rate.sleep()
#
#while True:            
#    sendTagsROS([3,4,5])
#    talker([2,2,2])
##j = 0
#while True:
#    j+=1
#    print(j)
#talker()
#Ethan()

class GetDepth:

    def __init__(self):
        self.trajectory = dict()

    def clear(self):
        self.trajectory = dict()

    def updateDepth(self, aligned_frame, detectorResult):
        corners, ids, rejected = detectorResult

        timestamp = aligned_frame.get_timestamp()
        depth_frame = aligned_frame.get_depth_frame()
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        # depth_image = np.asanyarray(depth_frame.get_data())
        center_id = []
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                coord = self._getCoordinate(depth_intrinsics, depth_frame, (topRight, bottomRight, bottomLeft, topLeft))
                center_id += [[coord, markerID]]
                
                # center_id += markerID
                # print(center_id)
                if coord is not None:
                    self._add(timestamp, markerID, coord)
            return center_id, markerCorner







    def _add(self, timestamp, id, coord):
        if id not in self.trajectory.keys():
            self.trajectory[id] = list()
        x, y, z = coord
        self.trajectory[id].append((timestamp, x, y, z))
        # print(self.trajectory[0][-10:], '\n')

    def _getCoordinate(self, depth_intrinsics, depth_frame, markerPos):
        # for simplicity, just use the center point to extract the 3D coordinate
        # TODO: In future update, we can use fillPoly to mask the marker, and compute the average 3D coordinate
        topRight, bottomRight, bottomLeft, topLeft = markerPos
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        depth = depth_frame.get_distance(cX, cY)
        if depth > 0:
            dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cX, cY], depth)
            # print(dist)
            return dist
        else:
            return None
    

#%% This is for finding the rotation and translation of the camera wrt a Aruco Tag
#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

#%%
#Homogenous Transform Code

def rot3(theta, axis):
    # function uses radians to create rotation matrix
    # theta = 90 *np.pi/180
    c = np.cos(theta) 
    s = np.sin(theta)     
    if axis == 1: # basic rotation, x axis
        R = np.array([[1,   0,   0] ,
               [0 ,  c,  -s], 
               [0  , s  , c ]]) 
    elif axis == 2: # basic rotation, x axis
        R = np.array([[  c,   0,   s,],
               [0,   1,   0], 
              [-s,   0,   c ]] )
    elif axis == 3: # basic rotation, x axis
        R = np.array([ [ c,  -s ,  0],
               [s ,  c   ,0],  
               [0   ,0   ,1 ]] )
    else: # no rotation
        R = np.eye(3) 
    return R

def getH_fromRp( R, p): 
    # R: 3x3 rotation matrix
    # X: [x;y;z] position 
    # assemble into transformation matrix
    if np.shape(p) != (3,1):
        p = np.reshape(p, (3,1))
        
    H = np.hstack((R,p))
    H = np.vstack((H,np.array([0,0,0,1])))
    # H = np.array([[    R     , p ],
    #      [np.array([0,0,0,1])] ])
    return H


def getvec_thruH(H_in_out , v_in):
    v_in = np.append(v_in, np.array([1]))
    try:
        v_in = np.reshape(v_in, (4,1))
    except:
        print(f"ERRORRORORORORO. v_in = {v_in}")
        input()
    v = H_in_out @ v_in
    v_out = v[:3, -1]
    return(v_out)

def H_inv(H):
    R = H[:3,:3]
    p = np.reshape((H[0:3, -1]), (3,1))
    
    p_inv = -np.transpose(R) @ p
    R_inv = np.transpose(R)
    H_inv = np.hstack((R_inv, p_inv))
    H_inv = np.vstack((H_inv, np.array([0,0,0,1])))
    H_inv = np.linalg.inv(H)
    # print(H_inv @ H)
    return H_inv


R_T_0 = rot3( 0 , 3 )*rot3( 0 , 1 )*rot3( 0 , 2 )
H_T_0 = getH_fromRp( R_T_0 , p_T_0 )






#%%


def findDepth(algined_frame, circles):
    # TODO: if there is not depth data grabbed for a point then continue to run frames until there is one. This will go in the except statement 
    aligned_frame = frame
    depth_frame = aligned_frame.get_depth_frame()
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    circ = []
    unknown_printables = np.array([])
    print(f'Circles sent to findDepth function: {circles}')
    for i, center in enumerate(circles):  # if issues come up look at the index number to see how many layers are being peeled back.
#        print(f'round {i}, find Depth for = {center}')
        ####
        #update: if the pixel is on the right of the screen, the program will grab the rightmost data point (opposite for the left). This allows a better approximation of the center of the round.
        ####
        print("Line 306")
        if int(round(center[0])) > frame_width/2:
            x = int(round(center[0])) + int(round(center[2])) + buffer #right side, positive x. add the radius 
            y = int(round(center[1]))
            
            #while:
            #depth == 0
            #continue pulling frames for 100 frames
            #if depth > 0 :
            #h    break
            
# NEW CODE IN PROGRESS WORKS            
             
            print("312\n")
            depth = depth_frame.get_distance(x, y)
            print(depth)
#            input("line315")
            if depth > 0:
                dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
                #shift x coordinate back to the middle of the circle
                dist[0] = dist[0] - radius_of_round
                circ = np.append(circ, [dist])
                # print('add to circ')
                print(f'\n_____________successful grab of depth for round {i+1}. First time\n\n')
            else:
                print(f'Error with round {i+1}. Trying other side.')
#                input()
                x = int(round(center[0])) - int(round(center[2]))  -  buffer#right side, positive x. add the radius 
                
                print("326\n")
#                input()
                depth = depth_frame.get_distance(x, y)
                if depth > 0:
                    dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
                    dist[0] = dist[0] + radius_of_round
                    circ = np.append(circ, [dist])
                    # print('add to circ')
                    print(f'\n_____________successful grab of depth for round {i+1}. Second time\n')
                else:
                    print(center)
#                    unknown_printables.append(center.tolist())
#                    unknown_printables = np.append(unknown_printables, center)
                    print(f'unknown_printables = {unknown_printables}')
                    #TODO fix this
#                    input()

# OLD CODE THAT WORKS
#            try:
#                print("312\n")
#                depth = depth_frame.get_distance(x, y)
#                print(depth)
#                input("line315")
#                if depth > 0:
#                    dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
#                    #shift x coordinate back to the middle of the circle
#                    dist[0] = dist[0] - radius_of_round
#                    circ = np.append(circ, [dist])
#                    # print('add to circ')
#                    print(f'\n_____________successful grab of depth for round {i+1}. First time\n\n')
#                else:
#                    print("you're stupid")
#                    input("somehow depth is less than zero")
#            except:
#                print(f'Error with round {i+1}. Trying other side.')
#                input()
#                x = int(round(center[0])) - int(round(center[2]))  -  buffer#right side, positive x. add the radius 
#                try:
#                    print("326\n")
#                    input()
#                    depth = depth_frame.get_distance(x, y)
#                    if depth > 0:
#                        dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
#                        dist[0] = dist[0] + radius_of_round
#                        circ = np.append(circ, [dist])
#                        # print('add to circ')
#                        print(f'\n_____________successful grab of depth for round {i+1}. Second time\n')
#                except: 
#                    print(f"BAD DATA")
#                    input('bad data')
#        print("338 in the middle\n")
                
#        input('round epth?')           
#        else: 
#            x = int(round(center[0])) - int(round(center[2])) #left side, negative x. subtract the radius 
#            y = int(round(center[1]))
#            try:
#                depth = depth_frame.get_distance(x, y)
#                if depth > 0:
#                    dist = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
#                    dist[0] = dist[0] + radius_of_round
#                    circ = np.append(circ, [dist])
#                    # print('add to circ')
#                    print(f'_____________successful grab of depth for round {i+1}. Second time')
#            except: 
#                print(f"Error with round {i+1}")
#    input('hey we made it to 397')
    if len(circ)>1:
        circ = circ.reshape(int(len(circ) / 3), 3)

    return circ, unknown_printables


def sendTags(center_id, H_C_O):
    
    tag_locations= []
    for center in center_id:
        if center[1] != 1:
#            print(f'Location of ID tag {center[1]} relative to camera is: {center[0]}')
#            input()
            p_Ttemp_C = np.array([center[0]]) #this line may be wrong
            try:
                #sometimes the tag does not get a full read and causes errors
                dist = getvec_thruH(H_C_O, p_Ttemp_C)
            except:
                pass
#            dist[1] = tag_position[1] - 
            dist[1] = dist[1] - 2*(dist[1] - tag_position[1])  #I'm missing a rotation somewhere so this makes he correction
#            dist[0] = -dist[0]
            tag_locations.append(dist)
    tag_locations = np.array(tag_locations)
    tag_locations = tag_locations.reshape(tag_locations.size).tolist() 
    return tag_locations


#%%


if __name__ == '__main__':
    import time
    import cv2
    
    from ArUcoDetector import ArUcoDetector
    from Camera import Camera
    center_id = []
    visualize = True
    dict_to_use = 'DICT_4X4_50'
    arucoDetector = ArUcoDetector(dict_to_use)
    rvec = []
    tvec = []
    
    tracker = GetDepth()

    camera = Camera()
    camera.startStreaming()
    #allow the camera to warm up
    time.sleep(1)
    start_time = time.time()
    
    
#    talker([4.234])
    
    try:
        while True:
            frame = camera.getNextFrame()
            depth_image, color_image = camera.extractImagesFromFrame(frame)
            
            
            
            
            # Remove unaligned part of the color_image to grey
            grey_color = 0#was 153
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            masked_color_image = np.where(depth_image_3d <= 0, grey_color, color_image)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Detect markers and draw them on the images
            result = arucoDetector.detect(color_image)
            color_image, center_id = ArUcoDetector.getImageWithMarkers(color_image, result)
            masked_color_image, __ = ArUcoDetector.getImageWithMarkers(masked_color_image, result)
            depth_colormap, __ = ArUcoDetector.getImageWithMarkers(depth_colormap, result)


           #%%
           #tune the parameters on the first frame
           #inside the ES you will need to input the number of circles you expect to detect
            try:
                # print(f'ES_flag = {ES_flag}')
                if ES_flag == 0:
                    print("testing")
                    ES = EvolutionStrategy(3,15,fitness,[100,50,30,10,150], sigma = 15, frame = color_image)
                    
                    high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = ES.optimize()
                    ES_flag = 1
                    #cycle to the next frame for processing        
                    frame = camera.getNextFrame()
                    depth_image, color_image = camera.extractImagesFromFrame(frame)
                elif ES_flag == 3:
                    print("here")
                    param = list(map(int, input("Type parameter numbers with space: ").split()))
                    high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = param
                    print("Tune Again?")
                    ES_flag = input("If done enter 1. If not, enter 3")
                else:
#                    param = [50,100,20,3,25]
                    high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius = param

            except:
                if ES_flag == 0:
                    print("test fail")
                else: 
                    print("look at lines 172 to 186")
 

            #%%
            # use the roundDetection Object
            # print(f' Old = {old_circles}')
            # color_image, circles, old_circles, printables = FindRounds(color_image, old_circles).output()
            
            #PUT THIS LINE BACK IN AFTER DEMO
            color_image, circles, old_circles, printables, reset_num, edge_img = FindRounds(color_image, old_circles, high_gradient, low_gradient, accumulator_threshold, minRadius, maxRadius).output()
            print(f'printables = {printables}')
            
                # update: change "circles" to "printables"
            #this only runs 
            if printables is not None:
                if len(printables) > 0:
                    # remove the vote count from printables
                    printables = printables[:, :3]
                    printables_holder = printables
                    circles3D, unknown_printables = findDepth(frame, printables)
                    print(f'Circles3d = {circles3D}')
#                    print(f')
    #                    input() #TODO

                # # get center coord and the
            
            try:
                center_id, markerCorner = tracker.updateDepth(frame, result)
            except:
                print("No tags in frame")
            depth_frame = frame.get_depth_frame()
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            #%%3
            # Finding the Rotation Matrix for of the Camera wrt the world
            f_x= depth_intrinsics.fx
            f_y= depth_intrinsics.fy
            c_x= depth_intrinsics.ppx
            c_y = depth_intrinsics.ppy
            camera_matrix = np.array([[f_x,0,c_x],
                                 [0,f_y,c_y],
                                 [0 ,0 ,1 ]])
                       
                       
            dist_coeff= np.array([[depth_intrinsics.coeffs[0],depth_intrinsics.coeffs[1],depth_intrinsics.coeffs[2],depth_intrinsics.coeffs[3],depth_intrinsics.coeffs[4]]])
            
#            if len(center_id)>0:
#                ret = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.05, camera_matrix, dist_coeff)
#                try:
#                    rvec = ret[0][0, 0, :]
#                    tvec = ret[1][0, 0, :]
#                    font = cv2.FONT_HERSHEY_PLAIN
#                    cv2.aruco.drawAxis(color_image, camera_matrix,dist_coeff,rvec,tvec,0.05)
#                    #print(tvec[0], tvec[1], tvec[2])
#                    
#                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
#                    R_tc = R_ct.T
#                    
#                    #NOT SURE IF THIS IS CORRECT. SKANDER?
#                    R_flip = np.zeros((3,3),dtype=np.float32)
#                    R_flip [0,0] = 1.0
#                    R_flip [1,1] = -1.0
#                    R_flip [2,2] =-1.0
#                    # Get the attitude in terms of euler 321 (Needs to be flipped first
#                    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip *R_tc)
#                    
#                    
#                    pos_camera =np.matmul( -R_tc, np.matrix(tvec).T)
#                    
#                    roll_camera,pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
#                    
#                    
#                    # str_position = f'{pos_camera[0]}, {pos_camera[1]}, {pos_camera[2]} '
#                    # str_position2 = f'{math.degrees(roll_marker)}, {math.degrees(pitch_camera)}, {math.degrees(yaw_camera)} '
#                    # cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
#                    # cv2.putText(color_image, str_position2, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
#                    
#                    
#                    #finding the transform of the Camera wrt the origin H_C_O
#                    #Rotation Matrix of tag wrt the camera
#                    R_T_C = rot3(roll_camera, 1) * rot3(pitch_camera,2) * rot3(yaw_camera,3)
#                    
#                    
#                except:
#                    pass   
#            else:
#                pass
#                # cv2.destroyAllWindows()        
                       

 
            

#%%

            flag = 0
            tag_locations = []
            
            try:
                if len(center_id)>0:
                    for center in center_id:
                        if center[1] == 1:
                            # print(f"tag 1: {center}")
                            tag1 = center[0]
                            if flag_HCO < 5:
                                ret = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.05, camera_matrix, dist_coeff)
                                try:
                                    rvec = ret[0][0, 0, :]
                                    tvec = ret[1][0, 0, :]
                                    font = cv2.FONT_HERSHEY_PLAIN
                                    cv2.aruco.drawAxis(color_image, camera_matrix,dist_coeff,rvec,tvec,0.05)
                                    #print(tvec[0], tvec[1], tvec[2])
                                    
                                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                                    R_tc = R_ct.T
                                     
                                    #NOT SURE IF THIS IS CORRECT. SKANDER?
                                    R_flip = np.zeros((3,3),dtype=np.float32)
                                    R_flip [0,0] = 1.0
                                    R_flip [1,1] = -1.0
                                    R_flip [2,2] =-1.0
                                    # Get the attitude in terms of euler 321 (Needs to be flipped first
                                    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip *R_tc)
                                    
                                    
                                    pos_camera =np.matmul( -R_tc, np.matrix(tvec).T)
                                    
                                    roll_camera,pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                                    
                                    
                                    # str_position = f'{pos_camera[0]}, {pos_camera[1]}, {pos_camera[2]} '
                                    # str_position2 = f'{math.degrees(roll_marker)}, {math.degrees(pitch_camera)}, {math.degrees(yaw_camera)} '
                                    # cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                                    # cv2.putText(color_image, str_position2, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                                    
                                    
                                    #finding the transform of the Camera wrt the origin H_C_O
                                    #Rotation Matrix of tag wrt the camera
                                    R_T_C = rot3(roll_camera, 1) * rot3(pitch_camera,2) * rot3(yaw_camera,3)
                                    flag_HCO += 1
                        
                                except:
                                    pass
                        
                            
                            
                            
#                            print(f'This is the global reference; {tag1}')
#                            input()
                            flag +=1
                            
                            #create H_T_C to find H_C_T
                            p_T_C = np.array([center[0]])
                            H_T_C = getH_fromRp( R_T_C , p_T_C )
                            H_C_T = H_inv(H_T_C)
                            #create HT of camera wrt the origin
                            H_C_O = H_T_0 @ H_C_T
                            # print("\n*******************\nH_C_O created\n*****************\n")
                            
                        if center[1] == 2:
                            # print(f"tag 2: {center}")
                            tag2 = center[0]
                            flag +=1
                        if flag ==2:
                            try:
                                dist_between = transform(tag1, tag2)
                                print(f'Distance between Aruco Tags {dist_between}')
                                input()
                            except:
                                print("Bad data sent to the transform function")
            except:
                pass


                
            if len(circles3D) > 0 and reset_num == reset_threshold: #TODO be here. fix this later
                flag_sender = 1
                
                # print the distance from Tag 1 to all of the circles
                reshaper_value = len(circles3D)
                send_to_ROS = []
                for _circle in circles3D:
                    _circle = np.array(_circle)

                    dist = getvec_thruH(H_C_O, _circle)
#                        dist = dist.tolist()
                    # dist = transform(tag1, _circle)
                    dist[1] = dist[1] - 2*(dist[1] - tag_position[1])
                    
                    #reshape for matlab 
                    dist = [dist[1], -dist[0], dist[2]]
                    send_to_ROS.append(dist)
                holder_of_old_detected_rounds = send_to_ROS  
                number_of_rounds = len(holder_of_old_detected_rounds)/3

#                send_to_ROS = np.array(send_to_ROS)
#                send_to_ROS = send_to_ROS.reshape(send_to_ROS.size).tolist()
                # dist_0_round = transform(tag1, circles3D[0])
                # print(f'Distance from tag1 to the first circle {dist_0_round}')
#                try:
                print(f'*****send_to_ROS = {send_to_ROS}')
                print("sending to ROS...")
#                tags_to_receive = sendTags(center_id,H_C_O)
#                print(H_C_O) # if the camera is not parallel to the x and y axis then the matrix will not work. not sure why.
#                print(tags_to_receive)
#                print(f'Tag are at: {tags_to_receive}')
#                send_to_ROS.append([9999])
#                send_to_ROS.append(tags_to_receive)
#                
#                send_to_ROS = np.array(send_to_ROS)
#                send_to_ROS = send_to_ROS.reshape(send_to_ROS.size).tolist()
#                flat_list = [item for sublist in send_to_ROS for item in sublist] #flatten the list 
#                send_to_ROS = np.reshape(flat_list, np.size(flat_list)).tolist()
#
#                print(f'send_to_ROS = {send_to_ROS}')
#                input()

#                print(f'\n\nDa FULL THING send_to_ROS = {send_to_ROS}\n\n')
#                input()
                
                
            if flag_sender == 1:
#                print("Inside of the Flag_sender if")
#                input()
                tags_to_receive = sendTags(center_id,H_C_O) 
                number_of_tags = len(tags_to_receive)/3
                send_to_ROS = holder_of_old_detected_rounds
                print(f'In flag sender send to ros: {send_to_ROS}')
                
                
                if len(unknown_printables) > 0:
                    new_known_printables, unknown_printables = findDepth(frame, printables)
                    for _circle in new_known_printables:
                        _circle = np.array(_circle)
    
                        dist = getvec_thruH(H_C_O, _circle)
    #                        dist = dist.tolist()
                        # dist = transform(tag1, _circle)
                        dist[1] = dist[1] - 2*(dist[1] - tag_position[1])
                        
                        #reshape for matlab 
                        dist = [dist[1], -dist[0], dist[2]]
                        send_to_ROS.append(dist)
                
                
                if len(unknown_printables) == 0:
                    if number_of_tags <= number_of_rounds or True:
#                        input("inside second layer")
                        flag_sender = 0
                        print(tags_to_receive)
                        print(f'Tag are at: {tags_to_receive}')
                        send_to_ROS.append([9999])
                        send_to_ROS.append(tags_to_receive)
                        
                        send_to_ROS = np.array(send_to_ROS)
                        send_to_ROS = send_to_ROS.reshape(send_to_ROS.size).tolist()
                        flat_list = [item for sublist in send_to_ROS for item in sublist] #flatten the list 
                        send_to_ROS = np.reshape(flat_list, np.size(flat_list)).tolist()
        
                        print(f'send_to_ROS = {send_to_ROS}')
#                        input()
    
                        if ROS_ON==1:
        #                    input()
                            talker(send_to_ROS)
                        else:
                            print("____ROS IS OFF___")
                            input("ROS OFF")



                
#
            #%%
            # Show images
            
#            images = np.hstack((color_image, masked_color_image, depth_colormap))
            edge_img = np.dstack((edge_img,edge_img,edge_img)) #depth image is 1 channel, color is 3 channels

            images = np.hstack((color_image, edge_img, masked_color_image))
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            
            cv2.imshow('RealSense', images)
            cv2.waitKey(livestream_camera)
            

    finally:
        camera.stopStreaming()
#        cv2.destroyAllWindows() 



















#####################################################################################################################################


#OLD

#%%

def H_origin_camera(tag1):
    H_0_C= np.array([[1 ,0 ,0, tag1[0]],
                        [0, 1, 0, tag1[1]],
                        [0, 0, 1, tag1[2]],
                        [0, 0, 0,   1 ]])
    return H_0_C

#def H_origin_camera(tag1):
#    H_0_C= np.array([[1 ,0 ,0, 0],
#                        [0, 1, 0, 0],
#                        [0, 0, 1, .69],
#                        [0, 0, 0,   1 ]])
    return H_0_C

def H_newtag_camera(tag2):
    H_newT_C = np.array([[1 ,0 ,0, tag2[0]],
                        [0, 1, 0, tag2[1]],
                        [0, 0, 1, tag2[2]],
                        [0, 0, 0,   1  ]])
    return H_newT_C




def transform(tag1, tag2):
    H_newtag_origin2 = np.dot(np.linalg.inv(H_origin_camera(tag1)),(H_newtag_camera(tag2)))
    
    x_newtag2 = [H_newtag_origin2[0,3],H_newtag_origin2[1,3],H_newtag_origin2[2,3]]
    # print(f'NNNN look here: {x_newtag2}')
    #flip the coordinates to match Ethan's camera. Will need to adjust this more dynamically later
    x_newtag2 = [-x_newtag2[1], -x_newtag2[0], x_newtag2[2]]
    # print(f' Before final bump {x_newtag2}')
    #in Ethan's coordinate's
    location_Ar_R = [0.49, 0, -0.1]
    final_movement_coordinates = [location_Ar_R[0] + x_newtag2[0], location_Ar_R[1] + x_newtag2[1], location_Ar_R[2] + x_newtag2[2]]
#    final_movement_coordinates = final_movement_coordinatestolist()
    return final_movement_coordinates
