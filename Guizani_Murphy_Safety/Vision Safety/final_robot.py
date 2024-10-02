#!/usr/bin/env python3
## Detector for ArUco Markers with Intel RealSense Camera
## Author: zptang (UMass Amherst)

import cv2
import numpy as np
import math
from scipy.spatial import ConvexHull, convex_hull_plot_2d 
import pyrealsense2 as rs
import rospy 
from std_msgs.msg import Float64MultiArray
from decimal import *
import imutils
import time
##### ROS SETUP 
#test_dict = {}
Alex_Mat_list = Float64MultiArray() 


Skander_py_list = Float64MultiArray ()

# Set up publisher 

skan_bot_flat= Float64MultiArray()
skan_bot_pub= rospy.Publisher('skander_bot_5000', Float64MultiArray, queue_size=10) 
skan_bot_flat.data = np.zeros(shape=(1,1))
is_right = np.array([1])
is_not_right = np.array([2])
counter_frame = 0
is_test=  np.array([0])

class ArUcoDetector:

    ARUCO_DICT = {
	    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

    def __init__(self, dict_to_use):
        self.dict_to_use = dict_to_use
        self.arucoDict = cv2.aruco.Dictionary_get(ArUcoDetector.ARUCO_DICT[dict_to_use])
        self.arucoParams = cv2.aruco.DetectorParameters_create()


    def detect(self, image):
        result = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        return result



    @staticmethod
    def getImageWithMarkers(input_image, detect_res):
        image = input_image.copy()
        corners, ids, rejected = detect_res
        center_id = []
        markerCorner = []
        # verify *at least* one ArUco marker was detected
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

                # draw the bounding box of the ArUCo detection
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                
                #keep the center coordinate and the id tag together
                center_id += [[cX,cY,markerID]]
                #print(center_id)
                
                # draw the ArUco marker ID on the image
                cv2.putText(image, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                #print("[INFO] ArUco marker ID: {}".format(markerID))
        
        return image, center_id, markerCorner



###########CAMERA ROTATION###################
def isRotationMatrix(R):
    Rt= np.transpose(R )
    shouldBeIdentity = np.dot (Rt,R)
    I = np.identity (3,dtype=R.dtype)
    n = np.linalg.norm(I-shouldBeIdentity)
    return  n < 1e-6


##Calculate the Rotation Matrix to Euler Angles

def rotationMatrixToEulerAngles(R) :
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
R_flip = np.zeros((3,3),dtype=np.float32)
R_flip [0,0] = 1.0
R_flip [1,1] = -1.0
R_flip [2,2] =-1.0


rvec= []
tvec = []


def get_depth (aligned_frame, x,y) : 
    aligned_frame = frame
    depth_frame = aligned_frame.get_depth_frame()
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    coord = []
    depth = depth_frame.get_distance(x, y)
    if depth > 0:
            coord = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)


    return coord

p= [[0,1,3],[2,3,5]]
q = [2,2,2]

def ismoving (list_of_list, mov_point) : 
    x = f'what theee heell'
    if( list_of_list is not None )and (mov_point) is not None :
    #Function to calculate distance 
        print (mov_point)
        for rand_point in list_of_list : 
            dist_x =  (rand_point[0] - mov_point[0] )
            dist_y =  ( rand_point[1] - mov_point [1] )
            dist_z =  (rand_point[2] - mov_point[2] ) 
            dist = np.sqrt ((dist_x**2) + (dist_y**2 ) + (dist_z**2)) 
            print (dist)
            if (dist > 1.5) : 
                return True 
            if (dist >0.5) and (dist < 1.5)  : 
                return False 

    else : 
        return False




#print (ismoving (p,q))










theta=90

def rot3(theta, var):
    
    theta = math.pi/180 * theta
    c, s = np.cos(theta), np.sin(theta)

    if var== 'x':
        rot_x= np.array ([[1,0,0],[0,c,-s],[0,s,c]])

        return rot_x
    if var== 'y':
        rot_y=np.array([[c,0,s],[0,1,0],[-s,0,c]])

        return rot_y
    if var== 'z':
        rot_z=np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        return rot_z




    
def world_to_camera (x_world): 
    
    x_world = x_world[:]
    #x_world = np.resize(1,4)
    
    R_c_0 = np.identity(3)# camera at the origin

    X_c_0 =   np.zeros(3) # Camera at the origin

    
#######################################################
############Vectors Setup##############################



    R_c_0_aug_x = np.vstack([R_c_0 , [0,0,0]])
    aug_y =np.transpose([0,0,0,1])
    aug_y.resize ((4,1))
    H_C_0 = np.hstack(([R_c_0_aug_x, aug_y]))





    #Convert to [x y z] position world frame
    #rotate base to world frame
    ## Camera rotated
    R_b_0= np.matmul(np.matmul(rot3(0,'x') ,rot3(0,'y')), rot3(180,'z'))
    R_r_b= np.matmul(np.matmul(rot3(90,'z') ,rot3(-90,'x')), rot3(0,'z'))
    
    R_b_0_aug = np.vstack([R_r_b, [0,0,0]] )
    aug_R_tran = np.transpose([2.15,0,0.125,1])
    aug_R_tran.resize((4,1))
    R_c_0_final = np.hstack(([R_b_0_aug,aug_R_tran]))
    #print (R_c_0_final)
    
    #x_0_w.data= np.resize(x_0_w, (4,1))
    #print(x_0_w)
    #print(x_world)
    #print (R_c_0_final)
    
    #print (x_camera)
    H_C_O_final = np.linalg.inv (R_c_0_final)
    #print(H_C_O_final)
    
    x_camera= np.matmul(H_C_O_final,x_world)

    x_camera.resize(1,3)
    x_camera = x_camera[0:2]
    
    return x_camera




















###############################################
################################################


#################################ROS FUNCTION
#SUB FUNCTIONS: 
def sub_callback( data) :  
    Alex_Mat_list.data = data.data 
    return Alex_Mat_list 



def python_sub_pub() : 
    
    rospy.init_node('python_sub_ops_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz 
    #Matrix Get Ready 

    
    Alex_Mat_list.data = np.zeros(shape=(1,128))
    
    rospy.Subscriber('/chatter', Float64MultiArray,sub_callback) 

    Skander_pub_pose = np.reshape(Alex_Mat_list.data,(1,128) )
    
    #print(first_point)
    point_1 = Skander_pub_pose [0][:4]
    point_2 = Skander_pub_pose[0][4:8]
    point_3 = Skander_pub_pose[0][8:12]
    point_4 = Skander_pub_pose[0][12:16]
    point_5 = Skander_pub_pose[0][16:20]
    point_6 = Skander_pub_pose[0][20:24]
    point_7 = Skander_pub_pose[0][24:28] 
    point_8 = Skander_pub_pose[0][28:32]
    point_9 = Skander_pub_pose[0][32:36]
    point_10 = Skander_pub_pose [0] [36:40]
    point_11 =  Skander_pub_pose [0]  [40:44]
    point_12 = Skander_pub_pose [0] [44:48]
    point_13 =  Skander_pub_pose [0] [48:52]
    point_14 = Skander_pub_pose  [0] [52:56]
    point_15 = Skander_pub_pose [0] [56:60]
    point_16 = Skander_pub_pose [0] [60:64]
    point_17 = Skander_pub_pose [0] [64:68]
    point_18 = Skander_pub_pose [0] [68:72]
    point_19 = Skander_pub_pose [0] [72:76]
    point_20 =Skander_pub_pose [0] [76:80]
    point_21 =Skander_pub_pose [0] [80:84] 
    point_22 = Skander_pub_pose [0] [84:88]
    point_23 =  Skander_pub_pose [0] [88:92]
    point_24 = Skander_pub_pose [0] [92:96]
    point_25 = Skander_pub_pose [0] [96:100]
    point_26 = Skander_pub_pose [0] [100:104]
    point_27 = Skander_pub_pose [0] [104:108]
    point_28 = Skander_pub_pose [0] [108:112]
    point_29 = Skander_pub_pose [0] [112:116]
    point_30 = Skander_pub_pose [0] [116:120]
    point_31 = Skander_pub_pose [0] [120:124]
    point_32 = Skander_pub_pose [0] [124:128]
    

    

    #string_to_print = f'first point {first_point}\n second_point{second_point}\n third_point{third_point} \n fourth {fourth_point} \n fifth_point {fifth_point} \n sixth_point {sixth_point}'
    #string_to_print2 = f'seventh point {seventh_point}\n eight_point {eight_point} \n nine_point {nine_point} \n ten_point {ten_point} \n 11point {eleven_point} '
    #string_to_print3  = f'12point{twelve_point} \n 13point {thirteen_point} \n 14point {forteen_point} \n 15 point {fifteen_point} \n 16 point {sixteen_point}'

    #print (string_to_print3)





    


    x_1_robot = world_to_camera(point_1)
    x_1_robot = x_1_robot.ravel()
    x_2_robot =  world_to_camera (point_2)
    x_2_robot=x_2_robot.ravel()
    x_3_robot = world_to_camera(point_3)
    x_3_robot = x_3_robot.ravel()
    x_4_robot = world_to_camera(point_4)
    x_4_robot = x_4_robot.ravel() 
    
    x_5_robot = world_to_camera(point_5)
    x_5_robot = x_5_robot.ravel()
    x_6_robot =  world_to_camera (point_6)
    x_6_robot=x_6_robot.ravel()
    x_7_robot = world_to_camera(point_7)
    x_7_robot = x_7_robot.ravel()
    x_8_robot = world_to_camera(point_8)
    x_8_robot = x_8_robot.ravel() 
    x_9_robot = world_to_camera(point_9)
    x_9_robot = x_9_robot.ravel()
    x_10_robot =  world_to_camera (point_10)
    x_10_robot=x_10_robot.ravel()
    x_11_robot = world_to_camera(point_11)
    x_11_robot = x_11_robot.ravel()
    x_12_robot = world_to_camera(point_12)
    x_12_robot = x_12_robot.ravel() 

    x_13_robot = world_to_camera(point_13)
    x_13_robot = x_13_robot.ravel() 
    x_14_robot = world_to_camera(point_14)
    x_14_robot = x_14_robot.ravel() 
    x_15_robot = world_to_camera(point_15)
    x_15_robot = x_15_robot.ravel() 
    x_16_robot = world_to_camera(point_16)
    x_16_robot = x_16_robot.ravel() 
    
    x_17_robot = world_to_camera(point_17)
    x_17_robot = x_17_robot.ravel()
    x_18_robot =  world_to_camera (point_18)
    x_18_robot=x_18_robot.ravel()
    x_19_robot = world_to_camera(point_19)
    x_19_robot = x_19_robot.ravel()
    x_20_robot = world_to_camera(point_20)
    x_20_robot = x_20_robot.ravel() 
    
    x_21_robot = world_to_camera(point_21)
    x_21_robot = x_21_robot.ravel()
    x_22_robot =  world_to_camera (point_22)
    x_22_robot=x_22_robot.ravel()
    x_23_robot = world_to_camera(point_23)
    x_23_robot = x_23_robot.ravel()
    x_24_robot = world_to_camera(point_24)
    x_24_robot = x_24_robot.ravel() 
    x_25_robot = world_to_camera(point_25)
    x_25_robot = x_25_robot.ravel()
    x_26_robot =  world_to_camera (point_26)
    x_26_robot=x_26_robot.ravel()
    x_27_robot = world_to_camera(point_27)
    x_27_robot = x_27_robot.ravel()
    x_28_robot = world_to_camera(point_28)
    x_28_robot = x_28_robot.ravel() 

    x_29_robot = world_to_camera(point_29)
    x_29_robot = x_29_robot.ravel() 
    x_30_robot = world_to_camera(point_30)
    x_30_robot = x_30_robot.ravel() 
    x_31_robot = world_to_camera(point_31)
    x_31_robot = x_31_robot.ravel() 
    x_32_robot = world_to_camera(point_32)
    x_32_robot = x_32_robot.ravel() 
    
    






    rate.sleep()
        
    
    
    return x_1_robot,x_2_robot, x_3_robot,x_4_robot,x_5_robot,x_6_robot,x_7_robot,x_8_robot, x_9_robot,x_10_robot,x_11_robot,x_12_robot,x_13_robot,x_14_robot,x_15_robot,x_16_robot, x_17_robot,x_18_robot,x_19_robot,x_20_robot,x_21_robot,x_22_robot,x_23_robot,x_24_robot,x_25_robot,x_26_robot,x_27_robot,x_28_robot,x_29_robot,x_30_robot,x_31_robot,x_32_robot
    
    

# When you get the camera try to return the array of the output of the world camera, 











if __name__ == '__main__':
    from Camera import Camera
    import numpy as np
    #x_0_camera= np.array([[0.0,0.0,0.0]])
    #x_0_robot= np.array([[0.0,0.0,0.0]])
    avg = None
    avg2 = None
    visualize = True
    dict_to_use = 'DICT_4X4_50'
    arucoDetector = ArUcoDetector(dict_to_use)
    center_id = []
################################################
#'''Here are the inputs that need tuning'''
    referenceOBJ = 6.02 / 2 # [cm]
    count_of_frames = 0 
    camera = Camera()
    camera.startStreaming()
    while True:
        start_time = time.time()
       
        

        

        
            
            
            
            
            
            
            
        
                 
        
        
        
        frame = camera.getNextFrame()
        depth_image, color_image = camera.extractImagesFromFrame(frame)

        aligned_frame = frame
        depth_frame = aligned_frame.get_depth_frame()
        color_frame = aligned_frame.get_color_frame()
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics


        # Remove unaligned part of the color_image to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        masked_color_image = np.where(depth_image_3d <= 0, grey_color, color_image)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Detect markers and draw them on the images
        result = arucoDetector.detect(color_image)
        color_image, center_ids,corners = ArUcoDetector.getImageWithMarkers(color_image, result)
        masked_color_image, __ ,__= ArUcoDetector.getImageWithMarkers(masked_color_image, result)

        depth_colormap, __,__ = ArUcoDetector.getImageWithMarkers(depth_colormap, result)
        #What I need is to do the rotation and the translation to bring the point to the camera:

        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
        color_to_depth_extrin = color_frame.profile.get_extrinsics_to(depth_frame.profile)
        #print(x_0_camera)
        
        #x_0_camera = rs.rs2_transform_point_to_point(depth_to_color_extrin, x_0_camera)

















        ###############################CAMERA ROTATION FOR MAIN#####################
        f_x= depth_intrinsics.fx
        f_y= depth_intrinsics.fy
        c_x= depth_intrinsics.ppx
        c_y = depth_intrinsics.ppy
        camera_matrix = np.array([[f_x,0,c_x],
                                 [0,f_y,c_y],
                                 [0 ,0 ,1 ]])
        dist_coeff= np.array([[depth_intrinsics.coeffs[0],depth_intrinsics.coeffs[1],depth_intrinsics.coeffs[2],depth_intrinsics.coeffs[3],depth_intrinsics.coeffs[4]]])

        
        #print (tvec)
        #print(color_pixel1)
        #print (color_pixel1)

        

        

        


        









        try :       
            ret = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeff)
            rvec = ret[0][0, 0, :]
            tvec = ret[1][0, 0, :]     
            #color_pixel2 = rs.rs2_project_point_to_pixel(color_intrin, x_0_object_3)
            #color_pixel2 = np.around(color_pixel2)
            #color_pixel2 = abs(color_pixel2.astype(int))
            #print(color_pixel1,color_pixel2)


        #cv2.rectangle(color_image, [color_pixel2[0], color_pixel2[1]], [color_pixel1[0], color_pixel1[1]], color=(0, 0, 0),
                      #thickness=-1)
            font = cv2.FONT_HERSHEY_PLAIN
            cv2.aruco.drawAxis(color_image, camera_matrix,dist_coeff,rvec,tvec,0.05)
            #print(rvec)

            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # Get the attitude in terms of euler 321 (Needs to be flipped first
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip *R_tc)


            pos_camera =np.matmul( -R_tc, np.matrix(tvec).T)

            roll_camera,pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)


            str_position = f'{pos_camera[0]}, {pos_camera[1]}, {pos_camera[2]} '
            str_position2 = f'{math.degrees(roll_marker)}, {math.degrees(pitch_camera)}, {math.degrees(yaw_camera)} '
            cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, str_position2, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        except : 
            pass 
        #Skander_py_list = Skander_pub_pose.tolist()
        
        
        


        
        #rate.sleep()


        try:
            x_1_robot,x_2_robot, x_3_robot,x_4_robot,x_5_robot,x_6_robot,x_7_robot,x_8_robot, x_9_robot,x_10_robot,x_11_robot,x_12_robot,x_13_robot,x_14_robot,x_15_robot,x_16_robot, x_17_robot,x_18_robot,x_19_robot,x_20_robot,x_21_robot,x_22_robot,x_23_robot,x_24_robot,x_25_robot,x_26_robot,x_27_robot,x_28_robot,x_29_robot,x_30_robot,x_31_robot,x_32_robot=python_sub_pub()   
            list_of_list = [x_1_robot,x_2_robot, x_3_robot,x_4_robot,x_5_robot,x_6_robot,x_7_robot,x_8_robot, x_9_robot,x_10_robot,x_11_robot,x_12_robot,x_13_robot,x_14_robot,x_15_robot,x_16_robot, x_17_robot,x_18_robot,x_19_robot,x_20_robot,x_21_robot,x_22_robot,x_23_robot,x_24_robot,x_25_robot,x_26_robot,x_27_robot,x_28_robot,x_29_robot,x_30_robot,x_31_robot,x_32_robot]
            color_pixel1 = rs.rs2_project_point_to_pixel(color_intrin, x_1_robot)
            color_pixel1 = abs(np.around(color_pixel1))
            color_pixel1 = color_pixel1.astype(int)
            
            color_pixel2 = rs.rs2_project_point_to_pixel(color_intrin, x_2_robot)
            color_pixel2 = abs(np.around(color_pixel2))
            color_pixel2 = color_pixel2.astype(int)
            

            color_pixel3 = rs.rs2_project_point_to_pixel(color_intrin, x_3_robot)
            color_pixel3 = abs(np.around(color_pixel3))
            color_pixel3 = color_pixel3.astype(int)

            color_pixel4 = rs.rs2_project_point_to_pixel(color_intrin, x_4_robot)
            color_pixel4 = abs(np.around(color_pixel4))
            color_pixel4 = color_pixel4.astype(int)

            color_pixel5 = rs.rs2_project_point_to_pixel(color_intrin, x_5_robot)
            color_pixel5 = abs(np.around(color_pixel5))
            color_pixel5 = color_pixel5.astype(int)


            color_pixel6 = rs.rs2_project_point_to_pixel(color_intrin, x_6_robot)
            color_pixel6 = abs(np.around(color_pixel6))
            color_pixel6 = color_pixel6.astype(int)

            color_pixel7 = rs.rs2_project_point_to_pixel(color_intrin, x_7_robot)
            color_pixel7 = abs(np.around(color_pixel7))
            color_pixel7 = color_pixel7.astype(int)

            color_pixel8 = rs.rs2_project_point_to_pixel(color_intrin, x_8_robot)
            color_pixel8 = abs(np.around(color_pixel8))
            color_pixel8 = color_pixel8.astype(int)


            color_pixel9 = rs.rs2_project_point_to_pixel(color_intrin, x_9_robot)
            color_pixel9 = abs(np.around(color_pixel9))
            color_pixel9 = color_pixel9.astype(int)


            color_pixel10 = rs.rs2_project_point_to_pixel(color_intrin, x_10_robot)
            color_pixel10 = abs(np.around(color_pixel10))
            color_pixel10 = color_pixel10.astype(int)


            color_pixel11 = rs.rs2_project_point_to_pixel(color_intrin, x_11_robot)
            color_pixel11 = abs(np.around(color_pixel11))
            color_pixel11 = color_pixel11.astype(int)

            color_pixel12 = rs.rs2_project_point_to_pixel(color_intrin, x_12_robot)
            color_pixel12 = abs(np.around(color_pixel12))
            color_pixel12 = color_pixel12.astype(int)

            color_pixel13 = rs.rs2_project_point_to_pixel(color_intrin, x_13_robot)
            color_pixel13 = abs(np.around(color_pixel13))
            color_pixel13 = color_pixel13.astype(int)

            color_pixel14 = rs.rs2_project_point_to_pixel(color_intrin, x_14_robot)
            color_pixel14 = abs(np.around(color_pixel14))
            color_pixel14 = color_pixel14.astype(int)

            color_pixel15 = rs.rs2_project_point_to_pixel(color_intrin, x_15_robot)
            color_pixel15 = abs(np.around(color_pixel15))
            color_pixel15 = color_pixel15.astype(int)


            color_pixel16 = rs.rs2_project_point_to_pixel(color_intrin, x_16_robot)
            color_pixel16 = abs(np.around(color_pixel16))
            color_pixel16 = color_pixel16.astype(int)

            color_pixel17 = rs.rs2_project_point_to_pixel(color_intrin, x_17_robot)
            color_pixel17 = abs(np.around(color_pixel17))
            color_pixel17 = color_pixel17.astype(int)

            color_pixel18 = rs.rs2_project_point_to_pixel(color_intrin, x_18_robot)
            color_pixel18 = abs(np.around(color_pixel18))
            color_pixel18 = color_pixel18.astype(int)

            color_pixel19 = rs.rs2_project_point_to_pixel(color_intrin, x_19_robot)
            color_pixel19 = abs(np.around(color_pixel19))
            color_pixel19 = color_pixel19.astype(int)

            color_pixel20 = rs.rs2_project_point_to_pixel(color_intrin, x_20_robot)
            color_pixel20 = abs(np.around(color_pixel20))
            color_pixel20 = color_pixel20.astype(int)

            color_pixel21 = rs.rs2_project_point_to_pixel(color_intrin, x_21_robot)
            color_pixel21 = abs(np.around(color_pixel21))
            color_pixel21 = color_pixel21.astype(int)

            color_pixel22 = rs.rs2_project_point_to_pixel(color_intrin, x_22_robot)
            color_pixel22 = abs(np.around(color_pixel22))
            color_pixel22 = color_pixel22.astype(int)

            color_pixel23 = rs.rs2_project_point_to_pixel(color_intrin, x_23_robot)
            color_pixel23 = abs(np.around(color_pixel23))
            color_pixel23 = color_pixel23.astype(int)

            color_pixel24 = rs.rs2_project_point_to_pixel(color_intrin, x_24_robot)
            color_pixel24 = abs(np.around(color_pixel24))
            color_pixel24 = color_pixel24.astype(int)

            color_pixel25 = rs.rs2_project_point_to_pixel(color_intrin, x_25_robot)
            color_pixel25= abs(np.around(color_pixel25))
            color_pixel25 = color_pixel25.astype(int)

            color_pixel26 = rs.rs2_project_point_to_pixel(color_intrin, x_26_robot)
            color_pixel26 = abs(np.around(color_pixel16))
            color_pixel26 = color_pixel16.astype(int)

            color_pixel27 = rs.rs2_project_point_to_pixel(color_intrin, x_27_robot)
            color_pixel27 = abs(np.around(color_pixel27))
            color_pixel27 = color_pixel27.astype(int)

            color_pixel28 = rs.rs2_project_point_to_pixel(color_intrin, x_28_robot)
            color_pixel28 = abs(np.around(color_pixel28))
            color_pixel28 = color_pixel28.astype(int)

            color_pixel29 = rs.rs2_project_point_to_pixel(color_intrin, x_29_robot)
            color_pixel29 = abs(np.around(color_pixel29))
            color_pixel29 = color_pixel29.astype(int)

            color_pixel30 = rs.rs2_project_point_to_pixel(color_intrin, x_30_robot)
            color_pixel30 = abs(np.around(color_pixel30))
            color_pixel30 = color_pixel30.astype(int)

            color_pixel31 = rs.rs2_project_point_to_pixel(color_intrin, x_31_robot)
            color_pixel31 = abs(np.around(color_pixel31))
            color_pixel31 = color_pixel31.astype(int)

            color_pixel32 = rs.rs2_project_point_to_pixel(color_intrin, x_32_robot)
            color_pixel32 = abs(np.around(color_pixel32))
            color_pixel32 = color_pixel32.astype(int)

            

            






        except: 
            #print('something wrong with Ros')
            pass
        try :
            cv2.circle(color_image, (color_pixel3[0],color_pixel3[1]), 10, (0, 255, 0), 2)
            cv2.circle(color_image, (color_pixel4[0],color_pixel4[1]), 10, (0, 255, 0), 2)
            #cv2.rectangle(color_image, (color_pixel2[0],color_pixel2[1]),(color_pixel1[0],color_pixel1[1]), (255,0,0), -1)
            pts = np.array([color_pixel1,color_pixel2,color_pixel3,color_pixel4,color_pixel5,color_pixel6,color_pixel7,color_pixel8],np.int32)
            pts2= np.array([color_pixel9,color_pixel10,color_pixel11,color_pixel12,color_pixel13,color_pixel14,color_pixel15,color_pixel16],np.int32)

            pts3= np.array([color_pixel17,color_pixel18,color_pixel19,color_pixel20,color_pixel21,color_pixel22,color_pixel23,color_pixel24],np.int32)
            pts4= np.array([color_pixel25,color_pixel26,color_pixel27,color_pixel28,color_pixel29,color_pixel30,color_pixel31,color_pixel32],np.int32)
            
           


            #pts = np.array ([[50,100],[60,60],[70,70],[80,80],[90,90],[100,100]])
            #print(pts)
            #points = [[383, 313], [380, 382], [538, 528], [623, 545], [845, 475], [872, 401], [668, 352], [578, 333]]
            # create and reshape array
            #pts_convex = array([color_pixel1,color_pixel2,color_pixel3,color_pixel4,color_pixel4,color_pixel8,color_pixel6,color_pixel7],np.int32)
            
            #pts= pts.reshape((-1,1,2))
            pts_int = []
            hull_float = []
            if pts is not None : 

                hull = ConvexHull(pts)
                hull_float = hull.points[hull.vertices]
                hull_int = hull_float.astype(int)
                
            if pts2 is not None : 

                hull2 = ConvexHull(pts2)
                hull_float2 = hull2.points[hull2.vertices]
                hull_int2 = hull_float2.astype(int)
                
            if pts3 is not None : 

                hull3 = ConvexHull(pts3)
                hull_float3= hull3.points[hull3.vertices]
                hull_int3= hull_float3.astype(int)
                    #print (pts_int)
            if pts4 is not None : 

                hull4 = ConvexHull(pts4)
                hull_float4= hull4.points[hull4.vertices]
                hull_int4= hull_float4.astype(int)
            

        except: 
            #print ('something wrong with the hull')
            pass   
            # Attributes

        color_image2= color_image.copy()



        try :
            isClosed = True
            color = (255, 0, 0)
            thickness = 2
            #print (pts)
            # draw closed polyline
            #cv2.polylines(color_image, [hull_int], isClosed, color, thickness)
            cv2.fillPoly(color_image2, [hull_int], color=(255, 0, 0))
            #cv2.polylines(color_image, [hull_int2], isClosed, color, thickness)
            cv2.fillPoly(color_image2, [hull_int2], color=(255, 0, 0))
            cv2.fillPoly(color_image2, [hull_int3], color=(255, 0, 0))
            cv2.fillPoly(color_image2, [hull_int4], color=(255, 0, 0))
        except :
            #print ('nooo something wrong with the fill poly thingy')
            pass
        try :
            isClosed = True
            color = (255, 0, 0)
            thickness = 2
            #print (pts)
            # draw closed polyline
            #cv2.polylines(color_image, [hull_int], isClosed, color, thickness)
            #cv2.fillPoly(color_image, [hull_int], color=(0, 0, 0))
            #cv2.polylines(color_image, [hull_int2], isClosed, color, thickness)
            #cv2.fillPoly(color_image, [hull_int2], color=(0, 0, 0))
            #cv2.fillPoly(color_image, [hull_int3], color=(0, 0, 0))
            #cv2.fillPoly(color_image, [hull_int4], color=(0, 0, 0))
        except :
            #print ('nooo something wrong with the fill poly thingy')
            pass
            

            #cv2.polylines(color_image, [points2], isClosed, color, thickness)
            #cv2.polylines (color_image, pts,False, (0,255,0), -1)
                
            #cv2.polylines(color_image, [pts], True,(0,255,0), 2)
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        
            
        if avg is None:
            #avg = gray.copy().astype("float")
            avg = gray
            continue
        

        
        try :
            
            #cv2.accumulateWeighted(gray, avg, 0.5)
            #frameDelta = cv2.subtract(gray, avg, dtype=cv2.CV_64F)
            frameDelta = cv2.absdiff(gray, avg)
            #if counter_frame%2==0 : 
            avg = gray

            
            counter_frame = counter_frame + 1
            kernel = np.ones((5, 5))
            cv2.fillPoly(frameDelta, [hull_int], color=(0, 0, 0))
            
            cv2.fillPoly(frameDelta, [hull_int2], color=(0, 0, 0))
            cv2.fillPoly(frameDelta, [hull_int3], color=(0, 0, 0))
            cv2.fillPoly(frameDelta,[hull_int4], color=(0, 0, 0))
            mask2= np.zeros(frameDelta.shape[:2],np.uint8)
            mask2= cv2.bitwise_and (frameDelta,frameDelta,mask=mask2)
            
            dilate = cv2.dilate(frameDelta, kernel, iterations=1)
            
            thresh = cv2.threshold(frameDelta, 30, 255, cv2.THRESH_BINARY)[1]
            cv2.fillPoly(thresh, [hull_int], color=(0, 0, 0))
            
            cv2.fillPoly(thresh, [hull_int2], color=(0, 0, 0))
            cv2.fillPoly(thresh, [hull_int3], color=(0, 0, 0))
            cv2.fillPoly(thresh,[hull_int4], color=(0, 0, 0))
            mask3 = np.zeros(thresh.shape[:2],np.uint8)
            mask3= cv2.bitwise_and (thresh,thresh,mask=mask3)
            #new_thresh = np.uint8(dilate)

            contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            #contours = cv2.findContours(new_thresh.copy(), cv2.RETR_EXTERNAL,
                                        #cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(contours)
            

        except : 
            print ('there is an error in object detection')
        try : 
            skan_bot_flat.data = is_test
            skan_bot_pub.publish (skan_bot_flat)
            for cnt in cnts:
                if len(cnt) > 0:
                        # print("print to ROS")
                    
                    area = cv2.contourArea(cnt)

                if area > 200:
                    
                    
                    
                    cv2.drawContours(color_image, [cnt], -1, (0, 255, 0), 2)
                    x, y, w, h = cv2.boundingRect(cnt)
                    coord = get_depth(frame,x,y)
                        #print (coord)
                    cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                        
                    
                    if ismoving (list_of_list,coord) :
                        skan_bot_flat.data = is_right
                        skan_bot_pub.publish (skan_bot_flat)
                        print ('stop the robot')

                        
                    if ismoving (list_of_list,coord) == False:
                        #print ('person is coming stop SKander those are civilians ' )
                        skan_bot_flat.data = is_not_right
                        skan_bot_pub.publish (skan_bot_flat)
                        #print (f'time in seconds :{time.time()-start_time}')
                        #sum_of_people +=1
                        print ('keep moving')
                        #time_diff=time.time()-start_time
                        #test_dict [sum_of_people] = time_diff
                #print (test_dict)
                    
            #with open ('test_dataa.txt','a') as in_handle :
                            
                
              #  if (k is not None) and (v is not None) : 
                #for v,k in test_dict.items() : 
                #    count_of_frames +=1
                 #   print (count_of_frames)
                  #  in_handle.write(f'number of ob dected: {v} time to detect {k} \n ')

            #print (test_dict)
        except : 
            print ('there is an error in publisher')
         
        
                        
                    
        
        # print(center_ids)
        # Show images
        images = np.hstack((color_image, color_image2,depth_colormap))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        
        #cv2.imshow('thresh', thresh)
        #cv2.imshow('dilate', dilate)
        #cv2.imshow('new_thresh', new_thresh)
        #cv2.imshow('frameDelta', frameDelta)
        
        #cv2.imshow('avg',avg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



cv2.destroyAllWindows()
