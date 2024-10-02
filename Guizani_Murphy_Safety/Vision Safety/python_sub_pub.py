#!/usr/bin/env python

import rospy #allow ROS interface
from std_msgs.msg import Float64MultiArray #ROS data format
import numpy as np #matrix operations


# SET UP SUBSCRIBERS
Aflat = Float64MultiArray() #from Python
Bflat = Float64MultiArray() #from Matlab


# SET UP PUBLISHERS
Cflat = Float64MultiArray() #set 
pubC = rospy.Publisher('Cflat_py', Float64MultiArray, queue_size=10)

Dflat = Float64MultiArray()
pubD = rospy.Publisher('Dflat_py', Float64MultiArray, queue_size=10)


# SUBSCRIBER FUNCTIONS
def Aflat_callback(data):
    Aflat.data = data.data
    return Aflat

def Bflat_callback(data):
    Bflat.data = data.data
    return Bflat


def python_sub_ops_pub():

    # set up ROS node
    rospy.init_node('python_sub_ops_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # preallocate data size
    Aflat.data = np.zeros(shape=(1,16))
    Bflat.data = np.zeros(shape=(1,16))
    
    while not rospy.is_shutdown():

        # GET SUBSCRIBER DATA and CONVERT TO MATRIX FORM
        rospy.Subscriber('/Aflat_py', Float64MultiArray, Aflat_callback)
        rospy.Subscriber('/Bflat_mat', Float64MultiArray, Bflat_callback) 

        A = np.reshape(Aflat.data,(4,4)) #matrix form
        B = np.reshape(Bflat.data,(4,4))        



        # DO SOME MATRIX OPERATIONS
        C = A+B

        D = np.matmul(A,B)
        # D = np.multiply(A,B)


        # PREPROCESS AND PUBLISH 
        C1         = np.reshape(C,16) #reshape but still as numpy array
        Cflat.data = C1.tolist()      #convert to list and store in Float64MultiArray

        Dflat.data = np.reshape(D,16).tolist() #combine above 2 C steps for D

        pubC.publish(Cflat) #publish 
        pubD.publish(Dflat)                 


        # SLOW THIS DOWN AS REQUIRED
        rate.sleep()


if __name__ == '__main__':
    try:
        python_sub_ops_pub()
    except rospy.ROSInterruptException:
        pass

#SKander modify 