import hebi
import math
from time import sleep, time
import numpy as np
import sympy as sp
import time 
from .lines import line3D
from .testAlvin import StraightLine #, CalcToFeedback



x=0
def Home(group):
    cmd = hebi.GroupCommand(7)
    cmd.position = np.matrix([0,1.047,-1.047,-1.57,0,-1.047,-1.57,0])
    group.send_command(cmd)
    x=1


def getTrajectory(startAngles, endXYZ, waypoints, speed, tol):  
    calcPos = StraightLine(startAngles,endXYZ, tol, waypoints)
    # print("1")
    # print(calcPos)
    lastAngles = calcPos[:,-1]

  #convert calculations to usable feedback
    feedbackPos = np.copy(calcPos) #MAKE SURE TO USE COPY AND NOT THE EQUAL SIGN OR IT WILL MUTATE THE ORIGINAL ARRAY
    feedbackPos[1,:] = feedbackPos[1,:]*(-1)  #row 2
    feedbackPos[2,:] = feedbackPos[2,:]-(0.5*np.pi) #row 3
    feedbackPos[3,:] = feedbackPos[3,:]*(-1)
    feedbackPos[4,:] = feedbackPos[4,:]*(-1)
    newRow = -1*feedbackPos[1,:]
    feedbackPos = np.insert(feedbackPos,2,newRow,axis = 0)


    num_joints = np.shape(feedbackPos)[0]  
    num_waypoints = np.shape(feedbackPos)[1]
    # print("Number of Joints:")
    # print(num_joints)
    # print("Number of Waypoints: ")
    # print(num_waypoints)
    vel = np.empty((num_joints,num_waypoints)) #defines velocity matrix
    acc = np.empty((num_joints,num_waypoints)) #defines velocity matrix
    vel[:,0] = acc[:,0] = 0.0 #make start velocity/acceleration 0
    vel[:,-1] = acc[:,-1] = 0.0  #make end velocity/acceleration 0
    vel[:,1:-1] = acc[:,1:-1] = np.nan
    time = np.linspace(0.0, speed*num_waypoints, num_waypoints) 
    trajectory = hebi.trajectory.create_trajectory(time, feedbackPos, vel, acc)
    # print("First Set of Gimbal Angles: ")
    # print(trajectory.get_state(0))
    
    #return trajectory object, number of waypoints/joints, and last set of angles in the (Originally calculated) big waypoints matrix
    # print("Return Last Angles: ")
    # print(lastAngles)
    return trajectory,num_joints,num_waypoints, lastAngles

def runTrajectory(trajectory,num_joints,num_waypoints,isFinal, group):
    cmd = hebi.GroupCommand(num_joints)
    period = 0.01 #affects execution rate
    duration = trajectory.duration
    
    pos_cmd = np.array(num_joints, dtype=np.float64)
    vel_cmd = np.array(num_joints, dtype=np.float64)

    posfinal_cmd = 0 #matrices to hold final position
    velfinal_cmd = 0
    accfinal_cmd = 0
    t = 0

    while (t < duration):
        pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
        posfinal_cmd, velfinal_cmd, accfinal_cmd = trajectory.get_state(t)
        print("=======")
        print(num_waypoints)
        print(t)
        print(pos_cmd)
        print(vel_cmd)
        print(acc_cmd)
        print("=======")
        cmd.position = pos_cmd
        cmd.velocity = vel_cmd
        group.send_command(cmd)
        t = t + period
        sleep(period) #helps it not execute too fast
    
    loop = isFinal
    while(loop):
        print("=======")
        print("HOLDING FINAL POSITION")
        print(posfinal_cmd)
        print(velfinal_cmd)
        print(accfinal_cmd)
        print("=======")
        cmd.position = posfinal_cmd
        cmd.velocity = velfinal_cmd
        group.send_command(cmd)

    return True

def main():
    start = time.time()
    # #====================== SETUP HEBI AND FEEDBACK =============================
    lookup = hebi.Lookup()
    dir(hebi)
    # Wait 2 seconds for the module list to populate
    sleep(2.0)

    #Code for 6DOF arm
    family_name = "Arm"
    mod1_name = "m1"
    mod2_name = "m2"
    mod3_name = "m3"
    mod5_name = "m5"
    mod6_name = "m6"
    mod7_name = "m7" #send negative value of 6
    mod8_name = "m8"

    group = lookup.get_group_from_names([family_name], [mod8_name, mod6_name, mod7_name, mod5_name, mod3_name, mod2_name, mod1_name])

    if group is None:
        print('Group not found: Did you forget to set the module family and name above?')
        exit(1)

    print('Found group on network with {0} modules.'.format(group.size))
    group.feedback_frequency = 50.0

    group_feedback = hebi.GroupFeedback(group.size)

    #======= Get Initial Theta Position Feedback =======
    group.send_feedback_request()
    group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
    angles = group_feedback.position
    #print(angles)
    #===========================================================


    #============= USING FEEDBACK AND FUNCTIONS TO DO CALCS =========

    tolerance = 0.01 #how close do you want calculations to converge

    offset_m6_times = -1
    offset_m3_times = -1
    offset_m2_times = -1
    offset_m5_add = 1.57

    #Remember to offset the feedback -> calc
    Startangles = np.matrix([[angles[0]],[angles[1]*offset_m6_times],[angles[3]+offset_m5_add],[angles[4]*offset_m3_times],[angles[5]*offset_m2_times],[angles[6]]])

    Point1 = np.matrix([[0.565], [-0.184], [0.331]]) 
    Point2 = np.matrix([[0.4], [0.4], [0.4]]) 
    Point3 = np.matrix([[0.4], [0.4], [0.1]]) 
    Point4 = np.matrix([[0.4], [0.4], [0.4]]) 
    Point5 = np.matrix([[0.565], [-0.184], [0.331]]) 
    Point6 = np.matrix([[0.09], [-0.54], [0.4]]) 
    Point7 = np.matrix([[0.09], [-0.54], [0.1]])
    Point8 = np.matrix([[0.09], [-0.54], [0.4]]) 
    Point9 = np.matrix([[0.565], [-0.184], [0.331]])  




    desired_waypoints = 5  #number of waypoints between start point and next point (not necessary to update every single time)
    speed = 0.4 #seconds per waypoint (also not necessary to update every single time)

    trajCalcValues = getTrajectory(Startangles, Point1, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory = trajCalcValues[0]
    num_joints = trajCalcValues[1] #since numjoints is always the same, no need to redeclare every single time
    num_waypoints = trajCalcValues[2]

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point2, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory2 = trajCalcValues[0]
    num_waypoints2 = trajCalcValues[2] #should be 1 more than specified

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point3, desired_waypoints, speed, tolerance)
    print(time.time()-start) 
    trajectory3 = trajCalcValues[0]
    num_waypoints3 = trajCalcValues[2] 

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point4, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory4 = trajCalcValues[0]
    num_waypoints4 = trajCalcValues[2] 

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point5, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory5 = trajCalcValues[0]
    num_waypoints5 = trajCalcValues[2] 

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point6, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory6 = trajCalcValues[0]
    num_waypoints6 = trajCalcValues[2] 

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point7, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory7 = trajCalcValues[0]

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point8, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory8 = trajCalcValues[0]

    Startangles = trajCalcValues[3]
    trajCalcValues = getTrajectory(Startangles, Point9, desired_waypoints, speed, tolerance) 
    print(time.time()-start)
    trajectory9 = trajCalcValues[0]


    isFinalTrajectory = False
    runTrajectory(trajectory, num_joints, num_waypoints, isFinalTrajectory, group)
    runTrajectory(trajectory2, num_joints, num_waypoints, isFinalTrajectory, group)
    runTrajectory(trajectory3, num_joints, num_waypoints, isFinalTrajectory, group)
    runTrajectory(trajectory4, num_joints, num_waypoints, isFinalTrajectory, group)
    runTrajectory(trajectory5, num_joints, num_waypoints, isFinalTrajectory, group)
    runTrajectory(trajectory6, num_joints, num_waypoints, isFinalTrajectory, group)
    runTrajectory(trajectory7, num_joints, num_waypoints, isFinalTrajectory, group)
    runTrajectory(trajectory8, num_joints, num_waypoints, isFinalTrajectory, group)
    isFinalTrajectory = True
    runTrajectory(trajectory9, num_joints, num_waypoints, isFinalTrajectory, group)

if __name__ == '__main__':
    main()