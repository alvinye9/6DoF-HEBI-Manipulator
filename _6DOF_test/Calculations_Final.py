import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import time 
from _6DOF_test.lines import line3D, curve3D
from _6DOF_test.jacobian import bigSub, AttndPos
# from lines import line3D, curve3D
# from jacobian import bigSub, AttndPos


def tonumpy(m):
    numrows = sp.shape(m)[0]
    numcols =sp.shape(m)[1]
    nummy = np.zeros([numrows, numcols])
    for i in range(0,numcols):
        for j in range(0,numrows):
            nummy[j,i] = m[j,i]

    return nummy

def calcX(thetaActual,desiredXYZ,J):
    ang1 = thetaActual[0,0]
    ang2 = thetaActual[1,0]
    ang3 = thetaActual[2,0]
    ang4 = thetaActual[3,0]
    ang5 = thetaActual[4,0]
    ang6 = thetaActual[5,0]
    realAng = sp.Matrix([[ang1],[ang2],[ang3],[ang4],[ang5],[ang6]])
    fold = AttndPos(realAng)[4]
    J = J(ang1,ang2,ang3,ang4,ang5,ang6)
    fnew = np.zeros([12,1])
    fnew[0]= 0
    fnew[1]= 0
    fnew[2]= -1
    fnew[3]= 0
    fnew[4]= 1
    fnew[5]= 0
    fnew[6]= 1
    fnew[7]= 0
    fnew[8]= 0
    fnew[9]= desiredXYZ[0]
    fnew[10]= desiredXYZ[1]
    fnew[11]= desiredXYZ[2]

    J = tonumpy(J)
    fold = tonumpy(fold)
    fnew = tonumpy(fnew)
    invJ = np.linalg.pinv(J)
    X = np.matmul(invJ,(fnew - fold))

    return X

def checkforexit(tht,cmnd,tol):
    check = abs(tht - cmnd)
    return (check[0,0]<tol and check[1,0]<tol and check[2,0]<tol and check[3,0]<tol and check[4,0]<tol and check[5,0]<tol)

def allcalcs(XYZ,start,tol,J):
    cmnd = start
    loop = True
    anglepath = np.transpose(start)
    loopcount = 0

    while loop:
        loopcount += 1
        X = calcX(cmnd,XYZ,J)
        thtnew = cmnd + X
        logger = np.transpose(thtnew)
        anglepath = np.append(anglepath,logger,axis = 0)
        if checkforexit(thtnew,cmnd,tol):
            loop = False
        
        else:
            cmnd = thtnew

    anglepath = np.delete(anglepath, (0), axis=0)
    anglepath = np.transpose(anglepath)
    return np.transpose(logger)

def calculatedXYZ(finalangles):
    f = AttndPos(finalangles)[4]
    f = np.matrix([[f[9]],[f[10]],[f[11]]])
    return f


def StraightLine(startA,end,tol,numP):
    start = np.transpose(calculatedXYZ(startA))
    linPoints = np.transpose(line3D(start,np.transpose(end),numP, False))
    size = np.shape(linPoints)[1]
    allAngs = startA
    J = bigSub()
    for i in range(size-1):
        moves = allcalcs(linPoints[:,i+1], startA,tol,J)
        allAngs = np.append(allAngs,moves,axis = 1)
        startA = moves[:,-1]
    return allAngs


def toXYZforPlot(allAng):
    XYZs = np.matrix([[0],[0],[0]])
    for i in range(np.shape(allAng)[1]):
        col = calculatedXYZ(allAng[:,i])
        XYZs = np.append(XYZs,col,1)
    XYZs = np.delete(XYZs,(0),1)    
    return XYZs
    


def armMovementPlot(angs):
    ToPlot = toXYZforPlot(angs)

    x = ToPlot[0]
    y = ToPlot[1]   
    z = ToPlot[2]

    # xf = ToPlot[0,-1]
    # yf = ToPlot[1,-1]
    # zf = ToPlot[2,-1]

    fig = plt.figure(figsize = (10,10))
    ax = plt.axes(projection='3d')
    ax.grid()
    # alphas = np.linspace(0.2, 1, np.shape(x)[1])
    colors = cm.rainbow(np.linspace(0, 1, np.shape(x)[1]))
    ax.scatter(ToPlot[0,0], ToPlot[1,0], ToPlot[2,0], c = 'r', s = 35, marker = "1")
    ax.scatter(0, 0, 0, c = 'black', s = 35)
    ax.scatter(x, y, z, c = colors, s = 25, alpha = 1)
    ax.scatter(finalXYZ[0,0], finalXYZ[1,0], finalXYZ[2,0], c = 'g', s = 35, marker = "1")
    ax.set_title('Path of Arm')
    
    # Set axes label
    ax.set_xlabel('x', labelpad=20)
    ax.set_ylabel('y', labelpad=20)
    ax.set_zlabel('z', labelpad=20)
    plt.show()
    return


if __name__ == '__main__':
    start = time.time()
    Tol = 0.05
    NumberP = 5
    offset_m6_times = -1
    offset_m3_times = -1
    offset_m2_times = -1
    offset_m5_add = 1.57
    Startangles = np.matrix([[0],[1.047*offset_m6_times],[-1.57+offset_m5_add],[-1.047*offset_m3_times],[-1.57*offset_m2_times],[0]])  #PRETEND THIS IS FEEDBACK, MAKE SURE TO ADD 90 (1.57) TO gimbal 5 BEFORE SENDING, also for gimbal 6 (DEFAULT),3, and 2 times -1 before sending
    finalXYZ = np.matrix([[0.4], [0.4], [0.3]])
    pos = StraightLine(Startangles,finalXYZ,Tol,NumberP)
    #pos = CalcToFeedback(Startangles,finalXYZ,Tol,NumberP)[1]
    #print(np.shape(pos))
    #print(pos)

    # print(np.shape(finalPos))
    # print(finalPos)
    # print(np.shape(finalPos))
    # pos = StraightLine(Startangles, finalXYZ,Tol,NumberP)
    # pos = SmoothCurve(Startangles,desiredXYZ,Tol,NumberP,h)
    # print(np.shape(pos))
    end = time.time()
    duration = end - start
    print(duration,"seconds total calc")
    # finalXYZ = calculatedXYZ(pos[:,-1])
    # diff = desiredXYZ - finalXYZ
    # print(diff)
    # armMovementPlot(pos)
    # newXYZ = calculatedXYZ(pos[:,-1])
    # diff = finalXYZ - newXYZ
    # diffx = np.float64(diff[0,0])
    # diffy = np.float64(diff[1,0])
    # diffz = np.float64(diff[2,0])
    # dist = np.sqrt(diffx*diffx+diffy*diffy+diffz*diffz)
    # print(dist)