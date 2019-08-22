    
    fig = plt.figure()
    plt.imshow(cv_rgb1)
    plt.show()
    plt.draw()



    # used to  compare obtained pc from depth and rgb img with the ros one


    topicPC ="/depth_registered/points"   
    pcmsg = rospy.wait_for_message(cameraNames[0] + topicPC, PointCloud2)

    truecloud = converter.PC2toOpen3DPC(pcmsg)

    open3d.draw_geometries([truecloud,pcd])
    

    import open3d
import math
import numpy as np

#console.log("mat rot")



#def makeRx():
theta=15
#math.cos(theta)

n_referentials = 3


def genRotMat(angle):

    Rx = [[1,0,0],[0,math.cos(angle[0]),-math.sin(angle[0])],[0,math.sin(angle[0]),math.cos(angle[0])]]

    Ry = [[math.cos(angle[1]),0,math.sin(angle[1])],[0,1,0],[-math.sin(angle[1]),0,math.cos(angle[1])]]


    Rz = [[math.cos(angle[2]),-math.sin(angle[2]),0],[math.sin(angle[2]),math.cos(angle[2]),0],[0,0,1]] 

    aux = np.dot(Rx,Ry)


    return np.dot(aux,Rz)


#print(np.dot(rot.T,rot))

#print(np.linalg.det(rot))

rot=[]
#generate matrices
for i in range(0,n_referentials):


    rot.append(genRotMat(np.random.rand(3,1)))


print(len(rot))

rotrel=[[1,2,3],[1,2,3],[0,0,0]]

print(rotrel)

#generate measurements
for i in range(0,n_referentials):
    for j in range(0,n_referentials):

        rotrel[i][j]=np.dot(rot[i].T,rot[j])

        print("===========")
        print((i,j))
        print(rotrel[i][j])


#add noise
std = 0.0001


for i in range(0,n_referentials):
    for j in range(0,n_referentials):

        noise  =  std * np.random.randn(3,3)

        rotrel[i][j]= rotrel[i][j] + noise

        print("===========")
        print((i,j))
        print(rotrel[i][j])



solv = np.zeros([n_referentials*3,n_referentials*3])



def changeMat(solv,change,x,y):
    print(x*3+0,x*3+3,y*3+0,y*3+3)
    solv[x*3+0:x*3+3,y*3+0:y*3+3] = change
    return solv

solv = changeMat(solv,np.eye(3),0,0)
solv = changeMat(solv,rotrel[1][2],1,2)
solv = changeMat(solv,rotrel[2][1],2,1)


def LaGrange(x,A,lamb):
        return np.linalg.multi_dot([x.T,solv.T,solv,x]) + lamb *( np.linalg.multi_dot([x.T,x])-1)


print("===========")
print(solv)

print("===========")
w,v = np.linalg.eig(np.dot(solv,solv.T))

print(w,v)

minval = LaGrange(v[:,0],solv,w[0])
minindex=0

for i in range(0,len(w)):

        
        val = LaGrange(v[:,i],solv,w[i])

        if (val<minval):
                minindex = w[i]
                minval = val
        


print(v.shape)


#Translation tests


    R=[]
    t=[]

    R.append(Rtmat.genRotMat([0,0,0]))
    t.append(np.array([0,0,0]))

    R.append(Rtmat.genRotMat([0,0,0]))
    R.append(Rtmat.genRotMat([0,0,-90]))

    t.append(np.array([0,-20,0]))
    t.append(np.array([0,20,0]))

    rr = (np.dot(R[1],R[2].T))

    ola = np.dot(rr,np.array([-20,0,0]))
    print(ola + np.array([0,40,0]))

    ViewRefs(R,t)

    print("2tw",np.dot(-R[2],t[2]))



    def SampleGeneratorMin(rot,noise = 1e-10):

    obs=[]

    for i in range(0,len(rot)-1):
        #SHOULDNT IT BE rot[i+1],rot[i+1].T)
        obs.append({"from":i,"to":i+1,"rot":np.dot(np.dot(rot[i+1],rot[i].T),Rtmat.genRotMat(np.squeeze([np.random.rand(3,1)*noise])))})   
        #print(i,i+1)

    obs.append({"from":len(rot)-1,"to":0,"rot":np.dot(rot[0],rot[len(rot)-1].T)})  #delete this line after
    return obs


    ### TRANSLATION TESTS

    import pprint
import numpy as np
import Rtmat
import pickler as pickle
from synth import *

R1 = np.eye(3)
R2 = np.array([[0,1,0],[-1,0,0],[0,0,1]])


R=[]
R.append(R1)
R.append(R2)



ola0 = np.dot(R[0].T,np.array([10,10,0]))

ola = ola0 + [0,-20,0]

print(ola)


quit()


t=[]
t.append([0,0,0])
t.append([10,10,0])

ViewRefs(R,t)

r  = np.dot(R[0],R[1].T)
t0w=np.array([0,20,0])

ola = np.dot(r,t0w)

tw1 = ola + np.array([10,-10,0])

print(tw1)

### MORE TRANSLATIONS TESTS

import open3d
import math
import numpy as np
import pickler as pickle
import pprint
import random
import Rtmat
import phase2_sparkle as phase2
import synth



def main():
    print("helloworld")

    R=[]
    t=[]

    
    t0=np.array([0,0,0]))

    R01 =Rtmat.genRotMat([0,0,0])
    t01 = np.array([10,0,0])




    synth.ViewRefs(R,t)

    rotRel2 =  (np.dot(R[3],R[2].T)) 
    print("after local")
    print(rotRel2)


    #global transformation
    R.append(np.dot(Rtmat.genRotMat([0,0,-90]),R[0]))
    R.append(np.dot(Rtmat.genRotMat([0,0,-90]),R[1]))

    rotRel3 =  (np.dot(R[5],R[4].T)) 

    print("after global")
    print(rotRel3)
    



if __name__ == '__main__':
    main()


## GETTING cAMERA CALIB CAM CALIB INTRINSIC PARAMS

    K,D = roscv.camCalib(cameraName)

    pickle.In("CameraInfo","K",K)
    pickle.In("CameraInfo","D",D) 



    ########################################## PERMUTATOR

    import matmanip as mmnip
import numpy as np
import pickler as pickle
import visu


g = pickle.Out("pickles/ArucoRot.pickle")

#print("LOCALLL",g['Rglobal'])

#print("GLOBALL",g['Rlocal'])

permuter = [[0,-1,0],[1,0,0],[0,0,-1]]

finalR=  mmnip.PermuteCols(g['Rlocal'],permuter)

pickle.ShowData()

pickle.In("ArucoRot","RlocalPermutated",finalR,putDate=False)

pickle.ShowData()

visu.ViewRefs(finalR)

##############################333333333333333333

g = pickle.Out("pickles/ArucoRot.pickle")

#print("LOCALLL",g['Rglobal'])

#print("GLOBALL",g['Rlocal'])

permuter = [[0,-1,0],[-1,0,0],[0,0,-1]]

finalR=  mmnip.PermuteCols(g['Rlocal'],permuter)


pickle.In("ArucoRot","RlocalPermutated",finalR,putDate=False)


visu.ViewRefs(finalR)

####################################




def ObsMatcher(camsObs):

    for i in range(0,len(camsObs)):
        for j in range(0,i):
###################################### UNDERSTANDING RIGID BODY TRANSFORMATION

import open3d
import matmanip as mnip
import visu
import numpy as np
import pickler2 as pickle


Rww=mnip.genRotMat([0,0,0])
tww=[[0],[0],[0]]

#visu.ViewRefs([Rww,Rw1],)

Rw1=mnip.genRotMat([0,-90,0])
print(Rw1)
tw1=[[10],[0],[-10]]

tw = [[10],[0],[-10]]

totransform =  [[20],[0],[-10]]

a = np.dot(Rw1,totransform)+tw1
print(a)


#visu.ViewRefs([Rww,Rw1,mnip.genRotMat([0,-90,0])])

print(mnip.InvertT(Rw1,tw1))


##

import json
import numpy as np
import FileIO
import visu
import algos
import matmanip as mmnip
import open3d

a = np.array([[1],[2],[3]])

b = np.array([4])
print(b.shape)
c = np.array([[1]])

print(np.squeeze(a))

print(np.squeeze(b))

lol =(np.squeeze(c))

print(lol)
print(type(lol))

lol = lol.tolist()
if(type(lol)==int):
    print("wow")

print(len(lol))

