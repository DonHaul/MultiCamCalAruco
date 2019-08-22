"""
synth.py

This module contains functions to:
-Generate Synthetic Observations in a set of Referentials
-Generate Synthetic scenes and models (Sets of Referantials)
-Generate Synthetic Observations between a Set of Referantials(Cameras) that are observing another set (Aruco)
"""

import numpy as np
import random
import matmanip as mmnip

import visu

def SampleGenerator(R,t,samples=1000,noise = 0.00001,noiset=0.0001):
    ''' Generates observations(Rotations and translations) between a set of Referentials

    Args:
        R: List of rotations of the referentials in world coordinates (Riw)
        t: List of translations of the referentials in world coordinates (tiw)
        samples: Approximated number of samples to generate
        noise: Noise scale in degrees that will be added to the rotations observed
        noiset: Noise scale in degrees that will be added to the tranlations observed

    Returns:
        obsR: Dictionary with the Rotation Observations. Each Observations Contains
            -from: Where the rotation comes from
            -to: Where the rotation is going to
            -R: The rotation itself
        obst: Dictionary with the Translation Observations. Each Observations Contains
            -from: Where the rotation comes from
            -to: Where the rotation is going to
            -t: The translation itself
    '''

    #initializes array that tells if this rotation r[i] has atleast one observation
    r = np.zeros([len(R),1])  

    #for a while (this loop only occurs 1 time if we are lucky)
    while True:

        obsR = []
        obst = []

        #generates samples
        for i in range(0,samples):

            #for each observation        

            #pick 2 different ids
            r1 =  random.randint(0, len(R)-1)
            r2 = r1
            while r2==r1:
                r2 = random.randint(0, len(R)-1)
            


            t1w = t[r1] #translation of referantial 1 in world coordinates
            t2w = t[r2] #translation of referantial 2 in world coordinates
                        
            t12 =np.dot(R[r2].T, t1w - t2w) #translation of referantial 1 in referantial 2's coordinates
            
            #print("from:"+str(r2)+"to:"+str(r1))
            #print(np.dot(R[r1],R[r2].T))
            #raw_input()
            
            #generate a R observation w/ noise
            obsR.append({"from":r2,"to":r1,"R":np.dot(mmnip.genRandRotMatrix(noise),np.dot(R[r1].T,R[r2]))})
            
            #generate a t observation  w/ noise
            obst.append({"from":r1,"to":r2,"t":t12+np.random.rand(3)*noiset}) #*noiset
            
            #sets this cameras as having observations
            r[r1]=1
            r[r2]=1


        #there is at least one observation per marker then, exit, else generate more samples
        if sum(r)==len(R):
            break

    return obsR,obst


def MultiCamSampleGeneratorMoving(Rcam,tcam,R,t,):
    pass


def MultiCamSampleGeneratorFixed(Rcam,tcam,R,t,nObs=5,noise = 0.0,noiset = 0.0):
    '''
    Simulates one single time instance and generates Synthetic Observations
    between a Set of Referantials(Cameras) that are observing another set (Aruco)
    
    Every rotation received comes from world coordinates  w -> i
    Every translation received is to world coordinates  i -> w

    Args:
        Rcam: List of rotations of every camera
        tcam: List of translations  of every camera
        R: List of rotations of the aruco model that both cameras see
        t: List of translations  of the aruco model that both cameras see
        nObs: Number of Markers that each camera sees,
        noise: Noise added to the rotations
        noiset: Noise added to the translations
    '''

    #number of observations of a camera in a certain frame, prevent it from being bigger than all markers
    if(nObs>len(t)):
        print("Warning: Number of observations requested higher than total markers")
        nObs=len(t)-1


    camsObs = []        #list of all observations
    
    #generate samples for each camera
    for i in range(0,len(Rcam)):
        
        #pick random aruco markers, #noBs of them
        rnds = random.sample(range(0, len(R)), nObs)

        obs=[]  #list of Observations
        
        #For each observed marker
        for r in rnds:

            tcr =np.dot(Rcam[i].T, t[r] - tcam[i]) # t from observation r to camera i  

            #generate the samples  'from' Camera i 'to' sample i
            #'ObsId' = 'to'                 #'camId = to ObsId = 'from'

            #same as R w->r *(R w->cam transposed)

            o ={ "obsId":r,"R": np.dot(mmnip.genRandRotMatrix(noise), np.dot(Rcam[i].T,R[r])),'t':tcr}


            obs.append(o)
            
        camsObs.append(obs)



    return camsObs

def Scenev1():
    '''
    Generate a scene with 3 cameras
    '''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,180,0]))
    R.append(mmnip.genRotMat([0,-90,0]))
    R.append(mmnip.genRotMat([0,0,0]))

    t.append(np.array([0,0,0]))
    t.append(np.array([50,0,-50]))
    t.append(np.array([0,0,-100]))

    return R,t

def Scenev2():
    '''
    Generate a scene with 3 cameras
    '''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,180,0]))
    #R.append(mmnip.genRotMat([0,-90,0]))
    
    t.append(np.array([0,0,-100]))
    t.append(np.array([0,0,0]))
    #t.append(np.array([50,0,-50]))
    

    return R,t


def Scenev3():
    '''
    Generate a scene with 3 cameras
    '''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,45,0]))
    #R.append(mmnip.genRotMat([0,-90,0]))
    
    t.append(np.array([0,0,-100]))
    t.append(np.array([0,0,0]))
    #t.append(np.array([50,0,-50]))
    

    return R,t


def FakeArucoWTF():
    '''Generate aruco model with 4 markers'''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,90,0]))
    R.append(mmnip.genRotMat([0,180,0]))
    #R.append(mmnip.genRotMat([0,270,0]))
    
    t.append(np.array([0,0,10]))
    t.append(np.array([10,0,0]))
    t.append(np.array([0,0,-10]))
    #t.append(np.array([-10,0,0]))

    return R,t

def FakeArucoRotated():
    '''Generate aruco model with 4 markers'''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([90,180,0]))
    R.append(mmnip.genRotMat([90,90,0]))
    R.append(mmnip.genRotMat([90,0,0]))
    R.append(mmnip.genRotMat([90,-90,0]))
    
    t.append(np.array([0,10,0]))
    t.append(np.array([10,0,0]))
    t.append(np.array([0,-10,0]))
    t.append(np.array([-10,0,0]))

    return R,t


def FakeAruco():
    '''Generate aruco model with 4 markers'''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,90,0]))
    R.append(mmnip.genRotMat([0,180,0]))
    R.append(mmnip.genRotMat([0,270,0]))
    
    t.append(np.array([0,0,10]))
    t.append(np.array([10,0,0]))
    t.append(np.array([0,0,-10]))
    t.append(np.array([-10,0,0]))

    return R,t

def FakeAruco2Markers():
    '''Generate aruco model with 2 markers'''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,180,0]))
    
    t.append(np.array([0,0,10]))
    t.append(np.array([0,0,-10]))

    return R,t

   
def FakeArucoReal9():
    '''Generate aruco model with 12 markers'''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,0,0]))

    R.append(mmnip.genRotMat([0,90,0]))
    R.append(mmnip.genRotMat([0,90,0]))
    R.append(mmnip.genRotMat([0,90,0]))
    
    R.append(mmnip.genRotMat([0,180,0]))
    R.append(mmnip.genRotMat([0,180,0]))
    R.append(mmnip.genRotMat([0,180,0]))


    
    t.append(np.array([0,0,10]))
    t.append(np.array([0,30,10]))
    t.append(np.array([0,50,10]))

    t.append(np.array([10,0,0]))
    t.append(np.array([10,30,0]))
    t.append(np.array([10,50,0]))

    t.append(np.array([0,0,-10]))
    t.append(np.array([0,30,-10]))
    t.append(np.array([0,50,-10]))


    return R,t

def TestScene51():

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,-90,0]))
    R.append(mmnip.genRotMat([0,0,45]))
    R.append(mmnip.genRotMat([0,45,0]))
    R.append(mmnip.genRotMat([90,45,0]))

  
    
    t.append(np.array([20,20,20]))
    t.append(np.array([20,40,20]))
    t.append(np.array([20,20,40]))
    t.append(np.array([0,40,20]))

    return R,t

def TestScene31():

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,185,0]))
    R.append(mmnip.genRotMat([0,262,0]))
    R.append(mmnip.genRotMat([0,0,10]))
    R.append(mmnip.genRotMat([-7,90,0]))

  
    
    t.append(np.array([0,0,2.1]))
    t.append(np.array([1.9,0,0]))
    t.append(np.array([0.1,0,-2]))
    t.append(np.array([-2,-0.2,0]))

    return R,t


def TiltedCams():


    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,90,0]))
    R.append(mmnip.genRotMat([-45,0,0]))
    R.append(mmnip.genRotMat([0,-90,0]))

  
    
    t.append(np.array([0,0,0]))
    t.append(np.array([0,0,10]))
    t.append(np.array([0,0,20]))

    return R,t
   
def FakeArucoReal():
    '''Generate aruco model with 12 markers'''

    R=[]
    t=[]

    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,0,0]))
    R.append(mmnip.genRotMat([0,0,0]))

    R.append(mmnip.genRotMat([0,90,0]))
    R.append(mmnip.genRotMat([0,90,0]))
    R.append(mmnip.genRotMat([0,90,0]))
    
    R.append(mmnip.genRotMat([0,180,0]))
    R.append(mmnip.genRotMat([0,180,0]))
    R.append(mmnip.genRotMat([0,180,0]))

    R.append(mmnip.genRotMat([0,270,0]))
    R.append(mmnip.genRotMat([0,270,0]))
    R.append(mmnip.genRotMat([0,270,0]))
    
    t.append(np.array([0,0,10]))
    t.append(np.array([0,30,10]))
    t.append(np.array([0,50,10]))

    t.append(np.array([10,0,0]))
    t.append(np.array([10,30,0]))
    t.append(np.array([10,50,0]))

    t.append(np.array([0,0,-10]))
    t.append(np.array([0,30,-10]))
    t.append(np.array([0,50,-10]))

    t.append(np.array([-10,0,0]))
    t.append(np.array([-10,30,0]))
    t.append(np.array([-10,50,0]))

    return R,t






