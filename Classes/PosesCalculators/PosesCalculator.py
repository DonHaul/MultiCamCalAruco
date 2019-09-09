"""
CamPoseGetter.py

This module contains the a class receives all the images and observations of the cameras, and calculates stuff with it
"""

import numpy as np
import cv2

from libs import *


class PosesCalculator(object):
    def __init__(self,data):

        self.estimating='R'
        
        self.N_objects = data['N_objects']



        #A.T A initialized
        self.ATAR = np.zeros((self.N_objects*3,self.N_objects*3))

        #A.T A initialized
        self.ATAt = np.zeros((self.N_objects*3,self.N_objects*3))

        #A.T b initialized
        self.ATb = np.zeros((self.N_objects*3,1))

        if 'record' in data:
            self.recordRT=data['record']
        else:
            self.recordRT=False


        self.recordedRs=[]
        self.recordedTs=[]

        self.R=None
        self.t=None

    def Show(self):
        print(self.ATAR)

    def Clear(self):
        #A.T A initialized
        self.ATAR = np.zeros((self.N_objects*3,self.N_objects*3))

        #A.T A initialized
        self.ATAt = np.zeros((self.N_objects*3,self.N_objects*3))

        #A.T b initialized
        self.ATb = np.zeros((self.N_objects*3,1))
        
    def AddObservations(self,obsR,obsT):

        #print("ADDING")
        if self.recordRT:
            #print("RECORDING")
            for oT in obsT:
                self.recordedTs.append(oT['t'])

            for oR in obsR:
                
                self.recordedRs.append(oR['R'])

                #print("len is", len(self.recordedRs))


        #calculates rotations
        if self.estimating =='R':
            
            #only if there are observations it makes the A matrix
            if len(obsR)>0:

                A =  probdefs.rotationProbDef(obsR,self.N_objects)

                self.ATAR = self.ATAR  + np.dot(A.T,A)

        #calculates translations
        elif self.estimating == 't':
            
            if len(obsT)>0:

                A,b =  probdefs.translationProbDef(obsT,self.R,self.N_objects)

                self.ATAt = self.ATAt + np.dot(A.T,A) #way to save the matrix in a compact manner

                self.ATb = self.ATb + np.dot(A.T,b) #way to save the matrix in a compact manner

        else:
            print("Not Estimating Anything")

    def CalcRthenStartT(self):
            
            rotSols = algos.RProbSolv1(self.ATAR,3,self.N_objects)

            #converts to world coordinates or into them
            rotSolsNotUsed = mmnip.Transposer(rotSols)

            #converts in first ref coordinates , 
            rr = mmnip.genRotRelLeft(rotSolsNotUsed)

            
            self.R=rr

            self.estimating='t'

            return rr

    
    
    def CalcT(self):

        x = np.dot(np.linalg.pinv(self.ATAt),self.ATb)
    
        solsplit2 = np.split(x,self.N_objects)
  
        t=mmnip.Transl_fromWtoRef(self.R,solsplit2)

        self.t = t

        return t





        



    



            

            
            
        
        




   