"""
CamPoseGetter.py

This module contains the a class receives all the images and observations of the cameras, and calculates stuff with it
"""

import numpy as np
import cv2
from libs import *

import os

import StreamReader


class ImgStreamReader(StreamReader.StreamReader):
    def __init__(self,path):

        StreamReader.StreamReader.__init__(self)
        
        self.path=path
        self.count=0

        self.camNames= os.listdir(path)
        
        print(self.camNames)

        self.nextIsAvailable=True

    
    def next(self):
        
        super(StreamReader,self).next()

        data={'names':self.camNames,'rgb':[],'depth':[]}

        for name in self.camNames:

                rgbpath=self.path + '/'+name+'/'+'rgb_'+str(self.count)+'.png'
                depthpath=self.path + '/'+name+'/'+'depth_'+str(self.count)+'.png'

                if os.path.isfile(rgbpath):
                    data['rgb'].append(cv2.imread(rgbpath))

                if os.path.isfile(depthpath):
                    data['depth'].append(cv2.imread(depthpath))

        self.count = self.count + 1

        self.nextIsAvailable=True
        print("SET BACK TO TREU")
        
        if(len(data['rgb'])==0):
            return None

        return data        


        



        



    



            

            
            
        
        




   