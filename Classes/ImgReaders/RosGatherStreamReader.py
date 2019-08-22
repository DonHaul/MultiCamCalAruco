
import numpy as np
import cv2
from libs import *

import message_filters

import os

from sensor_msgs.msg import Image

import threading

import rospy

import StreamReader

class RosGatherStreamReader(StreamReader.StreamReader):
    
    def __init__(self,camNames=None,inputData=None,freq=2,mutex=None):


        self.mutex=mutex

        StreamReader.StreamReader.__init__(self)

        self.freq = freq

        self.camNames=camNames

        if len(self.camNames)==0:
            self.camNames = IRos.getAllPluggedCameras()

              
        #number of camera
        self.N_cams = len(self.camNames)



        #stasticial var with the total number of times it receives information for each camera
        self.gatherCounter = [0] * self.N_cams

        #Array with 0 and 1 telling from which camera it received info from
        self.gatherReady = np.zeros((self.N_cams),dtype=np.uint8)



        self.N_cams = len(self.camNames)

        self.data={'names':self.camNames,'rgb':[None] * self.N_cams,'depth':[None] * self.N_cams}

        self.nextIsAvailable=False

        self.topicRGB=inputData['rgbtopic']
        self.topicDepth=inputData['depthtopic']

        

        Readers=[]
      
        rospy.init_node('my_name_is_jeff', anonymous=True)

        for i in range(0,len(self.camNames)):

            #initialize class for each camera
            ig = ReaderCallback(self.camNames[i],i,self,"rgb")

            #saves it for some reason
            Readers.append(ig)

            #subscribe to each camera
            rospy.Subscriber(self.camNames[i] + self.topicRGB, Image, ig.callback)


            #initialize class for each camera
            ig = ReaderCallback(self.camNames[i],i,self,"depth")

            #saves it for some reason
            Readers.append(ig)

            #subscribe to each camera
            rospy.Subscriber(self.camNames[i] + self.topicDepth, Image, ig.callback)


    def GatherImg(self,camId,imgtype,img):
        '''Gathers Images and observations from a specific camera
        Args:
            camId:from which camera this is coming
            img: the image that the camera is transmitting
            obs: the observations extracted from the image
        '''
        
        #set image
        if self.mutex is not None:

            with self.mutex:
                self.data[imgtype][camId] = img
        else:
            self.data[imgtype][camId] = img
        print(self.data[imgtype][camId].shape)
        
        #increment statistic
        self.gatherCounter[camId] = self.gatherCounter[camId] +1
        
        #set is as fetched
        self.gatherReady[camId]=1

        #print(self.gatherReady)

        #if all camera have sent something
        if(np.sum(self.gatherReady)== self.N_cams):

            print("GATHERED")

            print(self.data[imgtype][0].shape)
            print(self.data[imgtype][1].shape)

            self.nextIsAvailable=True

            self.gatherReady = np.zeros((self.N_cams),dtype=np.uint8)
    

    def next(self):
        

        #print(type(RosStreamReadear))
        self.nextIsAvailable=False # this should be replaced by the next line
        #super(RosStreamReadear,self).next()

        return self.data    
        
class ReaderCallback():
    
    def __init__(self,camName,camId,gatherer,imgtype):       
        
        self.camId = camId
        self.imgtype = imgtype
        self.camName = camName #number of markers, MUST BE CONTIGUOUS for this to work
        self.gatherer=gatherer #offset dos markers, como vao do 2 ao 12, o offset e 2
        
    def callback(self,data):
        '''Callback called upon new message arrives on ROS
        
        Args:
            data: data on that message
            args: arguments I dont actually need and should delete
        '''
        if(self.imgtype=="rgb"):
            img = IRos.rosImg2RGB(data)
        elif(self.imgtype=="depth"):
            img = IRos.rosImg2Depth(data)
        else:
            print("error")
        
        #transmit image to gatherer
        self.gatherer.GatherImg(self.camId,self.imgtype,img)
        

        self.nextIsAvailable=True
