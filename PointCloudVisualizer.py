#HERE WILL BE the v1, but organized in a good fashion
import rospy
import message_filters

from sensor_msgs.msg import Image
import cv2
import open3d
import numpy as np
import time

from libs import *

import sys

import copy
import open3d

class PCViewer():
    
    def __init__(self,poses,path,size,state):
        

        self.count = 0

        self.state=state


        self.state.pc = open3d.PointCloud()

        self.state.updated=False

        self.freq=0.0001
        self.R=poses['R']
        self.t=poses['t']
        self.camNames=poses['camnames']

        if len(self.camNames)==0:
            self.camNames = IRos.getAllPluggedCameras()

        #self.camNames=["diavolo","killerqueen"]

        #fetch K of existing cameras on the files
        self.intrinsics = FileIO.getIntrinsics(self.camNames)



        self.N_cams= len(self.camNames)

        self.state.pcs = [open3d.PointCloud() for i in range(self.N_cams)]
        print(len(self.state.pcs))

        self.height=size[0]
        self.width=size[1]

        rospy.init_node('do_u_kno_di_wae', anonymous=True)
        
        camSub = []

        #getting subscirpters to use message fitlers on/speedwagon/rgb/image_rect_color
        for name in self.camNames:
            print(name)
            camSub.append(message_filters.Subscriber(name+"/rgb/image_rect_color", Image))
            camSub.append(message_filters.Subscriber(name+"/depth_registered/sw_registered/image_rect_raw", Image))




        ts = message_filters.ApproximateTimeSynchronizer(camSub,20, 1.0/self.freq, allow_headerless=True)
        ts.registerCallback(self.callback)

        


    def callback(self,*args):

        print("CB")

        self.count = self.count + 1
        print(self.count)
        

        #iterate throguh cameras
        for camId in range(0,self.N_cams):
            

            #RGB
            rgb = IRos.rosImg2RGB(args[camId*2])
            #depth
            depth_reg = IRos.rosImg2Depth(args[camId*2+1])

            K = self.intrinsics[self.camNames[camId]]['rgb']['K']
            print(depth_reg.shape)
            #points,colors = mmnip.depthimg2xyz(depth_reg,rgb,self.intrinsics['K'][self.camNames[camId]])
            points = mmnip.depthimg2xyz2(depth_reg,K,(480,640))
            points = points.reshape((480*640, 3))

            #print(colors.shape)
            rgb1 = rgb.reshape((480*640, 3))#colors
            
            self.state.pcs[camId] = pointclouder.Points2Cloud(points,rgb1,clean=True,existingPc=self.state.pcs[camId])

        #visu.draw_geometry(self.state.pcs)

        self.state.updated=True

        #self.state.rgb = np.asarray(pcs[0].colors)
        #self.state.xyz = np.asarray(pcs[0].points)


        #print(self.state.pcs)
        #self.state.pc = pointclouder.MergeClouds(pcs2)



            