#HERE WILL BE the v1, but organized in a good fashion
import rospy
import message_filters
from sensor_msgs.msg import Image



import cv2
import open3d
import numpy as np
import time
from sensor_msgs.msg import CameraInfo


import sys

from libs import *

def main(argv):
    

    
    freq=50

    camNames = IRos.getAllPluggedCameras()
    camName = "mista"

    print(camNames)
    print(camName)

    topicDepthInfo = "/depth/camera_info"
    topicRGBInfo = "/rgb/camera_info"
    rospy.init_node('diavolo', anonymous=True)
    
    for camName in camNames:

        camInfoRGB = rospy.wait_for_message(camName + topicRGBInfo, CameraInfo)
        camInfoDepth = rospy.wait_for_message(camName + topicDepthInfo, CameraInfo)

        caminfo={'rgb':{},'depth':{}}
        caminfo['rgb']={'D':camInfoRGB.D,'K':np.asarray(camInfoRGB.K).reshape((3,3)),'R':np.asarray(camInfoRGB.R).reshape((3,3)),'P':np.asarray(camInfoRGB.P).reshape((3,4))}
        caminfo['depth']={'D':camInfoDepth.D,'K':np.asarray(camInfoDepth.K).reshape((3,3)),'R':np.asarray(camInfoDepth.R).reshape((3,3)),'P':np.asarray(camInfoDepth.P).reshape((3,4))}
    
        FileIO.saveAsPickle("camcalib_"+camName,caminfo,path="static/",putDate=False,animal=False)



if __name__ == '__main__':
    main(sys.argv)