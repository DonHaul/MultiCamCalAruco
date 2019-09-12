from libs import IRos
import sys

import rospy
import numpy as np
import cv2
import message_filters

from sensor_msgs.msg import Image
from libs import FileIO

       

class Ros2Img:
    def __init__(self,folderpath="../ImageSets",camNames=None):

        self.freq=0.0001

        self.folderpath = folderpath

        
        self.folderpath =  FileIO.CreateFolder(self.folderpath+"/",True,"_"+FileIO.GetAnimalName())

        if camNames is None:
            self.camNames = IRos.getAllPluggedCameras()

       
        self.N_cams= len(self.camNames)

        self.view=False

        

        rospy.init_node('do_u_kno_di_wae', anonymous=True)
        
        camSub = []

        #getting subscirpters to use message fitlers on/speedwagon/rgb/image_rect_color
        for name in self.camNames:
            
            camSub.append(message_filters.Subscriber(name+"/rgb/image_rect_color", Image))
            #camSub.append(message_filters.Subscriber(name+"/depth_registered/sw_registered/image_rect_raw", Image))




        ts = message_filters.ApproximateTimeSynchronizer(camSub,20, 1.0/self.freq, allow_headerless=True)
        ts.registerCallback(self.callback)
        self.count=0
        
        
    
    def callback(self,*args):

        for i in range(self.N_cams):
            rgb = IRos.rosImg2RGB(args[i])
            #depth_reg = IRos.rosImg2Depth(args[1])

            #copy image
            #hello = rgb.astype(np.uint8).copy() 

            if self.view:
                cv2.imshow("Camera_"+self.camNames[i],hello)
                cv2.waitKey(30)

            cv2.imwrite(self.folderpath+"/"+self.camNames[i]+"_rgb_"+str(self.count)+".png",rgb)
        
        self.count = self.count + 1


if __name__ == "__main__":
    folderpath = sys.argv[1]

    r2i = Ros2Img(folderpath,folderpath="../ImageSets",camNames=sys.argv[2:])

    try:
        print("SPINN")
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")

    FileIO.putFileWithJson({"camnames":r2i.camNames,"count":r2i.count},"info",r2i.folderpath)

    cv2.destroyAllWindows()
