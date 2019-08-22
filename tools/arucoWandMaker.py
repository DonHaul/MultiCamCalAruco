
import cv2
import rosinterface as IRos

import rospy

import sys
import numpy as np


from sensor_msgs.msg import Image


class LiveThing(object):

    def __init__(self):
        print("initiated")

        self.ids=[]

    def callback(self,data):


        img = IRos.rosImg2RGB(data)

        #What type of aruco markers are there
        adict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

        #make the image grey
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
        #get markers
        det_corners, ids, rejected  = cv2.aruco.detectMarkers(gray,dictionary=adict)

        hello = cv2.aruco.drawDetectedMarkers(img,det_corners,ids)

        if ids is not None:
            for i in ids:
                if i not in self.ids:
                    print(type(i))
                    print(i.shape)
                    
                    self.ids.append(i[0])
                    print(self.ids)

        cv2.imshow("Image window" , hello)
        cv2.waitKey(1)

            

if __name__ == "__main__":
    
    arucothing=LiveThing()

       
    camNames = IRos.getAllPluggedCameras()


    camName=camNames[0]
    

    rospy.init_node('do_u_kno_di_wae', anonymous=True)


    rospy.Subscriber(camName+"/rgb/image_color", Image, arucothing.callback)

    print("Fetching Messages")    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")


    print("yoyo2")
    print(arucothing.ids)