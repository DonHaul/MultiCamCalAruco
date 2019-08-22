import roslib
import rospy

import tf
import cv2
from geometry_msgs.msg import Pose

from libs import *

import numpy as np

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import sys


def main(argv):

    if(len(argv)>1):
        filename=argv[1]
    else:
        print("Scene File Needed")
        quit()

    scene = FileIO.getFromPickle(argv[1])
        

    print(scene)



    H = mmnip.Rt2Homo(R=np.eye(3),t=[1,1,1])

    #pub = rospy.Publisher('chatter', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        br = tf.TransformBroadcaster()
        

        for i in range(0,len(scene['camnames'])):

            H = mmnip.Rt2Homo(scene['R'][i],np.squeeze(scene['t'][i]))

            br = tf.TransformBroadcaster()
            br.sendTransform(tf.transformations.translation_from_matrix(H),
                            tf.transformations.quaternion_from_matrix(H),
                            rospy.Time.now(),
                            scene['camnames'][i],
                            "world")

            br.sendTransform(tf.transformations.translation_from_matrix(np.eye(4)),(0.5,-0.5,0.5,0.5),
                            rospy.Time.now(),
                            scene['camnames'][i]+"_link",
                            scene['camnames'][i])
            
            hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hello_str
            rate.sleep()
            print("sending tf")

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass