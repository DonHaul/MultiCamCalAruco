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
        
    #R,t,camNames
    scene = FileIO.LoadScene(filename)
    #print(scene)
    #quit()

    H = mmnip.Rt2Homo(scene[0][0])
    print(H)
    print(tf.transformations.quaternion_from_matrix(H))
    print(tf.transformations.translation_from_matrix(H))


    #pub = rospy.Publisher('chatter', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        for i in range(0,len(scene[2])):

            H = mmnip.Rt2Homo(scene[0][i],np.squeeze(scene[1][i]))

            br = tf.TransformBroadcaster()
            br.sendTransform(tf.transformations.translation_from_matrix(H),
                            tf.transformations.quaternion_from_matrix(H),
                            rospy.Time.now(),
                            scene[2][i],
                            "world")

            br.sendTransform(tf.transformations.translation_from_matrix(np.eye(4)),
                            tf.transformations.quaternion_from_matrix(np.eye(4)),
                            rospy.Time.now(),
                            scene[2][i]+"_rgb_optical_frame",
                            scene[2][i])
            hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hello_str)
            #pub.publish(hello_str)
            rate.sleep()
            print("sending tf")

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass