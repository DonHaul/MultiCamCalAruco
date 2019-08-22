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








import roslib
import rospy

import tf

import tf2_ros
import cv2
from geometry_msgs.msg import Pose

from libs import *

import numpy as np

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import sys
import geometry_msgs.msg

def main(argv):

  

    scene={}
    scene['camnames']=['diavolo','mista','speedwagon','emperorcrimson','killerqueen']




    print(scene)



    H = mmnip.Rt2Homo(R=np.eye(3),t=[0,0,0])

    #pub = rospy.Publisher('chatter', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
   

    


    static_tfs=[]
    
    count=0
    for i in range(0,len(scene['camnames'])):

      
        static_transformStamped=BuildStampedTransform("world",scene['camnames'][i],H)
        static_tfs.append(static_transformStamped)

        H = tf.transformations.quaternion_matrix([0.5,-0.5,0.5,0.5])
        static_transformStamped=BuildStampedTransform(scene['camnames'][i],scene['camnames'][i]+"_depth_optical_frame",H)
        static_tfs.append(static_transformStamped)


        print("Hello")

    broadcaster= tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(static_tfs)


    print("sending tf")
    rospy.spin()




def BuildStampedTransform(parent,child,H):

        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent
        static_transformStamped.child_frame_id = child
        transl = tf.transformations.translation_from_matrix(H)
        rot = tf.transformations.quaternion_from_matrix(H)

        static_transformStamped.transform.translation.x=transl[0]
        static_transformStamped.transform.translation.y=transl[1]
        static_transformStamped.transform.translation.z=transl[2]

        static_transformStamped.transform.rotation.x=rot[0]
        static_transformStamped.transform.rotation.y=rot[1]
        static_transformStamped.transform.rotation.z=rot[2]
        static_transformStamped.transform.rotation.w=rot[3]


        return static_transformStamped

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass