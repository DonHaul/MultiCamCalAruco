"""
PosePipelineMaker.py

Generates and executes a pipeline to estimate poses
"""


import numpy as np
import time
import rospy
import numpy as np
from shutil import copyfile
from libs import *
import sys
import threading

import PointCloudVisualizer

import Classes.State as State

import CommandLine
from open3d import *

import Classes.Commands.CommandsImporterPC as CommandsImporterPC 

def worker(state):

    #boom = create_mesh_sphere(10)
    #print(type(boom))
    vis = Visualizer()
    vis.create_window()

            
    #vis.add_geometry(state.pcs[0])

    #print("EEERRRRR")
    #print(state.pcs)
    
    



    count = 0

    
    while state.stop_threads==False:
        
        time.sleep(0.5)
        if(state.updated==True):


            count=count+1
            print("UPDATE THEINGS",count)
            state.updated=False
            #print(state.pcs)

            vis.add_geometry(state.pcs[0])
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()



def main(argv):

    poses = FileIO.getFromPickle(argv[0])

    state = State.State()

    state.path=argv[0][0:argv[0].rfind('/')]
    


    print(state.path)

    state.camNames=poses['camnames']
    state.R=poses['R']
    state.t=poses['t']

    print(poses)

    print("YEET")

    PointCloudVisualizer.PCViewer(poses,argv[0],(480,640),state)

    #sets thread for terminal window
    CommandLine.Start(state,CommandsImporterPC.CommandsImporterPC)

    #sets thread for pipeline
    #t1 = threading.Thread(target=worker,args=( state,))
    #t1.start()
    
    

    try:
        print("SPINN")
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")

    state.Stop()
    
    #t1.join()

    
if __name__ == '__main__':
    main(sys.argv[1:])