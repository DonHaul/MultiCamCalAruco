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
import copy

#pipeline classes
from Classes.ImgReaders import RosStreamReader,ImgStreamReader,StreamReader,RosGatherStreamReader
from Classes.ObservationGenners import CamerasObservationMaker,CangalhoObservationsMaker, CangalhoSynthObsMaker, CameraSynthObsMaker, CameraSynthObsMaker2, CangalhoSynthObsMaker2
from Classes.ArucoDetecc import CangalhoPnPDetector,CangalhoProcrustesDetector,SingleArucosDetector
from Classes.PosesCalculators import PosesCalculator, OutlierRemPoseCalculator , PosesCalculatorSynth
from Classes import PosePipeline
from Classes.Commands import CommandsImporterPose
import CommandLine


def worker(posepipe):

    #executes pipeline untill it is stopped
    while True:
        #while there are new images

        #print("AVAILABILITY")
        #print(posepipe.imgStream.nextIsAvailable)
        if posepipe.imgStream.nextIsAvailable:
            
            

            #set input as consumed
            posepipe.imgStream.nextIsAvailable=False

            #gets next image
            streamData= posepipe.imgStream.next()

            #if streamData is not None:
            #    posepipe.imgShower(streamData)               

            #stop if there are no more images
            #if streamData is None:
            #    posepipe.Stop()
            #    break
            

            #generates observations
            img,ids,obsR,obsT = posepipe.ObservationMaker.GetObservations(streamData)

            #print("RR")
            #print(obsR)
            #print("TT")
            #print(obsT)

            #adds observations to matrices
            posepipe.posescalculator.AddObservations(obsR,obsT)
        
        
        elif posepipe.imgStream.finished:
            print("STOPPING")
            posepipe.Stop()
            break




        if posepipe.GetStop(): 
            print("EXITING")
            break    


            



def main(argv):
   
    if len(argv)==0:
       raise Exception('Please specify a Pipeline File')

    #Reads the configuration file
    data =  FileIO.getJsonFromFile(argv[0])
    

    posepipeline = PosePipeline.PosePipeline()

    #holds stuff
    state={}




    #hash of aruco detector classes
    arucodetectors={
        'singular':SingleArucosDetector.SingleArucosDetector,
        'allforone':CangalhoPnPDetector.CangalhoPnPDetector,
        'depthforone':CangalhoProcrustesDetector.CangalhoProcrustesDetector        }


    #In case we want to get the cameras we need the arucomodel    
    if "CAMERA" in data['model']['type']:
        state['arucomodel'] = FileIO.getFromPickle(data['model']['arucomodel'])

    if "record" not in data["model"]:
        data["model"]['record']=False




    #Assigns the InputStream
    if data['input']['type']=='IMG':
        
        #must set path where images are
        posepipeline.imgStream = ImgStreamReader.ImgStreamReader(data['input']['path'])
    elif data['input']['type']=='ROS':

 

        camNames = []

        #sets cameras if there are any
        if "cameras" in data['model']:
            camNames = data['model']['cameras'] 

        
        posepipeline.imgStream = RosStreamReader.RosStreamReader(camNames=camNames,inputData = data['input'])

        #setting stuff on state
        state['intrinsics'] = FileIO.getIntrinsics(posepipeline.imgStream.camNames)
        state['arucodata'] = FileIO.getJsonFromFile(data['model']['arucodata'])


    elif data['input']['type']=='ROS_GATHER':
        
        print("ROS GATHER MODE")

        camNames = []

        

        #sets cameras if there are any
        if "cameras" in data['model']:
            camNames = data['model']['cameras']

        

        
        posepipeline.imgStream = RosGatherStreamReader.RosGatherStreamReader(camNames=camNames,inputData = data['input'])

        #setting stuff on state
        state['intrinsics'] = FileIO.getIntrinsics(posepipeline.imgStream.camNames)
        state['arucodata'] = FileIO.getJsonFromFile(data['model']['arucodata'])
        state['arucomodel'] = FileIO.getFromPickle(data['model']['arucomodel'])

        


    elif data['input']['type']=='SYNTH':
        posepipeline.imgStream = StreamReader.StreamReader()


        state['synthmodel']=FileIO.getFromPickle(data['model']['arucomodel'])
        if data['model']['type']=="SYNTH_CAMERA" or data['model']['type']=="SYNTH_CAMERA2":
            state['modelscene']=FileIO.getFromPickle(data['model']['modelscene'])

        #print(state['synthmodel'][0])
        #print(state['synthmodel'][1])
        #visu.ViewRefs(state['synthmodel'][0],state['synthmodel'][1],refSize=1,showRef=True,saveImg=True,saveName=posepipeline.folder+"/screenshot.jpg")
    
        if "CAMERA" in data['model']['type']:
            posepipeline.posescalculator=PosesCalculatorSynth.PosesCalculatorSynth({"N_objects":len(state['modelscene']['R'])})

        else:
            posepipeline.posescalculator=PosesCalculatorSynth.PosesCalculatorSynth({"N_objects":len(state['synthmodel']['R'])})

        
    else:
        print("This Pipeline input is invalid")




    #Assigns observation maker and posecalculator
    if data['model']['type']=='CANGALHO':
        

        
        #static parameters
        singlecamData={
            "K":state['intrinsics'][posepipeline.imgStream.camNames[0]]['rgb']['K'],
            "D":state['intrinsics'][posepipeline.imgStream.camNames[0]]['rgb']['D'],
            "arucodata":state['arucodata']}

        #sets observation maker
        posepipeline.ObservationMaker =  CangalhoObservationsMaker.CangalhoObservationMaker(singlecamData)



        #sets pose calculator
        posedata={
            "N_objects":len(state['arucodata']['ids']),
            }

        if 'record' in data["model"]:
            posedata["record"]=data["model"]["record"]
        else:
            posedata["record"]=False


        posepipeline.posescalculator = PosesCalculator.PosesCalculator(posedata)


    elif data['model']['type']=='CAMERA':

        #static parameters
        multicamData={
            "intrinsics":state['intrinsics'],
            "arucodata":state['arucodata'],
            "camnames":posepipeline.imgStream.camNames,
            "arucomodel":state['arucomodel'],
            "innerpipeline":{
                "arucodetector":arucodetectors[data['model']['arucodetection']]({'arucodata':state['arucodata'],'arucomodel':state['arucomodel']})
            }
            }
        
        #sets observation maker
        posepipeline.ObservationMaker = CamerasObservationMaker.CamerasObservationMaker(multicamData)

        #sets pose calculator
        posedata={
            "N_objects":len(posepipeline.imgStream.camNames),
            "record":data["model"]["record"]}
        

        #sets observation treatment
        if data['model']['mode']['type']=='REGULAR':
            posepipeline.posescalculator = PosesCalculator.PosesCalculator(posedata)
        
        elif data['model']['mode']['type']=='OUTLIERREMOVE':


            print("YOOO")
            #static parameters
            posedata['observations']=data['model']['mode']['observations']
            posedata['Rcutoff']=data['model']['mode']['Rcutoff']
            posedata['Tcutoff']=data['model']['mode']['Tcutoff']
            print(posedata)
            

            posepipeline.posescalculator = OutlierRemPoseCalculator.OulierRemovalPoseCalculator(posedata)

        else:
            print("This pose calculator is invalid")

    elif data['model']['type']=='SYNTH_CANGALHO':
        
        #in order to not copy by reference https://stackoverflow.com/questions/3975376/understanding-dict-copy-shallow-or-deep
        obsdata=copy.deepcopy(data['model'])
        obsdata['synthmodel']=state['synthmodel']

        posepipeline.ObservationMaker= CangalhoSynthObsMaker.CangalhoSynthObsMaker(obsdata)
    elif data['model']['type']=='SYNTH_CAMERA':
        
        #in order to not copy by reference https://stackoverflow.com/questions/3975376/understanding-dict-copy-shallow-or-deep
        obsdata=copy.deepcopy(data['model'])

        obsdata['synthmodel']=state['synthmodel']
        obsdata['modelscene']=state['modelscene']

        print("JEFF")
        visu.ViewRefs(obsdata['modelscene'][0],obsdata['modelscene'][1])

        posepipeline.ObservationMaker= CameraSynthObsMaker.CameraSynthObsMaker(obsdata)
  

    elif data['model']['type']=='SYNTH_CAMERA2':
        
        #in order to not copy by reference https://stackoverflow.com/questions/3975376/understanding-dict-copy-shallow-or-deep
        obsdata=copy.deepcopy(data['model'])
        obsdata['synthmodel']=state['synthmodel']
        obsdata['modelscene']=state['modelscene']

        print("DAATAAAA")
        print(data['model'])

    
        #visu.ViewRefs(obsdata['modelscene']['R'],obsdata['modelscene']['t'])

        posepipeline.ObservationMaker= CameraSynthObsMaker2.CameraSynthObsMaker2(obsdata)
        
    elif data['model']['type']=='SYNTH_CANGALHO2':
        
        state['arucodata'] = FileIO.getJsonFromFile(data['model']['arucodata'])

        state['arucodata']['idmap'] = aruco.markerIdMapper(state['arucodata']['ids'])


        #in order to not copy by reference https://stackoverflow.com/questions/3975376/understanding-dict-copy-shallow-or-deep
        obsdata=copy.deepcopy(data['model'])
        obsdata['synthmodel']=state['synthmodel']
        obsdata['arucodata']=state['arucodata']
 
        print("DAATAAAA")
        print(data['model'])

    
        #visu.ViewRefs(obsdata['modelscene']['R'],obsdata['modelscene']['t'])

        posepipeline.ObservationMaker= CangalhoSynthObsMaker2.CangalhoSynthObsMaker2(obsdata)
        
    else:
        print("This Pipeline Model is invalid")


    #sets thread for terminal window
    CommandLine.Start(posepipeline,CommandsImporterPose.CommandsImporterPose)

    #sets thread for pipeline
    t1 = threading.Thread(target=worker,args=( posepipeline,))
    t1.start()




    #spins ros if necessary
    if data['input']['type']=='ROS' or data['input']['type']=='ROS_GATHER':
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("shut")

    print("Exited Stuff")
    posepipeline.Stop()
    
    t1.join() 
    
    print("FINISHED ELEGANTLY")

    #Only create log if full process was done
    if posepipeline.posescalculator.t is not None:

        #Create the folder
        posepipeline.folder = FileIO.CreateFolder("./Logs/",suffix=FileIO.GetAnimalName())

        #print("DATA IS")
        #print(data)
        #saves pipeline configuration on the outputfolder
        FileIO.putFileWithJson(data,"pipeline",posepipeline.folder+"/")


        datatosave= {"R":posepipeline.posescalculator.R,"t":posepipeline.posescalculator.t}
        

        #compute the corners of the cangalho
        if data['model']['type']=='CANGALHO' or data['model']['type']=='SYNTH_CANGALHO2':

            arucoModel = {"R":posepipeline.posescalculator.R,"T":posepipeline.posescalculator.t}

            corners = aruco.ComputeCorners(state['arucodata'],arucoModel)

            visu.SeePositions(corners)

            datatosave['corners']=corners
        

        #see and save resulting scene
        print(posepipeline.posescalculator.R)
        print(posepipeline.posescalculator.t)
        visu.ViewRefs(posepipeline.posescalculator.R,posepipeline.posescalculator.t,refSize=0.1,showRef=True,saveImg=True,saveName=posepipeline.folder+"/screenshot.jpg")

        #record r and t
        if data["model"]["record"]==True:
            recordeddata={
                "R":posepipeline.posescalculator.recordedRs,
                "T":posepipeline.posescalculator.recordedTs
            }

            print(len(recordeddata['R']))

            print(len(recordeddata['T']))

            FileIO.saveAsPickle("/recorded",recordeddata,posepipeline.folder,False,False)
        
        

        if data['input']['type']=='ROS' or data['input']['type']=='ROS_GATHER':

            datatosave['camnames']=posepipeline.imgStream.camNames


        FileIO.saveAsPickle("/poses",datatosave,posepipeline.folder,False,False)
    
 

if __name__ == '__main__':
    main(sys.argv[1:])