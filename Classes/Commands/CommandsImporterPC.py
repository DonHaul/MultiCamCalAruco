import CommandsImporter
import sys,os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))+"/../..")
from libs import *

import open3d
import time



class CommandsImporterPC(CommandsImporter.CommandsImporter):
    def __init__(self,invoker,state):
        invoker.register("PC",SeePCs(state))
        invoker.register("PC5",SeePCTime(state))


class SeePCs(CommandsImporter.Command):
    
    def __init__(self, state):
        self.state = state

    def execute(self,*args):
        
       

        pcPath = FileIO.CreateFolder(self.state.path+"/PCs",putDate=False)

        newpcPath = FileIO.CreateFolderIncremental(pcPath+"/PC_")

        #save the individual pointclouds
        for i in range(len(self.state.pcs)):
            
            open3d.write_point_cloud(newpcPath +"/"+ self.state.camNames[i] +".pcd", self.state.pcs[i])

            H = mmnip.Rt2Homo(self.state.R[i],self.state.t[i].T)

            self.state.pcs[i].transform(H)
            print("TRANSFORMER")

        


        visu.draw_geometry(self.state.pcs)



class SeePCTime(CommandsImporter.Command):
    
    def __init__(self, state):
        self.state = state

    def execute(self,*args):
        
       

        pcPath = FileIO.CreateFolder(self.state.path+"/PCs",putDate=False)

        newpcPath = FileIO.CreateFolderIncremental(pcPath+"/PC_")

        time.sleep(5)

        #save the individual pointclouds
        for i in range(len(self.state.pcs)):
            
            open3d.write_point_cloud(newpcPath +"/"+ self.state.camNames[i] +".pcd", self.state.pcs[i])
            print("uhmwgat")
            H = mmnip.Rt2Homo(self.state.R[i],self.state.t[i].T)

            self.state.pcs[i].transform(H)
            print("TRANSFORMER")

        


        visu.draw_geometry(self.state.pcs)



