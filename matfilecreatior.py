import libs.FileIO as FileIO




import libs.FileIO as FileIO
import sys

"""
PosePipelineMaker.py

Generates and executes a pipeline to estimate poses
"""



def main(argv):

    poses = FileIO.getFromPickle(argv[0]+"poses.pickle")




    FileIO.SaveAsMat(poses,argv[0] + "poses.mat")

    
if __name__ == '__main__':
    main(sys.argv[1:])