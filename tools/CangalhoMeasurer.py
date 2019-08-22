import sys
import json
from libs import *

sys.path.append('..')


def main(argv):

    data={}

    arucoModel = FileIO.getFromPickle(argv[0])

    print(arucoModel)
    
    ts = arucoModel['T']

    for i in range(0,len(ts)):
        print("coordinates of " +str(i)+" relative to 0:")
        print(ts[i]) 

if __name__ == '__main__':
    main(sys.argv[1:])