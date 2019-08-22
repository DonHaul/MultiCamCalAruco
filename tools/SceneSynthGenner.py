import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))+"/..")
from libs import *

R,t = synth.TestScene31()

visu.ViewRefs(R,t)

newt=[]
for tt in t:
    newt.append(np.expand_dims(np.asarray(tt),axis=1))

print(newt)

FileIO.saveAsPickle("TestScene",{'R':R,'t':newt},path="./static/fakecangalhos/")