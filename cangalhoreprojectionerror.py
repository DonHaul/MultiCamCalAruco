

from libs import *

model = FileIO.getFromPickle("./static/arucoModelNEW.pickle")

print(model)

R=model['R']
t=model['t']

visu.ViewRefs(R,t,0.1)