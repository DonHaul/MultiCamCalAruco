import open3d
import matmanip as mnip
import visu
import numpy as np
import pickler2 as pickle


Rww=mnip.genRotMat([0,0,0])
tww=[[0],[0],[0]]

#visu.ViewRefs([Rww,Rw1],)

Rw1=mnip.genRotMat([0,-90,0])
print(Rw1)
tw1=[[10],[0],[-10]]

tw = [[10],[0],[-10]]

totransform =  [[20],[0],[-10]]

a = np.dot(Rw1,totransform)+tw1
print(a)


#visu.ViewRefs([Rww,Rw1,mnip.genRotMat([0,-90,0])])

print(mnip.InvertT(Rw1,tw1))



##########

import open3d
import matmanip as mnip
import visu
import numpy as np
import pickler2 as pickle

import synth



R,t = synth.Scenev1()

visu.ViewRefs(R,t)

print("Riw")
for r in R:
    print(r)


print("tiw")
for r in t:
    print(r)

newR=[]
newt = []

ref =1

newR = mnip.genRotRelLeft(R,ref)

for i in range(0,len(t)):
    newt.append(np.dot(R[ref].T,t[i])+mnip.InvertT(R[ref],t[ref]))


visu.ViewRefs(newR,newt)