import os
from distutils.dir_util import copy_tree
import libs
import shutil

ola = os.listdir('./errors')


for o in ola:
    
    newO =  o.replace(":", "_")
    print(newO)
    if newO == o:
        continue
    
    libs.FileIO.CreateFolder('./errors/' + newO,putDate=False)
    copy_tree('./errors/' + o ,'./errors/' + newO)

    shutil.rmtree('./errors/' + o)