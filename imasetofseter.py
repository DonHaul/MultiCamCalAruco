import os

directory ="/home/sipg_server/MscROSPackages/ImageSets/2019-10-07_20_06_26__stingray/"

camname="killerqueen"


for filename in os.listdir(directory):
    if camname in filename: 
         # print(os.path.join(directory, filename))
        fname = filename.replace(".","_")
        fname = fname.split("_")
        newnum=int(fname[2])-18

        
        os.rename(directory+filename,directory+"auxer"+"_"+fname[1]+"_"+str(newnum)+".png")
    else:
        continue


for filename in os.listdir(directory):
    if "auxer" in filename: 
         # print(os.path.join(directory, filename))
        fname = filename.replace(".","_")
        fname = fname.split("_")


        
        os.rename(directory+filename,directory+camname+"_"+fname[1]+"_"+fname[2]+".png")
    else:
        continue