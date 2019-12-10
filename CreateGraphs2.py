import sys
import numpy as np



import libs.FileIO as FileIO
import csv


import matplotlib.pyplot as plt
import numpy as np

import datetime


 
def main(argv,meas1="reprojection_avg [px]",meas2="frame",prettyname="Average Reprojection Error",unit="[px]",folderpath="./Media/"):

    folderpath = FileIO.CreateFolder(folderpath,True,"DATA/")

    camnames =["diavolo","killerqueen","emperorcrimson"]

    metrics=["tcangalho","Rcangalhoangle","reprojectionNormAvg"]
    prettyname=["Cross Translation Error [m]","Cross Rotation Error [deg]", "Cross Average Reprojection Error [px]"]
    multimetrics =["terr","Rerrangle"]

    
    view=False
    data={}
    ids=[]
 

    with open(argv[1]) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                

                for m in metrics:
                    
                    ids.append(row.index(m))
                    

                if -1 in ids:
                    print("Measures not found")
                    quit()

                for m in multimetrics:
                    for i in range(0,len(camnames)):
                        for j in range(i+1,len(camnames)):
                            print(i,j)
                            print(camnames[i],camnames[j])
                            if m+"_"+camnames[i]+"_"+camnames[j] in row:
                                lol = m+"_"+camnames[i]+"_"+camnames[j]
                            else:
                                lol = m+"_"+camnames[j]+"_"+camnames[i]

                            prettyname= prettyname + ["Translation Error [m]","Rotation Error [deg]"]
                            metrics.append(lol)
                            ids.append(row.index(lol))
                    for m in metrics:
                        data[m] = []
                        


                line_count += 1
            else:
                
                i=-1
                for m in metrics:
                    i=i+1


                    if row[ids[i]] != float(-1.0):
                        data[m].append(float(row[ids[i]]))
                    else:
                        data[m].append(0)
               

                line_count += 1

    length = len(data['tcangalho'])

    featuresMean=[]
    featuresMedian=[]
    featuresStd =[]

    for key in data:
        #print(key)
        #print("lol")
        #print(data[key])
        #print(type(features[i][0]))
        featuresMean.append(np.mean(np.asarray(data[key])))
        featuresMedian.append(np.median(np.asarray(data[key])))
        featuresStd.append(np.std(np.asarray(data[key])))

    with open(folderpath+'/statistics_'+ datetime.datetime.now().strftime("%Y-%m-%d_%H_%M_%S") +'.csv', 'w') as csvfile:
        stats = csv.writer(csvfile, delimiter=';')
        stats.writerow(['Measure','Mean','Std','Median'])

        i=0
        for key in data: 
            stats.writerow([key,featuresMean[i],featuresStd[i],featuresMedian[i]])
            i=i+1
   

    small=10
    medium=16
    plt.rc('axes', titlesize=small)
    plt.rc('axes', labelsize=medium)   
    plt.rc('xtick', labelsize=small)
    plt.rc('ytick', labelsize=small)
    color = (0.2, 0.4, 0.6, 0.75)
    color2 = (0.2, 0.4, 0.6, 1)
    
    frames = range(0,length)
    
    j=-1
    for m in metrics:
        print(m)
        j=j+1

        #Draws BarPlot
        fig_object = plt.figure(figsize=(2000/80.0, 400/80.0), dpi=80)
        plt.xlim([0,length])
        plt.ylim([0,max(data[m])*1.1])
        
        plt.plot(frames,data[m], color=color,linewidth=1.2)
        
        #plt.bar
        plt.fill_between(frames,data[m],color=color)
        plt.ylabel(prettyname[j])
        plt.xlabel("frames")
        plt.tight_layout()
        if view:
            plt.show(block=True)
            plt.pause(1)
            plt.close()

        FileIO.SaveImageAllFormats(fig_object,m+"_in_time",folderpath)


        plt.rc('axes', titlesize=small)
        plt.rc('axes', labelsize=medium)   
        plt.rc('xtick', labelsize=small)
        plt.rc('ytick', labelsize=small)

        #Draws Histogram
        fig_object = plt.figure(figsize=(1400/80.0, 800/80.0), dpi=80)
        plt.hist(np.asarray(data[m])[np.asarray(data[m])>0], bins=100,color=color2,edgecolor=(1, 1, 1, 1))  # arguments are passed to np.histogram
        plt.ylabel("Frequency")
  
        plt.xlabel(prettyname[j])
        plt.tight_layout()
        FileIO.SaveImageAllFormats(fig_object,m+"_hist",folderpath)

        if view:
            plt.show(block=True)
            plt.pause(1)
            plt.close()

if __name__ == '__main__':
    main(sys.argv)