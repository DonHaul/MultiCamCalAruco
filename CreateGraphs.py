import sys
import numpy as np



import libs.FileIO as FileIO
import csv


import matplotlib.pyplot as plt
import numpy as np

import datetime


 
def main(argv,meas1="reprojection_avg [px]",meas2="frame",prettyname="Average Reprojection Error",unit="[px]",folderpath="./Media/"):

    
    id1=-1
    id2=-1
    arr1=[]
    arr2=[]

    with open(argv[1]) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                

                if meas1 in row and meas2 in row:

                    id1 = row.index( meas1 )
                    id2 = row.index( meas2 )

                else:
                    print("Measures not found")
                    quit()

                line_count += 1
            else:
                
                if(row[id1] != "-1.0" and row[id2] != "-1.0"):
                    arr1.append(row[id1])
                    arr2.append(row[id2])
                

                line_count += 1

 


    names = [meas1]
    featurezero = np.asarray(arr2,dtype=np.float64)
    features=[np.asarray(arr1,dtype=np.float64)]


    featuresMean=[]
    featuresMedian=[]
    featuresStd =[]
    
    
    for i in range(len(names)):
        #print(type(features[i][0]))
        featuresMean.append(np.mean(features[i]))
        featuresMedian.append(np.median(features[i]))
        featuresStd.append(np.std(features[i]))

    with open(folderpath+'\statistics_'+ datetime.datetime.now().strftime("%Y-%m-%d_%H_%M_%S") +'.csv', 'w') as csvfile:
        stats = csv.writer(csvfile, delimiter=';')
        stats.writerow(['Measure','Mean','Std','Median'])

        for i in range(len(names)): 
            stats.writerow([names[i],featuresMean[i],featuresStd[i],featuresMedian[i]])
   
    i=0

    small=10
    medium=16
    plt.rc('axes', titlesize=small)
    plt.rc('axes', labelsize=medium)   
    plt.rc('xtick', labelsize=small)
    plt.rc('ytick', labelsize=small)

    color = (0.2, 0.4, 0.6, 1)
    print(features[i].shape)
    #Draws BarPlot
    fig_object = plt.figure(figsize=(1920/80.0, 1080/80.0), dpi=80)
    plt.xlim([0,1200])
    
    plt.bar(featurezero,features[i],width=1.0,edgecolor=color, color=color)
    plt.ylabel(prettyname +" "+ unit )
    plt.xlabel("frames")
    
    #plt.show(block=True)
    #plt.pause(1)
    #plt.close()

    FileIO.SaveImageAllFormats(fig_object,names[i]+"_in_time",folderpath)


    plt.rc('axes', titlesize=small)
    plt.rc('axes', labelsize=medium)   
    plt.rc('xtick', labelsize=small)
    plt.rc('ytick', labelsize=small)

    #Draws Histogram
    fig_object = plt.figure(figsize=(1920/80.0, 1080/80.0), dpi=80)
    plt.hist(features[i], bins=50,color=color,edgecolor=(1, 1, 1, 1))  # arguments are passed to np.histogram
    plt.ylabel("Frequency")
    plt.xlabel(prettyname +" "+ unit )
    
    FileIO.SaveImageAllFormats(fig_object,names[i]+"_hist",folderpath)

    plt.show(block=True)

    #plt.pause(1)
    #plt.close()

if __name__ == '__main__':
    main(sys.argv)