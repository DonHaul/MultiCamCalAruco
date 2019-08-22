import sys
from libs import *
import numpy as np
import open3d

from shutil import copyfile

import csv

# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
import numpy as np





def main(argv):

    #Load aruco Model
    transformationz = FileIO.getFromPickle(argv[1]+"/recorded.pickle")



    

    folderpath = FileIO.CreateFolder(argv[1]+"/Errors/",putDate=False)

    T = np.asarray(transformationz['T'])
    T=np.squeeze(T)

    print(T.shape)
    R = np.asarray(transformationz['R'])
    print(R.shape)
    R = R.reshape((R.shape[0],9))

    Ravg = np.mean(R,axis=0)
    Tavg = np.mean(T,axis=0)

    print(R.shape)

    features=[]
    for i in range(T.shape[1]):
        features.append(T[:,i])

    for i in range(R.shape[1]):
        features.append(R[:,i])

    

    names = ["X","Y","Z","R00","R01","R02","R10","R11","R12","R20","R21","R22"]


    featuresMean=[]
    featuresMedian=[]
    featuresStd =[]
    
    
    for i in range(len(names)):
        featuresMean.append(np.mean(features[i]))
        featuresMedian.append(np.median(features[i]))
        featuresStd.append(np.std(features[i]))

    with open(folderpath+'statistics.csv', 'w') as csvfile:
        stats = csv.writer(csvfile, delimiter=';')
        stats.writerow(['Values','Mean','Std','Median'])

        for i in range(len(names)): 
            stats.writerow([names[i],featuresMean[i],featuresStd[i],featuresMedian[i]])
   

    #absolute error
    #featuresMean = np.expand_dims(featuresMean,axis=0)
    

    #relative error
    
    #features = np.abs(features-featuresMean)/featuresMean
    
    
    #Saves Translations
    for i in range(len(names)):    

        x= range(len(features[i]))

        color = (0.2, 0.4, 0.6, 1)
        print(features[i].shape)
        #Draws BarPlot
        fig_object = plt.figure(figsize=(1920/80.0, 1080/80.0), dpi=80)
        plt.bar(x,features[i],width=1.0,edgecolor=color, color=color)
        plt.title(names[i]+" over Time")

        FileIO.SaveImageAllFormats(fig_object,names[i]+"_in_time",folderpath)

        #plt.show(block=False)
        #plt.pause(1)
        #plt.close()
        
        #Draws Histogram
        fig_object = plt.figure(figsize=(1920/80.0, 1080/80.0), dpi=80)
        plt.hist(features[i], bins=100,color=color)  # arguments are passed to np.histogram
        plt.title("Histogram of "+names[i])
        FileIO.SaveImageAllFormats(fig_object,names[i]+"_hist",folderpath)

        #plt.show(block=False)
        #plt.pause(1)
        #plt.close()

if __name__ == '__main__':
    main(sys.argv)