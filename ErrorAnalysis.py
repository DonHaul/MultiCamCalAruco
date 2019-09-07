'''
ErrorMnaager.py - generates a bunch of pipelines with different parameters
'''


from libs import FileIO
import PosePipelineMaker
import ErrorCalcs
import csv
import operator as op
from functools import reduce

def ncr(n, r):
    r = min(r, n-r)
    numer = reduce(op.mul, range(n, n-r, -1), 1)
    denom = reduce(op.mul, range(1, r+1), 1)
    return numer / denom


#baseline file
data =  FileIO.getJsonFromFile("./Pipelines/cangalhosynth2.json")

pixelnoisestd=[0.1,0.3,0.5,1,2,3,4,5]
n_detectedarucos=[3,5,7,12]
samples=[30,60,90,120,240]



resultfolders = []
errors = []


folderpath = FileIO.CreateFolder("./Errors/Err",putDate=True)

measurestext=['corner','center','reprojection','translation','angle','rodriguez']
measureunit = ['[m]','[m]','[px]','[m]','[deg]','']
statstext=['total','avg','median','std'] # i omitted the singular, cuz i dont need it here

fullmeasures = []
count = 0
for m in measurestext:
    for stat in statstext:
        fullmeasures.append(m+"_"+stat+" "+measureunit[count])
    count= count+1

with open(folderpath+'/__errtables.csv', 'wb') as csvfile:
    filewriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)

    #write header
    filewriter.writerow(['name','PixelStandardDeviation [px]','detected per frame','frames','obs per frame','total obs'] + fullmeasures)

    for pxstd in pixelnoisestd:
        for n_obs in n_detectedarucos:
            for samps in samples:
                data['model']['pixelnoisestd']=pxstd
                data['model']['n_detectedarucos']=n_obs
                data['model']['samples']=samps

                

                FileIO.putFileWithJson(data,"auxpipeline","./tmp")

                data = FileIO.getJsonFromFile("./tmp/auxpipeline.json")


                resultsfolder = PosePipelineMaker.main("./tmp/auxpipeline.json",view=False)

                resultfolders.append(resultsfolder)

                error = ErrorCalcs.main(resultsfolder)

                #fetch only folder name
                resultsfolder = resultsfolder.split('/')
                resultsfolder = (resultsfolder[len(resultsfolder)-1])

                #backup everything here

                
                FileIO.putFileWithJson(error,resultsfolder,folderpath)

                errorMeasures = []

                for m in measurestext:

                    for stat in statstext:
                        errorMeasures.append(error[m][stat])

                #addline
                filewriter.writerow([resultsfolder,pxstd,n_obs,samps,ncr(n_obs,samps),ncr(n_obs,samps)*samps]+errorMeasures)




