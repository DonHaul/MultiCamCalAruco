import libs.FileIO as FileIO


#location="/home/ramiro/Downloads/"

#ola = FileIO.getFromPickle(location + "poses.pickle")

#print(ola)


#FileIO.SaveAsMat(ola,location + "poses.mat")


string ="/./Logs/2019-09-07_23:26:54_mongoose.json"

ola = string.split('/')

print(ola[len(ola)-1])