import libs.FileIO as FileIO


location="/home/ramiro/Downloads/"

ola = FileIO.getFromPickle(location + "poses.pickle")

print(ola)


FileIO.SaveAsMat(ola,location + "poses.mat")

