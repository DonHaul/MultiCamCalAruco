import threading
import cv2
import datetime


def worker(what_to_snap,name):
    x=""
    count = 0
    while x!="q":
        x= raw_input("press Enter to take snapshot")
        print("*SNAPPITY*")
        count=count+1
        cv2.imwrite("Snaps/"+name +" "+str(count)+".png",what_to_snap())

    return


def Start(what_to_snap,snapname="wow"):
    name = snapname+datetime.datetime.now().strftime("%d-%m-%Y %H-%M-%S")
    t = threading.Thread(target=worker,args=(what_to_snap,name,))
    t.start()
