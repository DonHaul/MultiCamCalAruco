import sys
import json

def main(argv):

    data={}

    if len(argv)>0:
        f=open(argv[0],"r")
        
    string = f.read()

    lastname=""

    lines = string.split('\n')
    
    #print(lines)

    auxArr = []

    print(type(auxArr))
    for l in lines:

        arr = l.split(' ')
        #print(l)
        #print("l")
            
        try:
            val = float(arr[0])

            arr = l.split(' ')
            #print(type(auxArr))
            auxArr.append(arr)    

        except ValueError:
            
            if(len(auxArr)>0):    
                print(lastname)
                print(auxArr) 
                data[lastname]=auxArr
            lastname = arr[0]
            auxArr=[]

    f = open("output.json",'w')
    json.dump(data,f)

if __name__ == '__main__':
    main(sys.argv[1:])