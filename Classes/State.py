class State:
    def __init__(self):
        
        
        
        self.stop_threads = False

        self.GetStop = lambda : self.stop_threads
        self.folder =""

        

    def Stop(self):
        self.stop_threads=True
        