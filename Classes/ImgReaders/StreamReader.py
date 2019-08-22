class StreamReader():        
        
        def __init__(self):
            
            #next is only available once
            self.nextIsAvailable=True
            self.finished=False

        

        def next(self):

            self.nextIsAvailable=False
            return None 