import time
from datetime import datetime, timedelta

class Sleeper ():
    
    def __init__(self,dataWorker):
        self.dataWorker = dataWorker

    def sleep(self, timeToSleep):
        nowTime = datetime.now()
        timeDelta = timedelta(seconds = timeToSleep)
        wakeUpTime = nowTime + timeDelta
        #print(wakeUpTime)
        #print("------go work------")

        while nowTime < wakeUpTime:
            #print("------iterate------")
            time.sleep(5)
            nowTime = datetime.now()
            needMeasuringNow = self.dataWorker.getNeedMeasuringNow()
            
            if needMeasuringNow:
                self.dataWorker.setNeedMeasuringNow(0)
                #print("------break------")
                break;

        
        
    
