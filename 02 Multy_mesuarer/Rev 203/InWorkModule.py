import threading
import RPi.GPIO as GPIO
import time
from datetime import datetime

class InWork (threading.Thread):
    isWorking = 1
    delay = 1   
    valuePin = 0
    jamperPin = 0
    
    def __init__(self, name, valuePin, jamperPin, blockStartEnginePin, blockStartEngineWarningPin, dataWorker):
        threading.Thread.__init__(self)
        self.name = name
        self.valuePin = valuePin
        self.jamperPin = jamperPin
        self.dataWorker = dataWorker
        self.blockStartEnginePin = blockStartEnginePin
        self.blockStartEngineWarningPin = blockStartEngineWarningPin
        
    def stop(self):
        self.isWorking = 0

    def run(self):
        #print("Start InWork")
        while self.isWorking:
            oldInWork = self.dataWorker.getInWorkEngine()  
            inWork = self.updateInWork()
            isEngineStartBlock, isEngineStartBlockWarning = self.updateEngineBlocking()
            isEngineStartFail = self.updateEngineStartFail()

            self.dataWorker.setInWorkEngineData(inWork,isEngineStartBlock,isEngineStartBlockWarning,isEngineStartFail)

            if not inWork and oldInWork:
                #print("Update setLastEngineStopDateTime = ", datetime.now())
                self.dataWorker.setLastEngineStopDateTime(datetime.now())

            time.sleep(self.delay)
        #print("Stop InWork") 

    def updateInWork(self):
        inWork = self.getAveragePinValue(self.valuePin);
        #print("new inWork = ", inWork)
        return inWork

    def updateEngineBlocking(self):
        isNeedBlockEngine = self.getAveragePinValue(self.jamperPin)

        valueR30 = self.dataWorker.getR30()
        isEngineStartBlock = valueR30 < self.dataWorker.getEngineStartBlockValue()

        if not isNeedBlockEngine:
            GPIO.output(self.blockStartEnginePin, isEngineStartBlock)
            isEngineStartBlock = int(isEngineStartBlock)
        else:
            GPIO.output(self.blockStartEnginePin,0)
            isEngineStartBlock = 0
    
        isEngineStartBlockWarning = valueR30 <  self.dataWorker.getEngineStartBlockWarningValue()
        GPIO.output(self.blockStartEngineWarningPin ,isEngineStartBlockWarning)
        isEngineStartBlockWarning = int(isEngineStartBlockWarning)
        
        return isEngineStartBlock, isEngineStartBlockWarning

    def getAveragePinValue(self, pin):
        firstValue = GPIO.input(pin)
        x = 0
        while x < 20: 
            secondValue = GPIO.input(pin)
            x = x + 1
            if firstValue and not secondValue:
                firstValue = 0
            time.sleep(0.2)
            
        if (firstValue):
            return 1
        else:
            return 0

    def updateEngineStartFail(self):
        isEngineBlock = self.dataWorker.getIsEngineStartBlock()
        isEngineWork = self.dataWorker.getInWorkEngine()
        
        if isEngineWork == 1 and isEngineBlock:
            isEngineStartFail = 1
        else:
            isEngineStartFail = 0
            
        return isEngineStartFail


