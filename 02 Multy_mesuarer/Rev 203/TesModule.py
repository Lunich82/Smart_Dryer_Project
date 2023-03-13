import time
class Tes:
    command_tes_test = bytearray.fromhex('16 01 16 01 08 07 01 77')
    command_tes_get_data = bytearray.fromhex('16 01 16 01 08 04 63 77')
    command_tes_dont_sleep = bytearray.fromhex('16 01 16 01 01 21 80 77')
    command_tes_on = bytearray.fromhex('16 01 16 01 01 21 50 77')
    command_tes_off = bytearray.fromhex('16 01 16 01 01 21 40 77')
    
    def __init__(self, port, answerLen):
        self.port = port
        self.answerLen = answerLen
        
    def sendCommandOnTes(self, command):
        self.port.write(command)
        answer = self.port.read(self.answerLen)
        return answer

    def getData(self):
        answer = self.sendCommandOnTes(self.command_tes_get_data)
        #print("Answer from Tes: ", answer)
        time.sleep(1)
        return answer

    def isTesMeasuring(self):
        isMeasuring = 0
        answer = self.getData()
        #print(answer)
        if len(answer) > 8:
            isMeasuring = (answer[8]%2)
        return isMeasuring

    def startMeasuringTes(self):
        countIterate = 0
        isTesOff = True
        localDelay = 2       
        while isTesOff and countIterate < 5:
            #print("------start Measuring Tes------")
            answer = self.sendCommandOnTes(self.command_tes_on)
            #print("Answer from Tes: ", answer)
            if len(answer) == 4:
                if not answer[2] is None:
                    if answer[2] == 80:
                        isTesOff = False
                        #print("Tes Measuring started")
            if len(answer) == 5:
                if not answer[3] is None:
                    if answer[3] == 80:
                        isTesOff = False
                        #print("Tes Measuring started")
            countIterate = countIterate + 1
            time.sleep(localDelay)
        return not isTesOff

    def stopMeasuringTes(self):
        countIterate = 0
        localDelay = 2 
        isTesOn = True
        while isTesOn and countIterate < 5:
            #print("------stop Measuring Tes------")
            answer = self.sendCommandOnTes(self.command_tes_off)
            #print("Answer from Tes: ", answer)
            if len(answer) == 4:
                if not answer[2] is None:
                    if answer[2] == 64:
                        isTesOn = False
                        #print("Tes Measuring stoped")        
            countIterate = countIterate + 1
            time.sleep(localDelay)
        return not isTesOn
    
    
