import threading
import time
import sys
import traceback
from datetime import datetime

class Modem (threading.Thread):
    isWorking = 1
    modemWorkDelay = 0.5
    port = None
           
    def __init__(self, name, port, modbus,fileWorker):
        threading.Thread.__init__(self)
        self.name = name
        self.port = port
        self.dataModbusWorker = modbus
        self.fileWorker=fileWorker

    def stop(self):
        self.isWorking = 0

    def run(self):
        print("Start work with modem")
        self.fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " Start work with modem")
        while self.isWorking:
            modemData = self.port.read(20)
            #print('-------Receive from modem: ', modemData)
            try:
                if len(modemData) > 2:
                    response = self.dataModbusWorker.getResponse(modemData)
                    #print("response",response)
                    if not response is None:
                        self.port.write(response)
                else:
                    castil = 1
                    #print("Request not good")
            except Exception as e:             
                print("Error in modem")
                print ("Error: ",sys.exc_info()[0])
                st=""
                for tb in traceback.format_tb(sys.exc_info()[2]):
                    print(tb + '\n')
                    st = st +tb + '\n'
                self.fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " Error in modem: " + st)
            finally:
                time.sleep(self.modemWorkDelay)
        print("Stop work with modem")
        self.fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " Stop work with modem")

    def runTest(self):
        print("Start work with modem")
        while self.isWorking:
            modemData = self.port.read(20)
            print('-------Receive from modem: ', modemData)
            time.sleep(self.modemWorkDelay)
        print("Stop work with modem")
    
