import time
from pathlib import Path
import sys
import serial
import traceback
import os
import RPi.GPIO as GPIO
from DataModule import DataWorker
from FileModule import FileWorker
from ModbusModule import ModbusWorker
from InWorkModule import InWork
from ModemModule import Modem
from TesModule import Tes
from SleepModule import Sleeper
from datetime import datetime, timedelta

def initPortsAndPins():
    portUSB = serial.Serial(
        "/dev/ttyUSB0",
        baudrate=9600,
        parity = serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1)
    
    portSerial = serial.Serial(
        "/dev/serial0",
        baudrate=9600,
        parity = serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize = serial. EIGHTBITS,
        timeout = 1)

    GPIO.setmode(GPIO.BCM)

    GPIO.setup(supply_pin,GPIO.OUT)
    GPIO.output(supply_pin,False)
    
    GPIO.setup(switch_off_measuring_cable_pin,GPIO.OUT)
    GPIO.output(switch_off_measuring_cable_pin,False)

    GPIO.setup(block_start_engine_pin,GPIO.OUT)
    GPIO.output(block_start_engine_pin,False)

    GPIO.setup(block_start_engine_warning_pin,GPIO.OUT)
    GPIO.output(block_start_engine_warning_pin,False)

    GPIO.setup(in_work_out_pin,GPIO.OUT)
    GPIO.output(in_work_out_pin,True)
    GPIO.setup(in_work_in_pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

    GPIO.setup(jamper_out_pin,GPIO.OUT)
    GPIO.output(jamper_out_pin,True)
    GPIO.setup(jamper_in_pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

    return [portUSB, portSerial]

def getMOm(x1, x2, mode):
    if mode == 60:
        return (x1/10+x2/1000)
    if mode == 61:
        return (x1+x2/100)
    if mode == 62:
        return (x1*10+x2/10)
    if mode == 63:
        return (x1*100+x2)
    if mode == 64:
        return (x1*100+x2)
    
def getValueFromAnswer(answer,rTamu):
    x1 = int(hex(answer[6])[2:])
    x2 = int(hex(answer[7])[2:])
    mode = int(hex(answer[5])[2:])
    if rTamu > 0:
        rTotal = round(getMOm(x1,x2,mode)*1000)
        if rTamu - rTotal == 0: #CASTIL
            rTotal = rTotal -1
        valuekOmh = rTamu * rTotal / abs(rTamu - rTotal)
    else:
        valuekOmh = round(getMOm(x1,x2,mode)*1000)
    return valuekOmh

def recalcRTamu(rTotal, rEngine):
    if rEngine - rTotal == 0: #CASTIL
        rTotal = rTotal - 1   
    return rTotal * rEngine / abs(rEngine - rTotal)

def isNeedSleeping(lastEngineStopDateTime, tSleepAfterWorkDelta, now):
    #print("dateNow = ", now, " lastEngineStopDateTime = ", lastEngineStopDateTime, " tSleepAfterWorkDelta = ", tSleepAfterWorkDelta)
    return lastEngineStopDateTime + tSleepAfterWorkDelta > now

#if __name__ == '__main__':
"""define Pins const"""
supply_pin = 25
block_start_engine_pin = 26
block_start_engine_warning_pin = 19
in_work_out_pin = 22
in_work_in_pin = 27
jamper_out_pin = 20
jamper_in_pin = 21
switch_off_measuring_cable_pin = 17

"""program"""
print("Wait 60 sec")
time.sleep(60)

my_file = Path("/home/pi/Desktop/py/StopAutoStart")
if my_file.is_file():
    sys.exit("Stop Autostart")

print("Start work")
        
try:
    ports = initPortsAndPins()
    portUSB = ports[1]
    portSerial = ports[0]

    fileWorker = FileWorker("/home/pi/Desktop/py/config","/home/pi/Desktop/py/lastState","/home/pi/Desktop/py/defaultconfig")
    dataWorker = DataWorker(fileWorker)
    modbusWorker = ModbusWorker(dataWorker)
    sleeper = Sleeper(dataWorker)

    dataWorker.readConfigFromFile()
    dataWorker.readLastStateFromFile()

    threadInWork = InWork("inWorkThread",in_work_in_pin, jamper_in_pin, block_start_engine_pin, block_start_engine_warning_pin, dataWorker)
    threadModem= Modem("modemThread", portSerial, modbusWorker, fileWorker)#add fileWorker
        
    tesAnswerLen = 15
    tes = Tes(portUSB, tesAnswerLen)

    threadInWork.start()
    threadModem.start()

    inWork = False
    valuekOm0 = 0
    valuekOm30 = 0
    valuekOm60 = 0

    fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") + " Start work")   
    while True:
        #print("------ITERATE------")
        isNeedSleep = False
        time.sleep(1)
        GPIO.output(supply_pin,False)
        GPIO.output(switch_off_measuring_cable_pin,False)

        lastEngineStopDateTime = dataWorker.getLastEngineStopDateTime()

        if not lastEngineStopDateTime == 0:       
            dateNow = datetime.now()
            
            try:
                tSleepAfterWorkDelta = timedelta(seconds = dataWorker.getTimeSleepAfterWork())
            except:
                tSleepAfterWorkDelta = 0
                
            isNeedSleep = isNeedSleeping(lastEngineStopDateTime, tSleepAfterWorkDelta, dateNow)
            
            if isNeedSleep:
                #print("sleep with T_SLEEP_AFTER_WORK")
                #fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " sleep with T_SLEEP_AFTER_WORK")
                sleepFor = lastEngineStopDateTime + tSleepAfterWorkDelta - dateNow
                #time.sleep(sleepFor.total_seconds())
                #time.sleep(T_SLEEP_AFTER_WORK)
                sleeper.sleep(sleepFor.total_seconds())
                continue
            
        inWork = dataWorker.getInWorkEngine();

        if not inWork:
            time.sleep(2)
            #print("------switch On Tes by pin------")
            GPIO.output(supply_pin,True)
            GPIO.output(switch_off_measuring_cable_pin,True)
            time.sleep(5)
            isMeasuringStarted = tes.startMeasuringTes()
            time.sleep(2)
            isTesMeasuring = tes.isTesMeasuring()

            if isMeasuringStarted and isTesMeasuring:
                time.sleep(1)
                answer = tes.getData()
                if len(answer) == 10:
                    time.sleep(5)
                    answer0 = tes.getData()
                    #print("answer0 from tes: ",answer0)
                    #print(dataWorker.getCalcStatus())
                    #fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") + " answer0 from tes: " + str(answer0))
                    if dataWorker.getCalcStatus() == 1:
                        try:
                            valuekOm0 = getValueFromAnswer(answer0, 0)
                            rTamu = recalcRTamu(valuekOm0, dataWorker.getREngine())
                            dataWorker.setRTamu(rTamu)
                        finally:
                            isMeasuringStoped = tes.stopMeasuringTes()
                            GPIO.output(supply_pin,False)
                            GPIO.output(switch_off_measuring_cable_pin,False)
                        continue
                        
                    time.sleep(20)
                    answer30 = tes.getData()
                    #print("answer30 from tes: ",answer30)
                    #fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") + " answer30 from tes: " + str(answer30))
                    time.sleep(30)
                    answer60 = tes.getData()
                    #print("answer60 from tes: ",answer60)
                    #fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " answer60 from tes: " + str(answer60))
                    oldValuekOm0 = valuekOm0
                    oldValuekOm30 = valuekOm30
                    oldValuekOm60 = valuekOm60
                    try:
                        valuekOm0 = getValueFromAnswer(answer0, dataWorker.getRTamu())
                        if valuekOm0 == 0:
                            valuekOm0 = dataWorker.getRMax()
                        #print("valuekOm0: ",valuekOm0, inWork)
                        #fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") + " valuekOm0: " + str(valuekOm0))
                        
                        valuekOm30 = getValueFromAnswer(answer30, dataWorker.getRTamu())
                        if valuekOm30 == 0:
                            valuekOm30 = dataWorker.getRMax()
                        #print("valuekOm30: ",valuekOm30, inWork)
                        #fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " valuekOm30: " + str(valuekOm30))
                        
                        valuekOm60 = getValueFromAnswer(answer60, dataWorker.getRTamu())
                        if valuekOm60 == 0:
                            valuekOm60 = dataWorker.getRMax()
                        #fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " valuekOm60: " + str(valuekOm60))
                        #print("valuekOm60: ",valuekOm60, inWork)
                        
                    except:
                        valuekOm0 = oldValuekOm0
                        valuekOm30 = oldValuekOm30
                        valuekOm60 = oldValuekOm60
                        #print("Error when try parse answer")
                        fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " Error when try parse answer ")
                    finally:
                        isMeasuringStoped = tes.stopMeasuringTes()
                        
                        #print("------switch Off Tes by pin------")
                        GPIO.output(supply_pin,False)
                        GPIO.output(switch_off_measuring_cable_pin,False)

                        lastEngineStopDateTime = dataWorker.getLastEngineStopDateTime()
                        isNeedSleep = False

                        if not lastEngineStopDateTime == 0:
                            dateNow = datetime.now()
                            try:
                                tSleepAfterWorkDelta = timedelta(seconds = dataWorker.getTimeSleepAfterWork())
                            except:
                                tSleepAfterWorkDelta = 0
                            isNeedSleep = isNeedSleeping(lastEngineStopDateTime, tSleepAfterWorkDelta, dateNow)
        
                        inWork = dataWorker.getInWorkEngine();
                        
                        if not isNeedSleep and not inWork:
                            dataWorker.setEngineMeasuringData(valuekOm0, valuekOm30, valuekOm60)
                                
                        #print("------sleep------")
                        #time.sleep(dataWorker.getTimeSleepAfterMeasuring())
                        sleeper.sleep(dataWorker.getTimeSleepAfterMeasuring())
            else:
                #print("------switch Off Tes by pin------")
                GPIO.output(supply_pin,False)
                GPIO.output(switch_off_measuring_cable_pin,False)
                time.sleep(5)
            

except KeyboardInterrupt:  
    print ("end KeyboardInterrupt")
        
except Exception as e:  
    print ("Error: ",sys.exc_info()[0])
    for tb in traceback.format_tb(sys.exc_info()[2]):
        print(tb + '\n')
        
finally:
    GPIO.output(block_start_engine_pin,False)
    GPIO.output(block_start_engine_warning_pin,False)
    GPIO.output(supply_pin,False)
    GPIO.output(switch_off_measuring_cable_pin,False)
    GPIO.cleanup()
    threadInWork.stop()
    threadInWork.join()
    threadModem.stop()
    threadModem.join()
    print ("Script is off")
    dataWorker.writeConfigToFile()
    dataWorker.writeLastStateToFile()
    print ("Data saved to files")
    print ("Now reboot")
    fileWorker.writeLog(datetime.now().strftime("%I:%M%p on %B %d, %Y") +  " reboot ")
    time.sleep(3)
    os.system("sudo reboot")
