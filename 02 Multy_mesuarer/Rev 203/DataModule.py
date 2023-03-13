class DataWorker():
    lastEngineStopDateTime = 0

    #RA - RegisterAddress
    configTimeSleepAfterWorkRA = 11#2byte
    configTimeSleepAfterMeasuringRA = 12#2byte
    configDeviceAddressRA = 13 #2byte
    configEngineStartBlockValueRA = 14 #2byte
    configEngineStartBlockWarningValueRA = 15 #2byte
    configOneOrTwoEngineRA = 16 #2byte
    configSoftResetRA = 17 #2byte
    configNeedMeasuringNowRA = 18 #2byte
    configREngineRA = 20 #4byte
    configRMaxRA = 22 #4byte

    configValuesDictionaryHolding = {
        configTimeSleepAfterWorkRA:10,
        configTimeSleepAfterMeasuringRA:15,
        configDeviceAddressRA: 32,
        configEngineStartBlockWarningValueRA:5000,
        configEngineStartBlockValueRA:1000,
        configREngineRA:0,
        configOneOrTwoEngineRA:1,
        configSoftResetRA:0,
        configNeedMeasuringNowRA:0,
        configRMaxRA:999999
        }

    configFirmwareRA = 136
    configCalcStatusRA = 24
    configRTamuRA = 6

    isEngineWorkRA = 8
    value0RA = 9 #4byte
    value30RA= 11 #4byte
    value60RA = 13 #4byte
    engineStartBlockedRA = 15
    engineStartBlockWarningedRA = 16
    isEngineStartFailRA = 17

    configValuesDictionaryInput = {
        configCalcStatusRA:0,
        configRTamuRA:0,
        configFirmwareRA:2,
        isEngineWorkRA: 0,
        value0RA:0,
        value30RA:0,
        value60RA:0,
        engineStartBlockedRA:0,
        engineStartBlockWarningedRA:0,
        isEngineStartFailRA:0
        }

    def __init__(self, fileWorker):
        self.fileWorker = fileWorker

    def setInWorkEngineData(self, inWork, isEngineStartBlock, isEngineStartBlockWarning, isEngineStartFail):
        isNeedSaveChanges = 0
        
        if not self.configValuesDictionaryInput[self.isEngineWorkRA] == inWork:
            isNeedSaveChanges = 1
        self.configValuesDictionaryInput[self.isEngineWorkRA] = inWork

        if not self.configValuesDictionaryInput[self.isEngineStartFailRA] == isEngineStartFail:
            isNeedSaveChanges = 1
        self.configValuesDictionaryInput[self.isEngineStartFailRA] = isEngineStartFail

        if not self.configValuesDictionaryInput[self.engineStartBlockedRA] == isEngineStartBlock:
            isNeedSaveChanges = 1
        self.configValuesDictionaryInput[self.engineStartBlockedRA] = isEngineStartBlock
        
        if not self.configValuesDictionaryInput[self.engineStartBlockWarningedRA] == isEngineStartBlockWarning:
            isNeedSaveChanges = 1
        self.configValuesDictionaryInput[self.engineStartBlockWarningedRA] = isEngineStartBlockWarning

        if isNeedSaveChanges:
            self.writeLastStateToFile()

    def setEngineMeasuringData(self, value0, value30, value60):
        self.configValuesDictionaryInput[self.value0RA] = value0
        self.configValuesDictionaryInput[self.value30RA] = value30
        self.configValuesDictionaryInput[self.value60RA] = value60
        self.writeLastStateToFile()

    def setCalcStatus(self, value):
        self.configValuesDictionaryInput[self.configCalcStatusRA] = value
        
    def setRTamu(self, value):
        self.configValuesDictionaryInput[self.configRTamuRA] = value
        self.configValuesDictionaryInput[self.configCalcStatusRA] = 0
        self.writeConfigToFile()

    def setLastEngineStopDateTime(self, value):
        self.lastEngineStopDateTime = value

    def setNeedMeasuringNow(self, value):
        self.configValuesDictionaryHolding[self.configNeedMeasuringNowRA] = value

    def getNeedMeasuringNow(self):
        return self.configValuesDictionaryHolding[self.configNeedMeasuringNowRA]

    def getLastEngineStopDateTime(self):
        return self.lastEngineStopDateTime

    def getTimeSleepAfterWork(self):
        return self.configValuesDictionaryHolding[self.configTimeSleepAfterWorkRA]

    def getTimeSleepAfterMeasuring(self):
        return self.configValuesDictionaryHolding[self.configTimeSleepAfterMeasuringRA]

    def getDeviceAddress(self):
        return self.configValuesDictionaryHolding[self.configDeviceAddressRA]

    def getCalcStatus(self):
        return self.configValuesDictionaryInput[self.configCalcStatusRA]

    def getRTamu(self):
        return self.configValuesDictionaryInput[self.configRTamuRA]

    def getREngine(self):
        return self.configValuesDictionaryHolding[self.configREngineRA]

    def getRMax(self):
        return self.configValuesDictionaryHolding[self.configRMaxRA]

    def getR30(self):
        return self.configValuesDictionaryInput[self.value30RA] 

    def getInWorkEngine(self):
        return self.configValuesDictionaryInput[self.isEngineWorkRA]

    def getEngineStartBlockWarningValue(self):
        return self.configValuesDictionaryHolding[self.configEngineStartBlockWarningValueRA]

    def getEngineStartBlockValue(self):
        return self.configValuesDictionaryHolding[self.configEngineStartBlockValueRA]

    def getIsEngineStartBlock(self):
        return self.configValuesDictionaryInput[self.engineStartBlockedRA]
    

    def writeConfigToFile(self):
        configs = {   
        'configTimeSleepAfterWorkRA': self.configValuesDictionaryHolding[self.configTimeSleepAfterWorkRA],
        'configTimeSleepAfterMeasuringRA': self.configValuesDictionaryHolding[self.configTimeSleepAfterMeasuringRA],
        'configDeviceAddressRA': self.configValuesDictionaryHolding[self.configDeviceAddressRA],
        'configEngineStartBlockValueRA': self.configValuesDictionaryHolding[self.configEngineStartBlockValueRA],
        'configEngineStartBlockWarningValueRA': self.configValuesDictionaryHolding[self.configEngineStartBlockWarningValueRA],
        'configOneOrTwoEngineRA': self.configValuesDictionaryHolding[self.configOneOrTwoEngineRA],
        'configSoftResetRA': self.configValuesDictionaryHolding[self.configSoftResetRA],
        'configREngineRA': self.configValuesDictionaryHolding[self.configREngineRA],
        'configRMaxRA': self.configValuesDictionaryHolding[self.configRMaxRA],
        'configNeedMeasuringNowRA': self.configValuesDictionaryHolding[self.configNeedMeasuringNowRA],
        
        'configCalcStatusRA': self.configValuesDictionaryInput[self.configCalcStatusRA],
        'configRTamuRA': self.configValuesDictionaryInput[self.configRTamuRA],
        'configFirmwareRA': self.configValuesDictionaryInput[self.configFirmwareRA]
        }
        self.fileWorker.updateWorkConfig(configs)
        self.fileWorker.writeToFileWorkConfig()

    def readConfigFromFile(self):
        self.readAllConfigFromFile(self.fileWorker.readConfigFromFile())

    def resetConfig(self):
        self.readAllConfigFromFile(self.fileWorker.readDefaultConfig())
        self.configValuesDictionaryHolding[self.configSoftResetRA] = 0

    def readAllConfigFromFile(self, configFromFile):
        if configFromFile is None:
            return

        try:
            self.configValuesDictionaryHolding[self.configTimeSleepAfterWorkRA] = configFromFile['configTimeSleepAfterWorkRA']
            self.configValuesDictionaryHolding[self.configTimeSleepAfterMeasuringRA] = configFromFile['configTimeSleepAfterMeasuringRA']
            self.configValuesDictionaryHolding[self.configDeviceAddressRA] = configFromFile['configDeviceAddressRA']
            self.configValuesDictionaryHolding[self.configEngineStartBlockValueRA] = configFromFile['configEngineStartBlockValueRA']
            self.configValuesDictionaryHolding[self.configEngineStartBlockWarningValueRA] = configFromFile['configEngineStartBlockWarningValueRA']    
            self.configValuesDictionaryHolding[self.configREngineRA] = configFromFile['configREngineRA']
            self.configValuesDictionaryHolding[self.configRMaxRA] = configFromFile['configRMaxRA']
            self.configValuesDictionaryHolding[self.configSoftResetRA] = configFromFile['configSoftResetRA']
            self.configValuesDictionaryHolding[self.configOneOrTwoEngineRA] = configFromFile['configOneOrTwoEngineRA']
            self.configValuesDictionaryHolding[self.configNeedMeasuringNowRA] = configFromFile['configNeedMeasuringNowRA']
            
            self.configValuesDictionaryInput[self.configCalcStatusRA] = configFromFile['configCalcStatusRA']
            self.configValuesDictionaryInput[self.configRTamuRA] = configFromFile['configRTamuRA']
            self.configValuesDictionaryInput[self.configFirmwareRA] = configFromFile['configFirmwareRA']
        except:
            print("------!Error when try read from config File!------")

    def writeLastStateToFile(self):
        lastState = {
        'isEngineWorkRA': self.configValuesDictionaryInput[self.isEngineWorkRA],
        'value0RA': self.configValuesDictionaryInput[self.value0RA],
        'value30RA': self.configValuesDictionaryInput[self.value30RA],
        'value60RA': self.configValuesDictionaryInput[self.value60RA],
        'engineStartBlockedRA': self.configValuesDictionaryInput[self.engineStartBlockedRA],
        'engineStartBlockWarningedRA': self.configValuesDictionaryInput[self.engineStartBlockWarningedRA],
        'isEngineStartFailRA': self.configValuesDictionaryInput[self.isEngineStartFailRA]
        }
        self.fileWorker.updateLastState(lastState)
        self.fileWorker.writeToFileLastState()

    def readLastStateFromFile(self):
        lastStateFromFile = self.fileWorker.readLastStateFromFile()
        if lastStateFromFile is None:
            return

        try:
            self.configValuesDictionaryInput[self.isEngineWorkRA] = lastStateFromFile['isEngineWorkRA']
            self.configValuesDictionaryInput[self.value0RA] = lastStateFromFile['value0RA']     
            self.configValuesDictionaryInput[self.value30RA] = lastStateFromFile['value30RA']
            self.configValuesDictionaryInput[self.value60RA] = lastStateFromFile['value60RA']
            self.configValuesDictionaryInput[self.engineStartBlockedRA] = lastStateFromFile['engineStartBlockedRA']
            self.configValuesDictionaryInput[self.engineStartBlockWarningedRA] = lastStateFromFile['engineStartBlockWarningedRA']
            self.configValuesDictionaryInput[self.isEngineStartFailRA] = lastStateFromFile['isEngineStartFailRA']
        except:
            print("------!Error when try read from las state File!------")



    

    
