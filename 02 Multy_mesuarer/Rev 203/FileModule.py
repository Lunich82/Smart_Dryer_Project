from pathlib import Path
class FileWorker():
    configFileName = ""
    defaultConfigFileName = ""
    lastStateFileName = ""
    logFileName = "/home/pi/Desktop/py/logs"
    workConfig = {}
    lastState = {}
    
    def __init__(self, configFileName, lastStateFileName,defaultConfigFileName):
        self.configFileName = configFileName
        self.lastStateFileName = lastStateFileName
        self.defaultConfigFileName = defaultConfigFileName

    def updateWorkConfig(self, dictionary):
        self.workConfig = dictionary

    def updateLastState(self, dictionary):
        self.lastState = dictionary
    
    def readConfigFromFile(self):
        dictionary = {}
        
        check_file = Path(self.configFileName)
        if not check_file.is_file():          
            return None
        
        with open(self.configFileName, 'r') as f:
            for line in f:
                (key, val) = line.split(":")
                dictionary[(key)] = float(val)                      
        self.workConfig = dictionary
        return dictionary

    def readLastStateFromFile(self):
        dictionary = {}
        
        check_file = Path(self.lastStateFileName)
        if not check_file.is_file():          
            return None

        with open(self.lastStateFileName, 'r') as f:
            for line in f:
                (key, val) = line.split(":")
                dictionary[(key)] = float(val)                      
        self.lastState = dictionary
        return dictionary

    def writeToFileWorkConfig(self):
        with open(self.configFileName, 'w') as f:
            for key in self.workConfig:
                f.write(key +":"+ str(self.workConfig[key])+'\n')

    def writeToFileLastState(self):
        with open(self.lastStateFileName, 'w') as f:
            for key in self.lastState:
                f.write(key +":"+ str(self.lastState[key])+'\n')

    def writeLog(self, message):
        with open(self.logFileName, 'a') as f: 
            f.write(message + '\n')

    def readDefaultConfig(self):
        dictionary = {}
        
        check_file = Path(self.defaultConfigFileName)
        if not check_file.is_file():          
            return None
        
        with open(self.defaultConfigFileName, 'r') as f:
            for line in f:
                (key, val) = line.split(":")
                dictionary[(key)] = float(val)                      
        self.workConfig = dictionary
        return dictionary
        
