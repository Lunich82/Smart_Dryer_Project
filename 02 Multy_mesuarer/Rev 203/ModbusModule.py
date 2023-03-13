from crcmod import predefined
class ModbusWorker ():
    dataContainer = None
        
    """Commands"""
    commands = {
        'readHoldingRegisters': 3,
        'readInputRegisters': 4,
        'writeSingleHoldingRegister': 6,
        'writeHoldingRegisters': 16
        }

    """Code of error"""
    errors = {
        'errorNumberFunction': 1,
        'errorDataAddress': 2,
        'errorDataRequest': 3,
        'errorUnknow': 4,
        'longTimeProccessRequest': 5,
        'slaveIsBusy': 6,
        'errorRequestProgrammingFunction': 7,
        'errorParity': 8
        }

    def __init__(self,dataWorker):
        self.dataContainer = dataWorker

    def calculateCrc(self, payload):
        modbuscrc = predefined.Crc("modbus")
        modbuscrc.update(payload)
        return modbuscrc.crcValue.to_bytes(2, byteorder='little')

    def generateErrorResponse(self, address,numberFunction, numberError):
        errorNumberFunction = numberFunction | 128
        answer = bytes([address, errorNumberFunction, numberError])
        answerCrc = self.calculateCrc(answer)
        result = answer + answerCrc
        return result

    def isGoodRequestCrc(self, data):
        request = data[0:len(data)-2]              
        crc16Modem = data[len(data)-2:len(data)]             
        crc16Request = self.calculateCrc(request) 
        return crc16Modem == crc16Request
    
    def getResponse(self, data):
        response = None
        
        if not data[0] == self.dataContainer.getDeviceAddress():
            return response
        #function 03
        if data[1] == self.commands['readHoldingRegisters']:
            if len(data) == 8 and self.isGoodRequestCrc(data):
                try:
                    requestFirstRegisterAddress = int.from_bytes(data[2:4],byteorder='big')
                    requestReadRegisterCount = int.from_bytes(data[4:6],byteorder='big')
                except:
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataRequest'])
                    return response
                
                if not(requestFirstRegisterAddress in self.dataContainer.configValuesDictionaryHolding):
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataAddress'])
                    return response
                
                responseRegisterValues = b''
                index = requestFirstRegisterAddress
                while index <  requestFirstRegisterAddress + requestReadRegisterCount:
                    deterIndex = 1
                    
                    if not(index in self.dataContainer.configValuesDictionaryHolding):
                        nul = 0
                        dictVal= nul.to_bytes(2, byteorder='big')
                    else:
                        dictVal = self.dataContainer.configValuesDictionaryHolding[index]
                        
                    if not type(dictVal) is bytes:
                        if index == 20 or index == 22: 
                            deterIndex = 2
                            dictVal= round(dictVal).to_bytes(4, byteorder='big')
                        else:
                            dictVal= round(dictVal).to_bytes(2, byteorder='big')
                        
                    responseRegisterValues = responseRegisterValues + dictVal
                    index = index + deterIndex
                                                                            
                #print("responseRegisterValues",responseRegisterValues)
                response = bytes([data[0],data[1], len(responseRegisterValues)]) + responseRegisterValues
                responseCrc = self.calculateCrc(response)
                response = response + responseCrc
            else:
                print("not good data or crc16 of request")

        #function 04    
        elif data[1] == self.commands['readInputRegisters']:
            if len(data) == 8 and self.isGoodRequestCrc(data):
                try:
                    requestFirstRegisterAddress = int.from_bytes(data[2:4],byteorder='big')
                    requestReadRegisterCount = int.from_bytes(data[4:6],byteorder='big')
                except:
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataRequest'])
                    return response

                if not(requestFirstRegisterAddress in self.dataContainer.configValuesDictionaryInput):
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataAddress'])
                    return response
                
                responseRegisterValues = b''
                index = requestFirstRegisterAddress
                while index <  requestFirstRegisterAddress + requestReadRegisterCount:
                    deterIndex = 1
                    if not(index in self.dataContainer.configValuesDictionaryInput):
                        nul = 0
                        dictVal= nul.to_bytes(2, byteorder='big')
                    else:
                        dictVal = self.dataContainer.configValuesDictionaryInput[index]

                    if not type(dictVal) is bytes:
                        if index == 9 or index == 11 or index == 13: 
                            deterIndex = 2
                            dictVal= round(dictVal).to_bytes(4, byteorder='big')
                        else:
                            dictVal= round(dictVal).to_bytes(2, byteorder='big')
                        
                    responseRegisterValues = responseRegisterValues + dictVal
                    index = index + deterIndex
                                                                            
                #print("responseRegisterValues",responseRegisterValues)
                response = bytes([data[0],data[1], len(responseRegisterValues)]) + responseRegisterValues
                responseCrc = self.calculateCrc(response)
                response = response + responseCrc
            else:
                print("not good data or crc16 of request")
                
        #function 06 write 2 bytes      
        elif data[1] == self.commands['writeSingleHoldingRegister']:
            if self.isGoodRequestCrc(data):
                try:
                    requestFirstRegisterAddress = int.from_bytes(data[2:4],byteorder='big')
                except:
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataRequest'])
                    return response

                if not(requestFirstRegisterAddress in self.dataContainer.configValuesDictionaryHolding):
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataAddress'])
                    return response
                
                i = 4
                j = 6
                value = int.from_bytes(data[i:j],byteorder='big')
                #print(value)
                self.dataContainer.configValuesDictionaryHolding[requestFirstRegisterAddress] = value
                
                response = bytes([data[0],data[1]])+ data[2:4] + data[4:6]
                responseCrc = self.calculateCrc(response)
                response = response + responseCrc

                if requestFirstRegisterAddress == 17:
                    self.dataContainer.resetConfig()

                self.dataContainer.writeConfigToFile()
            else:
                print("not good data or crc16 of request")
                
        #function 16 write 4 bytes
        elif data[1] == self.commands['writeHoldingRegisters']:
            if self.isGoodRequestCrc(data):
                try:
                    requestFirstRegisterAddress = int.from_bytes(data[2:4],byteorder='big')
                    requestWriteRegisterCount = int.from_bytes(data[4:6],byteorder='big')
                    requestCountBytes = int.from_bytes(data[6:7],byteorder='big')
                except:
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataRequest'])
                    return response

                if not(requestFirstRegisterAddress in self.dataContainer.configValuesDictionaryHolding):
                    response = self.generateErrorResponse(data[0],data[1],self.errors['errorDataAddress'])
                    return response
                
                index = 0
                i = 7
                j = 11
                while requestWriteRegisterCount > 0:
                    value = int.from_bytes(data[i:j],byteorder='big')
                    self.dataContainer.configValuesDictionaryHolding[requestFirstRegisterAddress + index] = value
                    index = index + 1
                    i = i + 4
                    j = j + 4
                    requestWriteRegisterCount = requestWriteRegisterCount - 2
                
                response = bytes([data[0],data[1]])+ data[2:4] + data[4:6]
                responseCrc = self.calculateCrc(response)
                response = response + responseCrc

                if requestFirstRegisterAddress == 20:
                    self.dataContainer.setCalcStatus(1)
  
                self.dataContainer.writeConfigToFile()
                
            else:
                print("not good data or crc16 of request")
 
        else:
            response = self.generateErrorResponse(data[0],data[1],self.errors['errorNumberFunction'])

        return response
