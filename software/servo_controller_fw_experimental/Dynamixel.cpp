#include "Arduino.h"
#include "Dynamixel.h"

#define sendData(args)      (serialPort->write((uint8_t)args))  // Write Over Serial
#define availableData()     (serialPort->available())           // Check Serial Data Available
#define availableWrite()    (serialPort->availableForWrite())   // Bytes left to write
#define readData()          (serialPort->read())                // Read Serial Data
#define peekData()          (serialPort->peek())                // Peek Serial Data

void DynamixelInterface::begin(HardwareSerial *sPort,uint32_t bRate,uint8_t dPin,uint8_t TransmissionMode) {	
	serialPort = sPort;
	dirPin = dPin;
  pinMode(dPin,OUTPUT);
  TxMode = TransmissionMode;
  RxMode = 1 - TransmissionMode;
  //serialPort->begin(bRate);
  serialBufferLength = availableWrite();
	txDelay = (1600*9600)/bRate; 
}


// read status packet and saves to array pointed at by *data
// numBytes is the expected  number of a parameters (not including ID, length, error and checksum)
// total packet has numBytes + 6 bytes: 2 header bytes, ID, length, error and checksum 
// information saved to *data is ID, length, error, parameters (thus, numBytes+3 bytes)
// returns error code (and also saves it to object property  errorByte) 
uint8_t DynamixelInterface::receivePacket(uint8_t numBytes, uint8_t * data) {
  unsigned short timeCounter = 0;
  uint8_t i, incomingByte, sum, checksum;

  errorByte = 0; 
  sum = 0;  
  //wait until we have enough data 
  while( (availableData() <  (6 + numBytes)) && (timeCounter < TIME_OUT) ){
    timeCounter++;
    delayMicroseconds(TIME_COUNTER_DELAY);
  }
  delayMicroseconds(TIME_COUNTER_DELAY*2);

	while (availableData() > 2){
    incomingByte = readData();
    if ( (incomingByte == DXL_START) && (peekData() == DXL_START) ){
      readData();                       // Skip start Bytes
      for (i =0; i<numBytes+3; i++) {
        incomingByte = readData();
        sum+=incomingByte; 
        data[i]=incomingByte;
      } 
      //check the checksum 
      checksum = readData();
      if (checksum !=(uint8_t) ~sum ){
        // checksum failure 
        errorByte = CHECKSUM_ERROR;
#ifdef DXL_DEBUG        
        Serial.print("Incoming packet checksum: ");Serial.println(checksum, HEX);
        Serial.print("Expected checksum: ");Serial.println((uint8_t)~sum, HEX);
#endif         
      } else {
        errorByte = data[2];
      }
      return(errorByte);
    }
  }
  // if we are here, we never got a valid response
  errorByte = NO_RESPONSE_ERROR;
	return (errorByte);				
  //Serial.println("No response");	
}


//sends instruction packet
// data is a pointer to array containing length, instruction, and parameters
// it shoudl not contain start bytes, ID, or checksum 
void DynamixelInterface::sendPacket(uint8_t ID, uint8_t * data) {
  uint8_t checksum;
  uint8_t i;
  uint8_t length; // = number of parameters +2 
  //switch to Tx mode
  digitalWrite(dirPin, TxMode);
  delayMicroseconds(TIME_COUNTER_DELAY);
  //
  length = data[0];
  //compute checksum 
  checksum = ID; 
  for (i=0; i<length; i++) {
    checksum += data[i];
  }
  checksum = ~checksum;
  sendData(DXL_START);
  sendData(DXL_START);
  sendData(ID);
  for (i=0; i<length; i++) {
    sendData(data[i]);
  }
  sendData(checksum);
  while (serialPort->availableForWrite() != serialBufferLength){
    delayMicroseconds(20);  
  }
  delayMicroseconds(txDelay);//FIXME
  digitalWrite(dirPin, RxMode);
}

//reads a status returned by device
uint8_t DynamixelInterface::readStatus(){
  uint8_t rx_buffer[3];
  uint8_t error = receivePacket(0, rx_buffer);
  return(error);
}

//reads a signle-byte register from device
uint8_t DynamixelInterface::readRegisterByte(uint8_t ID, uint8_t reg){
  uint8_t packet[]={0x04, //length 
                     DXL_READ_DATA, //instruction
                     reg,
                     1};
  uint8_t rx_buffer[4];
  sendPacket(ID, packet);
  uint8_t error = receivePacket(1, rx_buffer);
#ifdef DXL_DEBUG
  if (error) {
    Serial.print("Error when reading register 0x"); Serial.println(reg, HEX);
    Serial.print("Error code: 0x"); Serial.println(error, HEX);
  }
#endif      
  if (error == 0) {
    return(rx_buffer[3]);
  } else {
    return(0);
  }

}
//reads multi-byte register from device
//numBytes is number of data bytes 
//it shoudl be less than MAX_READ_BUFFER-3
//saves to *data ID, length, error, and actual data bytes. Doesnt's save start bytes or checksum
//returns error
uint8_t DynamixelInterface::readRegister(uint8_t ID, uint8_t reg,  uint8_t numBytes, uint8_t * data ){
  uint8_t packet[]={0x04, //length 
                     DXL_READ_DATA, //instruction
                     reg,
                     numBytes};
  sendPacket(ID, packet);
  uint8_t error = receivePacket(numBytes, data);
#ifdef DXL_DEBUG
  if (error) {
    Serial.print("Error when reading register 0x"); Serial.println(reg, HEX);
    Serial.print("Error code: 0x"); Serial.println(error, HEX);
  }
#endif     
  return(error);
}

//writes a single-byte register to device
//returns error 
uint8_t DynamixelInterface::writeRegisterByte(uint8_t ID, uint8_t reg, uint8_t value){
  uint8_t packet[]={0x04, //length 
                     DXL_WRITE_DATA, //instruction
                     reg,
                     value};
  
  sendPacket(ID, packet);
  uint8_t error = readStatus();
#ifdef DXL_DEBUG
  if (error) {
    Serial.print("Error when writing register 0x"); Serial.println(reg, HEX);
    Serial.print("Error code: 0x"); Serial.println(error, HEX);
  }
#endif   
  return(error);
}
//writes a multi-byte register to device
// data should be pointer to array containing
// length, instruction, register address and actual data to write
// returns error
uint8_t DynamixelInterface::writeRegister(uint8_t ID,uint8_t * data){
  sendPacket(ID, data);
  uint8_t error = readStatus();
#ifdef DXL_DEBUG
  if (error) {
    Serial.print("Error when writing register 0x"); Serial.println(data[2], HEX);
    Serial.print("Error code: 0x"); Serial.println(error, HEX);
  }
#endif     
  return(error);
}

//pings all connected devices. 
// returns ID of device that responds. 
uint8_t DynamixelInterface::ping(){
    uint8_t packet[]={0x02, //length 
                     DXL_PING //instruction
                     };
    uint8_t rx_buffer[3];
    sendPacket(DXL_BROADCAST, packet);
    uint8_t error = receivePacket(0, rx_buffer);
    if (error) {
      return(0);
    }
    //otherwise, let's get the ID
    uint8_t ID = rx_buffer[0];
    return(ID); 
}
