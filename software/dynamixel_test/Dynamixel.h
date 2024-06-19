#ifndef Dynamixel_h
#define Dynamixel_h

#define DXL_START     0xFF //header
#define DXL_BROADCAST 0xFE


// Instruction Set ///////////////////////////////////////////////////////////////
#define DXL_PING              1
#define DXL_READ_DATA         2
#define DXL_WRITE_DATA        3
#define DXL_REG_WRITE         4
#define DXL_ACTION            5
#define DXL_RESET             6
#define DXL_SYNC_WRITE        131

//errors
#define NO_RESPONSE_ERROR			0x80 //this is added - not part of dynamixel protocol
#define INSTRUCTION_ERROR			0x40
#define OVERLOAD_ERROR				0x20
#define CHECKSUM_ERROR				0x10
#define RANGE_ERROR					  0x08
#define OVERHEATING_ERROR			0x04
#define ANGLE_LIMIT_ERROR	    0x02	
#define INPUT_VOLTAGE_ERROR	  0x01

//timing 
#define TIME_COUNTER_DELAY    600 //microseconds - waiting period 
#define TIME_OUT              50  //maximal number of repeats of delay 



#include <HardwareSerial.h>
class DynamixelInterface {
  private: 
    HardwareSerial *serialPort;
    uint8_t dirPin;
    uint8_t TxMode; //to which value should dirPin be set for transmission?
    uint8_t RxMode;
    uint32_t serialBufferLength;
    uint32_t txDelay;

  public: 
    uint8_t errorByte; //to save error of incoming communication
    void begin(HardwareSerial *sPort, uint32_t bRate,uint8_t dPin, uint8_t TransmissionMode);
    uint8_t receivePacket(uint8_t numBytes, uint8_t * data);
    void sendPacket(uint8_t ID, uint8_t * data);
    uint8_t readStatus();
    uint8_t ping();
    uint8_t readRegisterByte(uint8_t ID, uint8_t reg);
    uint8_t readRegister(uint8_t ID, uint8_t reg,  uint8_t numBytes, uint8_t * data );
    uint8_t writeRegister(uint8_t ID,uint8_t * data);
    uint8_t writeRegisterByte(uint8_t ID, uint8_t reg, uint8_t value);
};



#endif
