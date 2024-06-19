
/*
 * Testing dynamixel and other serial servos. 
 * The code below is an example of communication using Dynamixel protocol version 1
 * documented at https://emanual.robotis.com/docs/en/dxl/protocol1/
 * 
 */

#include "Dynamixel.h"

DynamixelInterface servo;

//pins for hardware Serial1 
#define RX1 18
#define TX1 17
//direction control pin
#define PIN_CTRL 13
#define BAUDRATE 76400 //this is the value used by Feetech dual mode servos. 
                       //Dynamixel and other serial servos typically use 57600 or 115200

uint8_t servoID;
uint8_t regValue;

void setup() {
  Serial.begin(57600);
  delay(3000);
  pinMode(PIN_CTRL, OUTPUT);
  digitalWrite(PIN_CTRL, HIGH);
  Serial1.begin(BAUDRATE,SERIAL_8N1,RX1,TX1);
  Serial.println("Servo test");
  servo.begin(&Serial1, BAUDRATE, PIN_CTRL, HIGH);
}

void loop() {
  delay(1000);
  servoID = servo.ping();
  if (servo.errorByte == 0 ) { // no error 
    Serial.print("Found servo with ID ");
    Serial.println(servoID);
    //read and print value of registers 6 -- 49 (0x31)
    for (int i = 6; i<50; i++) {
      regValue = servo.readRegisterByte(servoID, i);
      Serial.print("Register "); Serial.print(i); Serial.print(": 0x"); 
      Serial.println(regValue, HEX); 
    }
    
  } else {
    Serial.print("Error: 0x"); 
    Serial.println(servo.errorByte, HEX); 
  }
}
