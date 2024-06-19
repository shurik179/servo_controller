//version
#define FW_VERSION "1.0.1"

#include "Dynamixel.h"
DynamixelInterface servo;

//pins for hardware Serial1 
#define RX1 18
#define TX1 17
#define BAUDRATE 76800 //this is the value used by Feetech dual mode servos. 
                       //Dynamixel and other serial servos typically use 57600 or 115200
//reverse-engineered register values for smart servos, see https://github.com/shurik179/smart_servo
//they should be written to registers starting with register 0x06
uint8_t SERVO_BLOB_REG[] = {
  0x02,0x1E,0x00,0x05,0x00,0x0F,0x00,
  0x2D,0x00,0x00,0x00,0x00,0x0F,0x03,0xFC,
  0x00,0x00,0x00,0x00,0x41,0x03,0xC5,0x00,
  0x00,0x01,0xFF,0x01,0x00,0x02,0x09,0xC4,
  0x01,0xF4,0x03,0xE8,0x00,0x01,0x00,0x00,
  0x00,0x00,0x03,0xE8,0x00
};
uint8_t SERVO_BLOB_CR[] = {
  0x32,0x14,0x00,0x05,0x00,0x0A,0x00,
  0x0A,0x00,0x1E,0x00,0x00,0x00,0x03,0xFF,
  0x01,0x00,0x00,0x00,0x41,0x03,0xC5,0x01,
  0xF4,0x01,0xFF,0x00,0x00,0x02,0x06,0x0E,
  0x05,0xAA,0x03,0xE8,0x00,0x14,0x00,0x00,
  0x00,0x00,0x03,0xE8,0x00
};

// TFT font setup 
#define GFXFF 1
#define GLCD  0
#define FONT2 2
#define FONT4 4
#define FONT6 6
#define FONT7 7
#define FONT8 8


//pins
#define PIN_POWER_ON 15  // LCD and battery Power Enable
#define PIN_LCD_BL 38    // BackLight enable pin
#define PIN_CTRL 13      //for swiching direction of half-duplex serial; set high for TX, low for RX
#define PIN_VSENSE 3

uint8_t BUTTON_PINS[] = {43, 44, 21, 16}; //pins for buttons A -- D
uint8_t POT_PINS[] = {1,2};      //pins for potentiometers
uint8_t SERVO_PINS[] = {17, 12}; //pins for servos 



//PWM channels, for servos 
#define PWM_CHANNEL1 4
#define PWM_CHANNEL2 5

//button names 
#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_C 2
#define BUTTON_D 3
#define BUTTON_NONE -1
char BUTTON_NAMES[] = {'A', 'B', 'C', 'D'};
//for button debouncing 
#define DEBOUNCE_TIMEOUT  200 //200 ms

#define SERVO1 0
#define SERVO2 1

#define POS_NONE -1



#include "TFT_eSPI.h"
#include "lock-xbm.h" //lock icon
TFT_eSPI tft = TFT_eSPI();
// sprites 
TFT_eSprite left_spr = TFT_eSprite(&tft);
TFT_eSprite right_spr = TFT_eSprite(&tft);
TFT_eSprite bot_spr = TFT_eSprite(&tft);
TFT_eSprite battery_spr = TFT_eSprite(&tft); // for battery voltage

int min_pulse = 600;
int max_pulse = 2400;
int current_pos[2]; //current positions of servo, ranging 0-200
int long_press_button = BUTTON_NONE; // see function update_buttons below
int press_release_button = BUTTON_NONE; 
int saved_positions[4][2]; //  first index is button, second - servo 
int button_lock = BUTTON_NONE; // to keep track of button currently used to save posiiton
uint8_t BUTTON_LAST_STATE[4]; //buttons state (LOW/HIGH)
uint32_t BUTTON_LAST_CHANGE[4]; //time of last update in ms
uint32_t battery_last_update = 0; //time of last battery voltage update in ms

/*
 * BEGINNING OF FUNCTIONS
 */


//waits until one button is pressed and returns index of that button (0-3)
//timeout is time in seconds; if during this time no button was pressed, return BUTTON_NONE ( -1)
int wait_for_button(float timeout = 5.0){
  uint32_t end_time = millis() + 1000*timeout;
  int result = BUTTON_NONE;
  while ( (result == BUTTON_NONE) && (millis() < end_time) ) {
    for (int i = 0; i<4; i++) {
      if (digitalRead(BUTTON_PINS[i]) == LOW) {
        result = i;
      }
    }
  }
  return result;
}

//programs servo to continuous rotation or regular rotation
void program_servo(){
  bool CR = false;
  uint8_t error = 0;
  uint8_t servoID = 0;
  uint8_t * blob; //pointer to binary blob - register values; either SERVO_BLOB_CR or SERVO_BLOB_REG
  uint8_t temp;
  uint8_t i; //index
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Checking for connected servos", 5,5, GFXFF);
  tft.drawString("(port 1 only)", 5,30, GFXFF);
  //check servos 
  Serial1.begin(BAUDRATE,SERIAL_8N1,RX1,TX1);
  servo.begin(&Serial1, BAUDRATE, PIN_CTRL, HIGH);
  delay(1500);
  servoID = servo.ping();
  if (servoID == 0){
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("No servo found", 20,50, GFXFF);
      tft.drawString("Please connect servo to port 1", 20,80, GFXFF);      
      tft.drawString("and restart the controller", 20, 110, GFXFF);
      while(1);
  }
  //otherwise, we have a servo connected 
  temp = servo.readRegisterByte(servoID, 0x15);
  if (servo.errorByte) error=servo.errorByte;
  tft.fillScreen(TFT_BLACK);
  if (temp) {
    tft.drawString("Found connected CR servo", 5,5, GFXFF);
  } else {
    tft.drawString("Found connected regular servo", 5,5, GFXFF);    
  }
  tft.drawString("Press a button to program servo", 5,30, GFXFF);
  tft.drawString("A: regular servo", 5,55, GFXFF);
  tft.drawString("D: continuous rotation", 5,80, GFXFF);
  
  int choice = wait_for_button(1000.0);
  tft.fillScreen(TFT_BLACK);
  if (choice == BUTTON_D) {
    CR = true;
    tft.drawString("Setting servo to CR mode", 5,50, GFXFF);
    blob = SERVO_BLOB_CR;
  } else {
    tft.drawString("Setting servo to regular mode", 5,50, GFXFF);   
    blob = SERVO_BLOB_REG;
  }
  delay(1000);

  servo.writeRegisterByte(servoID, 0x34, 0x00);
  if (servo.errorByte) error=servo.errorByte;
  //key step, writing the blob of register values 
  //for redundancy, we do it one byte at a time
  for (i = 0; i < 44; i++) {
    servo.writeRegisterByte(servoID, i+6, blob[i]);
    if (servo.errorByte) error=servo.errorByte;
  }
  delay(100);
  servo.writeRegisterByte(servoID, 0x34, 0x01);  
  if (servo.errorByte) error=servo.errorByte;
  tft.fillScreen(TFT_BLACK);
  if (error) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("Communication error 0x"+String(error, HEX), 20,50, GFXFF);
    tft.drawString("Please turn the controller off and on", 20, 80, GFXFF);
    tft.drawString("to try again", 20, 110, GFXFF);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Programming complete", 20,50, GFXFF);
    tft.drawString("Please turn the controller off and on", 20, 80, GFXFF);
    tft.drawString("to continue", 20, 110, GFXFF);
  }
  while(1);
}


// reads potentiometer value, converts and saves to surrent_pos array 
void read_pot(uint8_t servo) {
  int32_t raw_read;
  uint32_t pos;
  raw_read = analogReadMilliVolts(POT_PINS[servo]);
  if (raw_read > 3050) raw_read = 3050; //ESP32-S3 max voltage is 3.1V, everythign above it doesn't matter
  pos = (raw_read*200)/3050;
  current_pos[servo]=pos;
}

//reads all buttons and sets global variables
// long_press_button: contains index of button that has been pressed for more that 3 seconds
// press_release_button: index of button that has been pressed (for short duration) and released
void update_buttons(){

  int i;
  uint32_t now = millis();
  uint8_t new_state =0;
  uint8_t prev_state = 0;
  long_press_button = BUTTON_NONE;
  press_release_button = BUTTON_NONE;
  for (i = 0; i<4; i++) {
    if (BUTTON_LAST_CHANGE[i]+DEBOUNCE_TIMEOUT < now){
      //last change was more than timeout ago
      prev_state = BUTTON_LAST_STATE[i];
      new_state = digitalRead(BUTTON_PINS[i]);
      //check for long press 
      if ((now - BUTTON_LAST_CHANGE[i]>2000) && prev_state == LOW ) {
        long_press_button = i;
        BUTTON_LAST_CHANGE[i] = now; //hackish way to make sure we only set LONG_PRESS once 
        BUTTON_LAST_STATE[i] = new_state;
      } else if (prev_state == LOW && new_state == HIGH) {
        press_release_button = i;
        BUTTON_LAST_CHANGE[i] = now; 
        BUTTON_LAST_STATE[i] = new_state;
      } else if (prev_state != new_state) {
        BUTTON_LAST_CHANGE[i] = now; 
        BUTTON_LAST_STATE[i] = new_state;
      }
    }
  }
}


//sets the servo value and updates the screen
// by default, uses current_pos array
// if override value is porvided, uses that instead (values should be integer between 0 -200)
void set_servo(uint8_t servo, TFT_eSprite * spr, int override = POS_NONE){
  spr->fillSprite(TFT_BLUE);
  if (override != POS_NONE) {
    current_pos[servo] = override;
    spr->drawXBitmap(4, 4, lock, lockWidth, lockHeight, TFT_BLUE, TFT_RED);//draw lock icon 
  }
  float pos_f = current_pos[servo]*0.005;//rescale to 0.0 - 1.0
  int pulse = min_pulse + pos_f*(max_pulse-min_pulse);
  spr->drawString(String(pulse),80, 5, GFXFF);
  spr->drawString(String(pos_f, 3),80, 55, GFXFF);
 
  if (servo == SERVO1) {
    spr->pushSprite(0,28);
    ledcWrite(PWM_CHANNEL1, (pulse<<14)/20000 ); //frequency is 50 Hz, so a cycle is 20 000 us and duty cycle fraction is (pulse_width/20 000). Resolution is 14 bits
  } else {
    spr->pushSprite(165,28);
    ledcWrite(PWM_CHANNEL2, (pulse<<14)/20000 );
  }
}

void show_locked_button(){
    bot_spr.setFreeFont(&FreeSansBold18pt7b);
    bot_spr.setTextColor(TFT_RED, TFT_BLACK);
    bot_spr.fillSprite(TFT_BLACK);
    if (button_lock != BUTTON_NONE){
      int x = 5 + 96*button_lock;
      bot_spr.drawString(String(BUTTON_NAMES[button_lock]), x, 3);   
    }
    bot_spr.pushSprite(0,135);    
}

//shows battery voltage 
void show_battery(){
  battery_spr.fillSprite(TFT_BLACK);
  float bat = analogReadMilliVolts(PIN_VSENSE)*2.0/1000.0;
  battery_spr.drawString("Battery: "+String(bat,1)+" V", 0,5, GFXFF);
  battery_spr.pushSprite(185,0);
  battery_last_update = millis();
}

//saves current potentiometer positions to a button 
void save_positions(int button){
    if (button == BUTTON_NONE) {return;}
    uint8_t pin = BUTTON_PINS[button];
    String button_name = String(BUTTON_NAMES[button]);
    
    saved_positions[button][SERVO1]=current_pos[SERVO1]; 
    saved_positions[button][SERVO2]=current_pos[SERVO2];
    bot_spr.fillSprite(TFT_WHITE);
    bot_spr.setTextColor(TFT_RED, TFT_WHITE);
    bot_spr.setFreeFont(&FreeSans12pt7b);
    bot_spr.drawString("Position saved: button "+button_name, 10,10, GFXFF);
    bot_spr.pushSprite(0,135);
    //wait until button is released
    while (digitalRead(pin) == LOW); 
    BUTTON_LAST_STATE[button] = HIGH;
    BUTTON_LAST_CHANGE[button] = millis();
    delay(200);
    show_locked_button();
}

void setup() {
  Serial.begin(57600);
  //set up pins  modes
  pinMode(PIN_POWER_ON, OUTPUT);  //enables the LCD and to run on battery
  pinMode(PIN_LCD_BL, OUTPUT);    // BackLight enable pin
  delay(100);
  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);
  //user buttons 
  for (int i = 0; i<4; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
  //pots and servos
  for (int i = 0; i<2; i++){
    pinMode(POT_PINS[i], INPUT);
    pinMode(SERVO_PINS[i], OUTPUT);
  }
  //misc
  pinMode(PIN_VSENSE, INPUT);
  pinMode(PIN_CTRL, OUTPUT);
  digitalWrite(PIN_CTRL, HIGH);// set direction to TX
  // PWM channels for servo control
  ledcSetup(PWM_CHANNEL1, 50, 14); //frequency 50 hz, 14 bit resolution
  ledcAttachPin(SERVO_PINS[SERVO1], PWM_CHANNEL1); //attach pin to channel
  ledcSetup(PWM_CHANNEL2, 50, 14); //frequency 50 hz, 14 bit resolution
  ledcAttachPin(SERVO_PINS[SERVO2], PWM_CHANNEL2); //attach pin to channel
  //Serial.begin(115200);  // be sure to set USB CDC On Boot: "Enabled"
  delay(100);
  // Initialize TFT display
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM); //top left
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(&FreeSans12pt7b);

  //create sprites
  // left - for servo 1
  left_spr.createSprite(155, 100); //width = 155, height = 100
  left_spr.setTextSize(1);
  left_spr.fillSprite(TFT_BLUE);
  left_spr.setTextColor(TFT_YELLOW, TFT_BLUE);
  left_spr.setTextDatum(TC_DATUM); //top center
  left_spr.setFreeFont(&FreeSansBold24pt7b);
  //right
  right_spr.createSprite(155, 100);
  right_spr.fillSprite(TFT_BLUE);
  right_spr.setTextColor(TFT_WHITE, TFT_BLUE);
  right_spr.setTextDatum(TC_DATUM); //top center
  right_spr.setFreeFont(&FreeSansBold24pt7b);
  //bottom 
  bot_spr.createSprite(320, 35);
  bot_spr.fillSprite(TFT_BLACK);
  bot_spr.setTextColor(TFT_RED, TFT_BLACK);
  bot_spr.setTextDatum(TL_DATUM); //top left
  bot_spr.setFreeFont(&FreeSansBold18pt7b);
  bot_spr.pushSprite(0,135);
  //battery 
  battery_spr.createSprite(135, 25);
  battery_spr.fillSprite(TFT_BLACK);
  battery_spr.setTextColor(TFT_WHITE, TFT_BLACK);
  battery_spr.setTextDatum(TL_DATUM); //top left
  battery_spr.setFreeFont(&FreeSans9pt7b);
  battery_spr.pushSprite(185,0);
  
  //initial user input
  read_pot(1);
  tft.drawString("Press button to select mode: ", 5,20, GFXFF);
  tft.drawString("A: range 800 - 2200", 5,45, GFXFF);
  tft.drawString("B: range 600 - 2400 (default)", 5,70, GFXFF);
  tft.drawString("C: range 500 - 2500", 5,95, GFXFF);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("D: program servo", 5,120, GFXFF);
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.setFreeFont(&FreeSans9pt7b);
  tft.drawString("FW version " + String(FW_VERSION), 70,150, GFXFF);

  int choice = wait_for_button(10.0);
  switch (choice) {
    case BUTTON_NONE:
    case BUTTON_B:
      min_pulse = 600;
      max_pulse = 2400;
      break;
    case BUTTON_A:
      min_pulse = 800;
      max_pulse = 2200;
      break;
    case BUTTON_C:
      min_pulse = 500;
      max_pulse = 2500;
      break;
    case BUTTON_D:
      program_servo();
      break;  
    }

  tft.setFreeFont(&FreeSans12pt7b);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("Range set: "+String(min_pulse)+" - "+String(max_pulse), 10,80, GFXFF);
  delay(1000);
  //reset font and color
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(&FreeSans9pt7b);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Range: "+String(min_pulse)+" - "+String(max_pulse), 5,5, GFXFF);
  //delay(2000);
  show_battery();
  //reset button change times 
  for (int button =0; button<4;button++){
    BUTTON_LAST_STATE[button] = HIGH;
    BUTTON_LAST_CHANGE[button] = millis();
  }

}

void loop() {
  //get the buttons readings
  update_buttons();
  if (long_press_button != BUTTON_NONE) {
    save_positions(long_press_button);
  }
  if (press_release_button != BUTTON_NONE) {
    //we had pressed a button to move servo(s) to saved position
    if (button_lock == press_release_button) {
      // we pressed currently active button again - need to release lock 
      button_lock = BUTTON_NONE;
    } else {
      //setting lock 
      button_lock = press_release_button;
    }
    show_locked_button();    
  }
  //update battery reading if necessary
  if (millis()-battery_last_update > 1000) { //update once a second
    show_battery();
  }
  read_pot(SERVO1);  
  read_pot(SERVO2);
  if (button_lock == BUTTON_NONE || button_lock == BUTTON_D) {
    set_servo(SERVO1,  &left_spr); 
  } else{
    set_servo(SERVO1,  &left_spr, saved_positions[button_lock][SERVO1]); //override potentiometer value
  }
  if (button_lock == BUTTON_NONE || button_lock == BUTTON_A) {
    set_servo(SERVO2,  &right_spr);
  } else{
    set_servo(SERVO2,  &right_spr, saved_positions[button_lock][SERVO2]); //override potentiometer value
  }
  delay(20);
}
