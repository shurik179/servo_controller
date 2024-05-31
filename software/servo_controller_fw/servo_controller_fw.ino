
#define GFXFF 1
#define GLCD  0
#define FONT2 2
#define FONT4 4
#define FONT6 6
#define FONT7 7
#define FONT8 8

#define PIN_POWER_ON 15  // LCD and battery Power Enable
#define PIN_LCD_BL 38    // BackLight enable pin

#define PIN_SERVO1_OLD 11

#define PWM_CHANNEL1 4
#define PWM_CHANNEL2 5

#define PIN_CTRL 13 //for swichin direction of hal-duplex serial; set high for TX, low for RX


#define PIN_VSENSE 3

#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_C 2
#define BUTTON_D 3
#define BUTTON_NONE -1

#define SERVO1 0
#define SERVO2 1

#define POS_NONE -1



#include "TFT_eSPI.h"
#include "lock-xbm.h" //lock logo
TFT_eSPI tft = TFT_eSPI();

TFT_eSprite left_spr = TFT_eSprite(&tft);
TFT_eSprite right_spr = TFT_eSprite(&tft);
TFT_eSprite bot_spr = TFT_eSprite(&tft);
int MIN_PULSE = 600;
int MAX_PULSE = 2400;
int current_pos[2]; //current positions of servo, ranging 0-200
uint8_t BUTTON_PINS[] = {43, 44, 21, 16}; //pins for buttons A -- D
uint8_t POT_PINS[] = {1,2};      //pins for potentiometers
uint8_t SERVO_PINS[] = {17, 12}; //pins for servos 
char BUTTON_NAMES[] = {'A', 'B', 'C', 'D'};
int long_press_button = BUTTON_NONE; // see function update_buttons below
int press_release_button = BUTTON_NONE; 
int saved_positions[4][2]; //  first index is button, second - servo 
int button_lock = BUTTON_NONE; // to keep track of button currently used to save posiiton
#define DEBOUNCE_TIMEOUT  200 //200 ms
uint8_t BUTTON_LAST_STATE[4]; //buttons state (LOW/HIGH)
uint32_t BUTTON_LAST_CHANGE[4]; //time of last update in ms



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

void read_pot(uint8_t servo) {
  int32_t raw_read;
  uint32_t pos;
  raw_read = analogRead(POT_PINS[servo]) - 8;
  if (raw_read < 0) raw_read = 0;
  if (raw_read > 4080) raw_read = 4080; //now  ranges between 0-4080
  pos = (raw_read*200)/4080;
  current_pos[servo]=pos;
}

//reads all buttons and sets global variables
// LONG_PRESS_BUTTON: contains index of button that has been pressed for more that 3 seconds
// PRESSED_RELEASED_BUTTON: index of button that has been pressed (for short duration) and released

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
void set_servo(uint8_t servo, TFT_eSprite * spr, int override = POS_NONE){
  spr->fillSprite(TFT_BLUE);
  float pos_f = current_pos[servo]*0.005;//rescale to 0.0 - 1.0
  if (override != POS_NONE) {
    pos_f = override*0.005;
    spr->drawXBitmap(4, 4, lock, lockWidth, lockHeight, TFT_BLUE, TFT_RED);
  }
  int pulse = MIN_PULSE + pos_f*(MAX_PULSE-MIN_PULSE);
  spr->drawString(String(pulse),80, 5, GFXFF);
  spr->drawString(String(pos_f, 3),80, 55, GFXFF);
 
  if (servo == SERVO1) {
    spr->pushSprite(0,28);
    ledcWrite(PWM_CHANNEL1, (pulse<<14)/20000 );
  } else {
    spr->pushSprite(165,28);
    ledcWrite(PWM_CHANNEL2, (pulse<<14)/20000 );
  }
}


void save_positions(int button){
    if (button == BUTTON_NONE) {return;}
    uint8_t pin = BUTTON_PINS[button];
    String button_name = String(BUTTON_NAMES[button]);
    
    saved_positions[button][SERVO1]=current_pos[SERVO1]; //button D only saves position for servo 2
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
    
    bot_spr.fillSprite(TFT_BLACK);
    bot_spr.setFreeFont(&FreeSansBold18pt7b);
    bot_spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    bot_spr.pushSprite(0,135);
}

void setup() {
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
  pinMode(PIN_SERVO1_OLD, INPUT);
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
  bot_spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  bot_spr.setTextDatum(TL_DATUM); //top left
  bot_spr.setFreeFont(&FreeSansBold18pt7b);
  bot_spr.pushSprite(0,135);
  //initial user input
  read_pot(1);
  tft.drawString("Press button to select mode: ", 5,20, GFXFF);
  tft.drawString("A: range 800 - 2200", 5,45, GFXFF);
  tft.drawString("B: range 600 - 2400 (default)", 5,70, GFXFF);
  tft.drawString("C: range 500 - 2500", 5,95, GFXFF);
  int choice = wait_for_button(10.0);
  switch (choice) {
    case BUTTON_NONE:
    case BUTTON_B:
      MIN_PULSE = 600;
      MAX_PULSE = 2400;
      break;
    case BUTTON_A:
      MIN_PULSE = 800;
      MAX_PULSE = 2200;
      break;
    case BUTTON_C:
      MIN_PULSE = 500;
      MAX_PULSE = 2500;
      break;
  }


  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("Range set: "+String(MIN_PULSE)+" - "+String(MAX_PULSE), 10,80, GFXFF);
  delay(1000);
  //reset font and color
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(&FreeSans9pt7b);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Range: "+String(MIN_PULSE)+" - "+String(MAX_PULSE), 5,5, GFXFF);

  //get and print battery voltage 
  float bat = analogRead(PIN_VSENSE)*3.3*2/4096;
  tft.drawString("Battery: "+String(bat,1)+" V", 185,5, GFXFF);
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
      bot_spr.fillSprite(TFT_BLACK);
    } else {
      //setting lock 
      button_lock = press_release_button;
      int x = 5 + 96*press_release_button;
      bot_spr.fillSprite(TFT_BLACK);
      bot_spr.drawString(String(BUTTON_NAMES[button_lock]), x, 3);   
    }
    bot_spr.pushSprite(0,135);    
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
