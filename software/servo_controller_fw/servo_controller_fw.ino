
#define GFXFF 1
#define GLCD  0
#define FONT2 2
#define FONT4 4
#define FONT6 6
#define FONT7 7
#define FONT8 8

#define PIN_POWER_ON 15  // LCD and battery Power Enable
#define PIN_LCD_BL 38    // BackLight enable pin

#define PIN_SERVO1 11
#define PIN_SERVO2 12
#define PWM_CHANNEL1 4
#define PWM_CHANNEL2 5

#define PIN_POT1 1
#define PIN_POT2 2

#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_C 2
#define BUTTON_D 3
#define BUTTON_NONE -1

#define SERVO1 0
#define SERVO2 1


#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();

TFT_eSprite left_spr = TFT_eSprite(&tft);
TFT_eSprite right_spr = TFT_eSprite(&tft);

int MIN_PULSE = 600;
int MAX_PULSE = 2400;
int pos_int = 0;
float pos1, pos2;
unsigned int pulse1, pulse2;
uint8_t BUTTON_PINS[] = {43, 44, 21, 16}; //pins for buttons A -- D
char BUTTON_NAMES[] = {'A', 'B', 'C', 'D'};
int LONG_PRESS_BUTTON = BUTTON_NONE; // see function update_buttons below
int PRESSED_RELEASED_BUTTON = BUTTON_NONE; 
#define DEBOUNCE_TIMEOUT  200 //200 ms




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

int read_pot(uint8_t pin) {
  int32_t raw_read;
    raw_read = analogRead(pin) - 8;
  if (raw_read < 0) raw_read = 0;
  if (raw_read > 4080) raw_read = 4080; //now  ranges between 0-4080
  return ((raw_read*200)/4080);
}

//reads all buttons and sets global variables
// LONG_PRESS_BUTTON: contains index of button that has been pressed for more that 3 seconds
// PRESSED_RELEASED_BUTTON: index of button that has been pressed (for short duration) and released

void update_buttons(){
  static uint8_t BUTTON_LAST_STATE[4]; //buttons state (LOW/HIGH)
  static uint32_t BUTTON_LAST_CHANGE[4]; //time of last update in ms
  int i;
  uint32_t now = millis();
  uint8_t new_state =0;
  uint8_t prev_state = 0;
  LONG_PRESS_BUTTON = BUTTON_NONE;
  PRESSED_RELEASED_BUTTON = BUTTON_NONE;
  for (i = 0; i<4; i++) {
    if (BUTTON_LAST_CHANGE[i]+DEBOUNCE_TIMEOUT < now){
      //last change was more than timeout ago
      prev_state = BUTTON_LAST_STATE[i];
      new_state = digitalRead(BUTTON_PINS[i]);
      //check for long press 
      if ((now - BUTTON_LAST_CHANGE[i]>3000) && prev_state == LOW ) {
        LONG_PRESS_BUTTON = i;
        BUTTON_LAST_CHANGE[i] = now; //hackish way to make sure we only set LONG_PRESS once 
        BUTTON_LAST_STATE[i] = new_state;
      } else if (prev_state == LOW && new_state == HIGH) {
        PRESSED_RELEASED_BUTTON = i;
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
// pos shoudl be an integer between 0 - 200
void set_servo(uint8_t index, int pos,TFT_eSprite * spr){
  float pos_f = pos*0.005;
  int pulse = MIN_PULSE + pos_f*(MAX_PULSE-MIN_PULSE);
  spr->fillSprite(TFT_BLUE);
  spr->drawString(String(pulse),80, 5, GFXFF);
  spr->drawString(String(pos_f, 3),80, 55, GFXFF);
  if (index == SERVO1) {
    spr->pushSprite(0,40);
    ledcWrite(PWM_CHANNEL1, (pulse<<14)/20000 );
  } else {
    spr->pushSprite(165,40);
    ledcWrite(PWM_CHANNEL2, (pulse<<14)/20000 );
  }
}

void setup() {
  //set up pins  modes
  pinMode(PIN_POWER_ON, OUTPUT);  //enables the LCD and to run on battery
  pinMode(PIN_LCD_BL, OUTPUT);    // BackLight enable pin
  delay(100);
  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);
  for (int i = 0; i<4; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
  // PWM channels for servo control
  ledcSetup(PWM_CHANNEL1, 50, 14); //frequency 50 hz, 14 bit resolution
  ledcAttachPin(PIN_SERVO1, PWM_CHANNEL1); //attach pin to channel
  ledcSetup(PWM_CHANNEL2, 50, 14); //frequency 50 hz, 14 bit resolution
  ledcAttachPin(PIN_SERVO2, PWM_CHANNEL2); //attach pin to channel
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
  left_spr.createSprite(155, 105); //width = 155, height = 105
  left_spr.setTextSize(1);
  left_spr.fillSprite(TFT_BLUE);
  left_spr.setTextColor(TFT_YELLOW, TFT_BLUE);
  left_spr.setTextDatum(TC_DATUM); //top center
  left_spr.setFreeFont(&FreeSansBold24pt7b);
  //right
  right_spr.createSprite(155, 105);
  right_spr.fillSprite(TFT_BLUE);
  right_spr.setTextColor(TFT_WHITE, TFT_BLUE);
  right_spr.setTextDatum(TC_DATUM); //top center
  right_spr.setFreeFont(&FreeSansBold24pt7b);
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
  tft.drawString("Range set: "+String(MIN_PULSE)+" - "+String(MAX_PULSE), 10,800, GFXFF);
  delay(1000);
  //reset font and color
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(&FreeSans9pt7b);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Range: "+String(MIN_PULSE)+" - "+String(MAX_PULSE), 90,150, GFXFF);
}

void loop() {
  //get the buttons readings
  update_buttons();
  String button_name = "    ";
  if (PRESSED_RELEASED_BUTTON != BUTTON_NONE) {
    button_name = String(BUTTON_NAMES[PRESSED_RELEASED_BUTTON]);
    delay(500);
  }
  tft.drawString("Button pressed: "+button_name, 10,10, GFXFF);

  set_servo(SERVO1, read_pot(PIN_POT1), &left_spr);
  set_servo(SERVO2, read_pot(PIN_POT2), &right_spr);
  delay(20);
}
