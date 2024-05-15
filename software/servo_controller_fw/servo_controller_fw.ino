
#define GFXFF 1
#define GLCD  0
#define FONT2 2
#define FONT4 4
#define FONT6 6
#define FONT7 7
#define FONT8 8

#define PIN_POWER_ON 15  // LCD and battery Power Enable
#define PIN_LCD_BL 38    // BackLight enable pin (see Dimming.txt)

#define PIN_SERVO1 11
#define PIN_SERVO2 12
#define PWM_CHANNEL1 4
#define PWM_CHANNEL2 5

#define PIN_POT1 1
#define PIN_POT2 2

#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();

TFT_eSprite left_spr = TFT_eSprite(&tft);
TFT_eSprite right_spr = TFT_eSprite(&tft);

int MIN_PULSE = 600;
int MAX_PULSE = 2400;
int pos_int = 0;
float pos1, pos2; 
unsigned int pulse1, pulse2;

uint32_t time_sec;
void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);  //enables the LCD and to run on battery
  pinMode(PIN_LCD_BL, OUTPUT);    // BackLight enable pin
  delay(100);
  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);
  ledcSetup(PWM_CHANNEL1, 50, 14); //frequency 50 hz, 14 bit resolution
  ledcAttachPin(PIN_SERVO1, PWM_CHANNEL1); //attach pin to channel
  ledcSetup(PWM_CHANNEL2, 50, 14); //frequency 50 hz, 14 bit resolution
  ledcAttachPin(PIN_SERVO2, PWM_CHANNEL2); //attach pin to channel
  Serial.begin(115200);  // be sure to set USB CDC On Boot: "Enabled"
  //(Serial print slows progres bar Demo)
  delay(100);
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(&FreeSans9pt7b);

  //create sprites 
  left_spr.createSprite(155, 105); //width = 155, height = 120
  left_spr.setTextSize(1);
  left_spr.fillSprite(TFT_BLUE);
  left_spr.setTextColor(TFT_YELLOW, TFT_BLUE);
  left_spr.setTextDatum(TC_DATUM); //top center
  left_spr.setFreeFont(&FreeSansBold24pt7b);
  //right 
  right_spr.createSprite(155, 105); //width = 155, height = 120
  right_spr.fillSprite(TFT_BLUE);
  right_spr.setTextColor(TFT_WHITE, TFT_BLUE);
  right_spr.setTextDatum(TC_DATUM); //top center
  right_spr.setFreeFont(&FreeSansBold24pt7b);

   tft.drawString("Range: "+String(MIN_PULSE)+" - "+String(MAX_PULSE), 160,150, GFXFF);

}

void loop() {
  // put your main code here, to run repeatedly:
 
   pos1 = read_pot(PIN_POT1)*0.005;
   pulse1 = MIN_PULSE + pos1*(MAX_PULSE-MIN_PULSE);   
  left_spr.fillSprite(TFT_BLUE);
  left_spr.drawString(String(pulse1),80, 5, GFXFF);
  left_spr.drawString(String(pos1, 3),80, 55, GFXFF);
  left_spr.pushSprite(0, 40);

  //convert pulse width to duty cycle 
  // cycle = 20 ms = 20 000 us
  // duty fraction = pulse /20 000
  // duty ticks = duty fraction * 2^14 = (pulse<<14)/20000
  ledcWrite(PWM_CHANNEL1, (pulse1<<14)/20000 );
  //
   pos2 = read_pot(PIN_POT2)*0.005;
   pulse2 = MIN_PULSE + pos2*(MAX_PULSE-MIN_PULSE);   
  right_spr.fillSprite(TFT_BLUE);
  right_spr.drawString(String(pulse2),80, 5, GFXFF);
  right_spr.drawString(String(pos2, 3),80, 55, GFXFF);
  right_spr.pushSprite(165, 40);
    ledcWrite(PWM_CHANNEL2, (pulse2<<14)/20000 );
  delay(100);
}

int read_pot(uint8_t pin) {
  int32_t raw_read;
    raw_read = analogRead(pin) - 8;
  if (raw_read < 0) raw_read = 0;
  if (raw_read > 4080) raw_read = 4080; //now  ranges between 0-4080
  return ((raw_read*200)/4080); 
}
