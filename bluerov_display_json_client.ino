/*
 * JSON Formatting
 * 
 * LED type here
 * {
 *  "type":"led_state" (the 0 integer)
 *  "color":[r,g,b]  (length 3 array of integers between 0 and 255 for rgb values)
 *  "number":n (integer between 0 and LENGTH indicating LED to be turned on)
 *  "pattern":p (integer: "pulse", "slow", "fast", "solid", or "off")
 *  }
 *  
 *  LCD type here
 *  {
 *    "type":"lcd_state" (the 1 integer)
 *    "message":m (string saying what to display on the screen (remember 2x16 chars) ) 
 *  }
 *  
 *  IMPLEMENTING LED STRIP CONTROL AND LCD DISPLAY
 *  
 */

#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include <Wire.h>
#include <math.h>

String incoming_json;
StaticJsonDocument<200> doc;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
String lcd_message;

const int LED_PIN(6); // pin controlling the port
const int LED_NUM(10); // number of leds in the strip
const int BUZZER_PIN(3);

CRGB leds[LED_NUM];

bool idle_state = true;
int led_states[LED_NUM][3]; // {r, g, b} for each LED,

int pattern = 0;
int incrementer = 0;

float revamped[LED_NUM]; // revamped color intensity

int buzzer_pattern = 0;
bool buzzer_intensity = false;

void setup() {
  // initialize Serial connection
  Serial.begin(38400);
  while (!Serial) {
    continue;
  }
  // initialize LED strip
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, LED_NUM);
  FastLED.setBrightness(32);
  for (int i = 0; i < LED_NUM; i++) {
    led_states[i][0] = 255; // default orange
    led_states[i][1] = 128;
    led_states[i][2] = 0;
  }
  // initialize LCD display
  lcd_message = "Idle State";
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(lcd_message);
  lcd.display();
  // initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  
  // update from JSON if necessary
  
  if (Serial.available()) {
    // get the JSON document from the Serial port
    incoming_json = Serial.readString();
    deserializeJson(doc, incoming_json);
    int message_type = doc["type"];
    
    if (message_type == 0) { // check if the message is for modifying LEDs
      int red = doc["color"][0];
      int green = doc["color"][1];
      int blue = doc["color"][2];
      int p = doc["pattern"];
      // update the state of nth led to [cr,cg,cb,p]
      for (int num = 0; num < LED_NUM; num++) {
        led_states[num][0] = red;
        led_states[num][1] = green;
        led_states[num][2] = blue;
      }
      pattern = p;
      
    } else if (message_type == 1) { // check if the message is for the LCD
      String lcd_message = doc["message"];
      Serial.println(lcd_message);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(lcd_message);
      lcd.display();
    } else if (message_type == 2) {
      int bp = doc["pattern"];
      buzzer_pattern = bp;
    }
  }

  // calculate the intensities/color for the given pattern
  int p = int(pattern);
  for (int num = 0; num < LED_NUM; num++) {
    // pulse is a sin^2 wave
    if (p == 0) {
      revamped[num] = pow(sin(PI*incrementer/512), 2.0);
    }
    // chase has one LED for every 4
    if (pattern == 1) {
      if (num % 4 == int(incrementer) % 4){
        revamped[num] = 1;
      } else {
        revamped[num] = 0;
      }
    }
    // fancy-chase is traveling pulse using gaussian distribution
    if (pattern == 2) {
      float mu = float(incrementer)/1024 * LED_NUM;
      float sigma = 1.0;
      revamped[num] = 1/(sigma*pow(2*PI, 0.5)) * exp(-0.5 * pow(((num - mu)/sigma), 2));
    }
    // two-color rainbow
    if (pattern == 3) {
      revamped[num] = 1;
      led_states[num][1] = int( 255 * pow(sin(PI*incrementer/256 + PI*num/LED_NUM), 2.0) );
      led_states[num][0] = 0;
      led_states[num][2] = int( 255 * pow(cos(PI*incrementer/256 + PI*num/LED_NUM), 2.0) );
    }
    // three-color rainbow
    if (pattern == 4) {
      float t = float(incrementer)/1024 * PI * 1.5;
      float x = 1.5 * PI * num/LED_NUM;
      revamped[num] = 1;
      if (fmod(t, 1.5*PI) < 0.5*PI) {
        led_states[num][1] = int( 255 * pow(sin(t), 2.0) );
        led_states[num][0] = int( 255 * pow(cos(t), 2.0) );
        led_states[num][2] = 0;
      } else if (fmod(t, 1.5*PI) < 1.0*PI) {
        led_states[num][1] = int( 255 * pow(sin(t), 2.0) );
        led_states[num][2] = int( 255 * pow(cos(t), 2.0) );
        led_states[num][0] = 0;
      } else {
        led_states[num][0] = int( 255 * pow(sin(t), 2.0) );
        led_states[num][2] = int( 255 * pow(cos(t), 2.0) );
        led_states[num][1] = 0;
      }
    }
    // even fancier chase
    if (pattern == 5) {
      float mu = (incrementer/32) % LED_NUM;
      float sigma = 1.0;
      float sig_1 = 0.7/(sigma*pow(2*PI, 0.5)) * exp(-0.5 * pow(((num - mu)/sigma), 2));
      float sig_2 = 0.7/(sigma*pow(2*PI, 0.5)) * exp(-0.5 * pow((((15 - num) - mu)/sigma), 2));
      revamped[num] = min(sig_1+sig_2, 1);
    }
  }

  // calculate the buzzer intensity for a given pattern
  if (buzzer_pattern == 0) {
    buzzer_intensity = false;
  } else if (buzzer_pattern == 1) {
    if (incrementer > 512) {
      buzzer_intensity = true;
    } else {
      buzzer_intensity = false;
    }
  } else if (buzzer_pattern == 2) {
    if ((incrementer > 768) || ((incrementer > 256) && (incrementer < 512))) {
      buzzer_intensity = true;
    } else {
      buzzer_intensity = false;
    }
  }
  
  //update incrementer
  incrementer += 1; // this changes the speed
  if (incrementer > 1023) {
    Serial.println(pattern);
    incrementer = 0;
  }
  
  
  // update the LED strip 
  for (int i = 0; i < LED_NUM; i++) {
    float brightness = revamped[i];
    int r = int(led_states[i][0] * brightness);
    int g = int(led_states[i][1] * brightness);
    int b = int(led_states[i][2] * brightness);
    leds[i] = CRGB(g,r,b);
  }
  FastLED.show();

  // update the buzzer
  digitalWrite(BUZZER_PIN, buzzer_intensity);

}
