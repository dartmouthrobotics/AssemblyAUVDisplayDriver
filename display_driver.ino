/*
 * JSON Formatting
 * 
 * LED type here
 * {
 *  "type":"led_state"
 *  "color":[r,g,b]  (length 3 array of integers between 0 and 255 for rgb values)
 *  "number":n (integer between 0 and LENGTH indicating LED to be turned on)
 *  "pattern":p (string: "pulse", "slow", "fast", "solid", or "off")
 *  }
 *  
 *  LCD type here
 *  {
 *    "type":"lcd_state"
 *    "message":m (string saying what to display on the screen (remember 2x16 chars) ) 
 *  }
 *  
 *  ONLY IMPLEMENTING LED STRIP CONTROL FOR NOW
 *  
 */

#include <ArduinoJson.h>
#include <FastLED.h>
#include <Wire.h>
#include <math.h>

#define LED_PIN 6 // pin controlling the port
#define LED_NUM 6 // number of leds in the strip

CRGB leds[LED_NUM];

String incoming_json;
StaticJsonDocument<200> doc;

bool idle_state = true;
int led_states[LED_NUM][4] = {
  {255,127,0,0},
  {255,127,0,0},
  {255,127,0,0},
  {255,127,0,0},
  {255,127,0,0},
  {255,127,0,0}
}; // {r, g, b, pattern} for each LED
int incrementer = 0;
float intensity[5] = {0,0,0,0,0};

void setup() {
  // initialize Serial connection
  Serial.begin(115200);
  while (!Serial) {
    continue;
  }
  // initialize LED strip
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, LED_NUM);
  FastLED.setBrightness(128);
}

void loop() {
  
  // update from JSON if necessary
  
  if (Serial.available()) {
    // get the JSON document from the Serial port
    incoming_json = Serial.readString();
    deserializeJson(doc, incoming_json);
    int cr = doc["color"][0];
    int cg = doc["color"][1];
    int cb = doc["color"][2];
    int n = doc["number"];
    int p = doc["pattern"];
    // update the state of led n to [cr,cg,cb,p]
    led_states[n][0] = cr;
    led_states[n][1] = cg;
    led_states[n][2] = cb;
    led_states[n][3] = p;
  }
  
  // calculate the intensities for each pattern

  // pulse is a sin^2 wave
  intensity[0] = pow(sin(incrementer*PI/512), 2.0);
  // slow blink is square wave
  if (incrementer > 511) {
    intensity[1] = 1;
  } else {
    intensity[1] = 0;
  }
  // fast blink is a square wave with doubled frequency
  if ((incrementer < 255) || (incrementer > 511 && 767 < incrementer)) {
    intensity[2] = 1;
  } else {
    intensity[2] = 0;
  }
  // solid and off intensities are constant
  intensity[3] = 1;
  intensity[4] = 0;
  //update incrementer
  incrementer++;
  if (incrementer > 1023) {
    incrementer = 0;
  }
  
  // update the display
  
  for (int i = 0; i < LED_NUM; i++) {
    float brightness = intensity[led_states[i][3]];
    int r = int(led_states[i][0] * brightness);
    int g = int(led_states[i][1] * brightness);
    int b = int(led_states[i][2] * brightness);
    leds[i] = CRGB(g,r,b);
  }
  FastLED.show();
}
