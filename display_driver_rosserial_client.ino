/*
 *  BLUEROV DISPLAY DRIVER CLIENT
 *  gets ROS messages over serial connection
 *  translates them to LED strip/LCD display
 */

#include <ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include <Wire.h>
#include <math.h>

ros::NodeHandle nh;

/*
 *  HELPER VARIABLES FOR LED STRIP
 */

const int LED_PIN(6);
const int LED_NUM(16);

CRGB leds[LED_NUM];

// controlled from ROS node
int color[3] = {255, 128, 0};
int pattern = 0;
// states of the led strip
int r;
int g;
int b;

int led_colors[LED_NUM][3]; // {r,g,b} for each LED
int led_brightness[LED_NUM]; // a for each LED

/*
 *  HELPER VARIABLES FOR LCD DISPLAY
 */
 
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); 
String message = "hello";

/*
 *  CALLBACKS FOR INTERFACING WITH MESSAGES FROM SERIAL
 */

// callback: update the color from the message published over serial
void update_color( std_msgs::ColorRGBA& led_color) {
  r = int(led_color.r * 255.0);
  g = int(led_color.g * 255.0);
  b = int(led_color.b * 255.0);
}

// callback: update the pattern from the message published over serial
void update_pattern( std_msgs::Int32& led_pattern) {
  pattern = int(led_pattern.data);
}

// callback: update the message from the message published over serial
void update_message( std_msgs::String& lcd_message) {
  String message(lcd_message.data);
}

ros::Subscriber<std_msgs::ColorRGBA> color_sub("led_strip_color", update_color);
ros::Subscriber<std_msgs::Int32> pattern_sub("led_strip_pattern", update_pattern);
ros::Subscriber<std_msgs::String> message_sub("lcd_message", update_message);

void setup() {
  Serial.begin(115200);
  // LED strip setup
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, LED_NUM);
  FastLED.setBrightness(32);
  // LCD display setup
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(message);
  lcd.display();
  // node handle setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(color_sub);
  nh.subscribe(pattern_sub);
  nh.subscribe(message_sub);
}

void loop() {
  nh.spinOnce();
  // update the colors on the LED strip
  for (int i = 0; i < LED_NUM; i++) {
    leds[i] = CRGB(g,r,b);
  }
  FastLED.show();
  // update the LCD display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(message);
  lcd.display();
  
  delay(10);
}
