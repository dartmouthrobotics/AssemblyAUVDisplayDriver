
/*
 * Message formats:
 * 
 * Each command is a comma separated list of values of the format <command_name>,<arg1>,<arg2>,...\n
 * 
 * The '\n' is the delimiter that signals the end of a command so it must be included.
 * 
 * To pass a string, enclose the string in double quotes ("). The string may not include escaped double quotes or commas. With the current implementation, the code will break if a string arg includes a comma.
 * 
 * The args for each command type must be sent in the proper order and the string value must be parse-able by the arduino string function.
 * 
 * ========================================
 * = The following commands are supported =
 * ========================================
 * 
 * BUZZER CONTROL
 * 
 * buzzer,<frequency>,<pattern>
 * frequency is the frequency in hz of the tone. time is the number of milliseconds to play the frequency for. Both must be integers.
 * Example: "buzzer,200,0\n"
 * 
 * LED CONTROL
 * 
 * led,<color_r>,<color_g>,<color_b>,<pattern_id>
 * color_r, color_g, color_b must be integers between 0 and 255. pattern id must be an integer
 * Example: "led,255,255,0,1\n"
 * 
 * LCD CONTROL
 * 
 * lcd,line_1,line_2
 * line_1 and line_2 must be a string with only ascii characters. No commas.
 * Example: "lcd,hello sam,this world\n
 */
 

#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include <Wire.h>
#include <math.h>

const String LED_COMMAND_TYPE_NAME("led");
const String LCD_COMMAND_TYPE_NAME("lcd");
const String BUZZER_COMMAND_TYPE_NAME("buzzer");

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
String lcd_message;

const int LED_PIN(6); // pin controlling the port
const int NUMBER_LEDS(10); // number of leds in the strip
const int BUZZER_PIN(3);

CRGB leds[NUMBER_LEDS];

int led_states[NUMBER_LEDS][3]; // {r, g, b} for each LED,

int LED_pattern = 0;

int incrementer = 0;

float revamped[NUMBER_LEDS]; // revamped color intensity

int buzzer_pattern = 0;
bool buzzer_intensity = false;

void setup() {
  // initialize Serial connection
  Serial.begin(9600);

  // initialize LED strip
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUMBER_LEDS);
  FastLED.setBrightness(32);
  for (int i = 0; i < NUMBER_LEDS; i++) {
    led_states[i][0] = 255; // default orange
    led_states[i][1] = 128;
    led_states[i][2] = 0;
  }
  FastLED.show();
  // initialize LCD display
  lcd_message = "Idle State";
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(":D");
  lcd.display();
  // initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

struct BuzzerCommand {
  int frequency;
  int pattern_id;
};

struct LEDCommand {  
  int color_r;
  int color_g;
  int color_b;
  int pattern_id;
};

struct LCDCommand {
  String line_1;
  String line_2;
};

struct BuzzerCommand parse_buzzer_command(String command_string) {
  struct BuzzerCommand parsed_buzzer_command;

  int first_comma = command_string.indexOf(',');
  int second_comma = command_string.indexOf(',', first_comma+1);

  String frequency = command_string.substring(first_comma+1, second_comma);
  String pattern_id = command_string.substring(second_comma+1);

  parsed_buzzer_command.frequency = frequency.toInt();
  parsed_buzzer_command.pattern_id = pattern_id.toInt();

  return parsed_buzzer_command;
}

struct LCDCommand parse_LCD_command(String command_string) {
  struct LCDCommand parsed_LCD_command;

  int first_comma = command_string.indexOf(',');
  int second_comma = command_string.indexOf(',', first_comma+1);

  parsed_LCD_command.line_1 = command_string.substring(first_comma+1, second_comma);
  parsed_LCD_command.line_2 = command_string.substring(second_comma+1);


  return parsed_LCD_command;
}

struct LEDCommand parse_LED_command(String command_string) {
  struct LEDCommand parsed_command;

  int first_comma = command_string.indexOf(',');
  int second_comma = command_string.indexOf(',', first_comma+1);
  int third_comma = command_string.indexOf(',', second_comma+1);
  int fourth_comma = command_string.indexOf(',', third_comma+1);

  parsed_command.color_r = command_string.substring(first_comma+1, second_comma).toInt();
  parsed_command.color_g = command_string.substring(second_comma+1, third_comma).toInt();
  parsed_command.color_b = command_string.substring(third_comma+1, fourth_comma).toInt();
  parsed_command.pattern_id = command_string.substring(fourth_comma+1).toInt();

  return parsed_command;
}

String get_command_name(String command_string) {
  int first_comma = command_string.indexOf(',');
  return command_string.substring(0, first_comma);
}

void loop() {
  // update from JSON if necessary
  if (Serial.available()) {
    // get the JSON document from the Serial port

    String incoming_command = Serial.readStringUntil('\n');
    incoming_command.trim();
    incoming_command.toLowerCase();

    String command_name = get_command_name(incoming_command);

    if (command_name == LED_COMMAND_TYPE_NAME) {
      struct LEDCommand led_command = parse_LED_command(incoming_command);

      // update the state of nth led to [cr,cg,cb,p]
      for (int num = 0; num < NUMBER_LEDS; num++) {
        led_states[num][0] = led_command.color_r;
        led_states[num][1] = led_command.color_g;
        led_states[num][2] = led_command.color_b;
      }
      LED_pattern = led_command.pattern_id;
    }
    else if (command_name == LCD_COMMAND_TYPE_NAME) {
      struct LCDCommand lcd_command = parse_LCD_command(incoming_command);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(lcd_command.line_1);
      lcd.setCursor(0,1);
      lcd.print(lcd_command.line_2);
      lcd.display();
    }
    else if (command_name == BUZZER_COMMAND_TYPE_NAME) {
      struct BuzzerCommand buzzer_command = parse_buzzer_command(incoming_command);
      buzzer_pattern = buzzer_command.pattern_id;
    }
    else {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Bad cmd type!");
      lcd.display();
      Serial.println("Unrecognized command");
    }
  }

  // calculate the intensities/color for the given pattern
  int p = int(LED_pattern);
  for (int num = 0; num < NUMBER_LEDS; num++) {
    // pulse is a sin^2 wave
    if (p == 0) {
      revamped[num] = pow(sin(PI*incrementer/512), 2.0);
    }
    // chase has one LED for every 4
    if (LED_pattern == 1) {
      if (num % 4 == int(incrementer) % 4){
        revamped[num] = 1;
      } else {
        revamped[num] = 0;
      }
    }
    // fancy-chase is traveling pulse using gaussian distribution
    if (LED_pattern == 2) {
      float mu = float(incrementer)/1024 * NUMBER_LEDS;
      float sigma = 1.0;
      revamped[num] = 1/(sigma*pow(2*PI, 0.5)) * exp(-0.5 * pow(((num - mu)/sigma), 2));
    }
    // two-color rainbow
    if (LED_pattern == 3) {
      revamped[num] = 1;
      led_states[num][1] = int( 255 * pow(sin(PI*incrementer/256 + PI*num/NUMBER_LEDS), 2.0) );
      led_states[num][0] = 0;
      led_states[num][2] = int( 255 * pow(cos(PI*incrementer/256 + PI*num/NUMBER_LEDS), 2.0) );
    }
    // three-color rainbow
    if (LED_pattern == 4) {
      float t = float(incrementer)/1024 * PI * 1.5;
      float x = 1.5 * PI * num/NUMBER_LEDS;
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
    if (LED_pattern == 5) {
      float mu = (incrementer/32) % NUMBER_LEDS;
      float sigma = 1.0;
      float sig_1 = 0.7/(sigma*pow(2*PI, 0.5)) * exp(-0.5 * pow(((num - mu)/sigma), 2));
      float sig_2 = 0.7/(sigma*pow(2*PI, 0.5)) * exp(-0.5 * pow((((15 - num) - mu)/sigma), 2));
      revamped[num] = min(sig_1+sig_2, 1);
    }
  }

  //test

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
    incrementer = 0;
  }
  
  // update the LED strip 
  for (int i = 0; i < NUMBER_LEDS; i++) {
    float brightness = revamped[i];
    int r = int(led_states[i][0] * brightness);
    int g = int(led_states[i][1] * brightness);
    int b = int(led_states[i][2] * brightness);
    leds[i] = CRGB(g,r,b);
  }

  FastLED.show();

  // update the buzzer
  digitalWrite(BUZZER_PIN, buzzer_intensity);

  delay(1);
}
