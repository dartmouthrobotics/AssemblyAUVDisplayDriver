#!usr/bin/env python

# BluerovDisplayMessage
#   type (String)
#     data (string)
#   frequency (Int8)
#     data (int)
#   interval (Int8)
#     data (int)
#   color (ColorRGBA)
#     r (float)
#     g (float)
#     b (float)
#   pattern (String)
#     data (string)
#   first (String)
#     data (string)
#   second (String)
#     data (string)

import rospy
import serial

from std_msgs.msg import String, Int8, Float32, ColorRGBA
from msg/custom.msg import BluerovDisplayMessage

buzzer_frequency = 440 # Hz
buzzer_interval = 0 # ms

led_color = ColorRGBA()
led_color.r = 0
led_color.g = 255
led_color.b = 0
led_color.a = 255
led_pattern = "pulse"

lcd_line_1 = "Connection"
lcd_line_2 = "Successful"

def callback(data):
    if data.type.data == "buzzer":
        # check to see if a value needs to be changed
        if data.frequency.data != buzzer_frequency or data.interval.data != buzzer_interval:
            # set new values for frequency and interval
            buzzer_frequency = data.frequency.data
            buzzer_interval = data.interval.data
            message = b"buzzer," + buzzer_frequency + "," + buzzer_interval + "\n"
            # send changes via serial port
            conn.write(message)
    if data.type.data == "led":
        # check to see if a value needs to be changed
        if data.color.r != led_color_r or data.color.g != led_color_g or data.color.b != led_color_g or data.pattern.data != led_pattern:
            # set new values for frequency and interval
            led_color_r = data.color.r
            led_color_g = data.color.g
            led_color_b = data.color.b
            led_color_pattern = data.pattern.data
            message = b"led," + str(int(led_color_r)) + "," + str(int(led_color_g)) + "," + str(int(led_color_b)) + "," + str(led_pattern) + "\n"
            # send changes via serial port
            conn.write(message)
    if data.type.data == "lcd":
        # checks to se if a value needs to be changed
        if data.first.data != lcd_line_1 or data.second.data != lcd_line_2:
            # set new values for frequency and interval
            lcd_line_1 = data.first.data
            lcd_line_2 = data.second.data
            message = b"lcd," + str(lcd_line_1) + "," + str(lcd_line_2) + "\n"
            # send changes via serial port
            conn.write(message)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("bluerov_display_topic", BluerovDisplayMessage, callback)
    rospy.spin()

conn = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
conn.open()

if __name__ == '__main__':
    listener()
