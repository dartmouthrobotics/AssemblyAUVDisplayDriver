#!/usr/bin/env python3

import rospy
import pyserial

from std_msgs.msg import ColorRGBA, Int8, String

global strip_color = [255,128,0]
global strip_pattern = 2
global lcd_message = "idle state"
global buzzer_pattern = 0

def callback_strip_color(data):
	strip_color = [int(data.r), int(data.g), int(data.b)]
	
	type = "0"
	color = str(strip_color)
	pattern = str(strip_pattern)
	command = "{type:" + type + ",color:" + color + ",pattern:" + pattern + "}"
	
def callback_strip_pattern(data):
	strip_pattern = data.data
	
	type = "0"
	color = str(strip_color)
	pattern = str(strip_pattern)
	command = "{type:" + type + ",color:" + color + ",pattern:" + pattern + "}"

def callback_display(data):
	lcd_message = data.data
	
	type = "1"
	message = str(lcd_message)
	command = "{type:" + type + ",message:" + message + "}"

def callback_buzzer(data):
	buzer_pattern = data.data
	
	type = "2"
	pattern = str(buzzer_pattern)
	command = "{type:" + type + ",pattern:" + pattern + "}"

def listener_neopixel():
	rospy.init_node('', anonymous=True)
	rospy.Subscriber("strip_color", ColorRGBA)
	rospy.Subscriber("strip_pattern", String)
	rospy.spinOnce()
	
def listener_display():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("lcd_message", String)
	rospy.spinOnce()
	
def listener_buzzer():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("buzzer_pattern")
	rospy.spinOnce()

port = serial.Serial('/dev/ttyUSB0', 38400)

if __name__ == '__main__':
	listener_strip_color()
	listener_strip_pattern()
	listener_display()
	listener_buzzer()
	
