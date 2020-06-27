# AssemblyAUVDisplayDriver
Arduino display driver for the assembly auv's tiny tube display system

<p>The display driver is an Arduino (Uno currently, may change to Nano) controlling a strip of WS2812B individually addressable RGB light emitting diodes and a 16x2 liquid crystal display display with an I2C backpack for ease of wiring.  Both the LCD display and the LED strip receive 5V input voltage.  The LCD display draws 150 mA and each LED theoretically draws 50 mA of current at maximum brightness (although I suspect that in practice it may be lower) according to the specifications.</p>
<p>The display driver communicates with the main computer via the rosserial protocol.  The ROS-side interface runs rosserial_python, creating a node on the main computer that connects the serial protocol to ROS itself.  This allows standard ROS serialized messages from multiple topics to be sent to the display driver over the serial port.  On the client side, the display driver uses the rosserial_arduino library, which runs rosserial_client.</p>
<p>A script runs on the main computer and publishes to the rostopics led_strip_color (std_msgs/ColorRGBA), led_strip_pattern (std_msgs/Int32), and lcd_display_message (std_msgs/String) based on the current state of the robot.  The display driver subscribes to these topics and updates the LED strip and LCD display accordingly.</p>

In order to setup and run everything correctly, do the following:
* Start subscribing by uploading the program to the display driver (&#8984; U in the Arduino IDE)
* Connect the display driver to the main computer with a USB cable and connect to the power source
* We need roscore running before we can do anything else, so do `roscore`
* Initialize the node using `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200`
* Start publishing with the script using `rosrun` (make sure to use the correct project and script names)
