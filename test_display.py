import serial
import json
import time

conn = serial.Serial(port="/dev/ttyUSB0", baudrate=38400)

test_message = {
    "type": "0",
    "color": [0, 0, 255],
    "pattern": "2"
}

message = json.dumps(test_message)
num_bytes = conn.write(message)

print message
print num_bytes


num_bytes = conn.write(message)

while True:
    num_bytes = conn.write(message)
