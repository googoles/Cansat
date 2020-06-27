import serial, time
import RPi.GPIO as GPIO
from xbee import Xbee

SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 9600

def receive_data(data):
    print("Received data: {}".format(data))
    rx = data['rf_data'].decode('utf-8')


# configure the xbee and enable asynchronous mode
ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE)
xbee = XBee(ser, callback=receive_data, escaped=False)

# main loop/functionality
while True:
    try:
        # operate in async mode where all messages will go to handler
        time.sleep(0.001)
    except KeyboardInterrupt:
        break

# clean up
GPIO.cleanup()
xbee.halt()
ser.close()