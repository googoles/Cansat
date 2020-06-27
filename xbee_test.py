import serial
import RPi.GPIO as GPIO
import time
import digi.xbee.serial

if __name__ == '__main__':
    #   xbee = serial.Serial('/dev/ttyAMA0', 9600)
    xbee = serial.Serial()
    xbee.port = '/dev/ttyUSB0'
    xbee.baudrate = 9600
    xbee.timeout = 1
    xbee.writeTimeout = 1
    xbee.open()
    string = 'Hello World'
    print('Sending %s' % string)
    xbee.write('%s' % string)

while True:
    try:
        data = xbee.readline().strip()
        if data:
            print(data)
            xbee.write('Received: %s' % data)
    except KeyboardInterrupt:
        break
xbee.close()
