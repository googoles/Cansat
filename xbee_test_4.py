import serial
import time
import Xbee

SERIAL_PORT = "/dev/serial1"
BAUD_RATE = 9600

ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE)
xbee = Xbee(ser,escaped=False)



def send_data(data):
    xbee.send("tx", dest_add=b'\x00\x00', data=bytes('{}'.format(data), 'utf-8'))

xbee.halt()
ser.close()
