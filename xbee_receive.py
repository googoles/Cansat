import serial, time
from xbee import XBee

SERIAL_PORT = 'sudo chmod a+rw /dev/ttyAMA0'
BAUD_RATE = 9600

def receive_data(data):
    print('Received data: {}'.format(data))
    rx = data['rf_data'].decode('utf-8')
    state = rx[:1]

    print(state)

ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE)
xbee = Xbee(ser, callback=receive_data, escaped=False)

while True:
    try:
        time.sleep(0.001)
    except KeyboardInterrupt:
        break

xbee.halt()
ser.close()