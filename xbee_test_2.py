import serial

# Enable USB Communication
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=.5)

while True:
    ser.write('Hello User \r\n')  # write a Data
    incoming = ser.readline().strip()
    print('Received Data : ' + incoming)