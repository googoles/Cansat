import serial, time
SERIALPORT = '/dev/ttyAMA0'
BAUDRATE = 115200

ser = serial.Serial(SERIALPORT,BAUDRATE)

#ser.bytesize = serial.EIGHTBITs

ser.timeout = 0.1

print('Starting up Serial Monitor')

try:
    ser.open()
except Exception as e:
    print("Exception: Opening serial port: "+str(e))

if ser.isOpen():
    try:
        ser.flushInput()
        ser.flushOutput()
        time.sleep(0.1)
        while True:
            ser.write(b'1')
            print('write 1')
            response = ser.readline().decode()
            print('pi4 recv data' + str(response))
        ser.close()
    except Exception as e:
        print("Error Communication..: "+ str(e))
        ser.close()
    finally:
        ser.close()
        pass
else:
    print('Cannot open serial port"')