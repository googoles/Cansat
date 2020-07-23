import sys
from digi.xbee.devices import *
import time
import RPi.GPIO as GPIO
sys.path.append('/home/pi/python_project/')
import smbus
import serial


# Integrated
# I need to Integrate Opencv
# Xbee Transmission Part

device = XBeeDevice("/dev/ttyUSB0",9600)
device.open()
device.set_sync_ops_timeout(100)
remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20040CA046A"))
start_time = time.time()

# GPS Setting
port = "/dev/ttyAMA0"
ser = serial.Serial(port, baudrate=9600, timeout=0.5)

# Motor Setting
PIN1 = 24
PIN2 = 23
EN1 = 18
motor_count = 0 # For Initial Start Motor
# for pwm

# def init():
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN1, GPIO.OUT)  # MOTOR 1 A,B Setup
GPIO.setup(PIN2, GPIO.OUT)
GPIO.setup(EN1, GPIO.OUT)

pwm1 = GPIO.PWM(EN1,1000) # 100hz sampling
pwm1.start(90) # 90% Throlttle

# Gyro Settings
# MPU6050 Registers
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
wait_time = 0.05

# GPS Settings

gpgga_info = "$GPGGA,"
port = "/dev/ttyAMA0" # Port
ser = serial.Serial(port, baudrate=9600, timeout=0.5) # Baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0

def MPU_Init(self):
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value

def motor_control_right():

    pwm1 = GPIO.PWM(EN1, 1000)  # 100hz sampling
    pwm1.start(90)  # 90% Throlttle
    pwm1.ChangeDutyCycle(90)
    GPIO.output(PIN1,GPIO.HIGH)
    GPIO.output(PIN2,GPIO.LOW)
    GPIO.cleanup()

def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]  # extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]  # extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]  # extract longitude from GPGGA string

    print("NMEA Time: ", nmea_time, '\n')
    print("NMEA Latitude:", nmea_latitude, "NMEA Longitude:", nmea_longitude, '\n')

    lat = float(nmea_latitude)  # convert string into float for calculation
    longi = float(nmea_longitude)  # convertr string into float for calculation

    lat_in_degrees = convert_to_degrees(lat)  # get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi)  # get longitude in degree decimal format


# convert raw NMEA string into degree decimal format
def convert_to_degrees(raw_value):
    decimal_value = raw_value / 100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value)) / 0.6
    position = degrees + mm_mmmm
    position = "%.4f" % (position)
    return position

if __name__ == "__main__":
    try:
        while True:

            # Get GPS Data
            received_data = (str)(ser.readline())  # read NMEA string received
            GPGGA_data_available = received_data.find(gpgga_info)  # check for NMEA GPGGA string


            #Read Accelerometer raw value
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)

            # Read Gyroscope raw value
            gyro_x = read_raw_data(GYRO_XOUT_H)
            gyro_y = read_raw_data(GYRO_YOUT_H)
            gyro_z = read_raw_data(GYRO_ZOUT_H)

            # Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = acc_x / 16384.0
            Ay = acc_y / 16384.0
            Az = acc_z / 16384.0

            Gx = gyro_x / 131.0
            Gy = gyro_y / 131.0
            Gz = gyro_z / 131.0

            if (GPGGA_data_available > 0):
                GPGGA_buffer = received_data.split("$GPGGA,", 1)[1]  # store data coming after "$GPGGA," string
                NMEA_buff = (GPGGA_buffer.split(','))  # store comma separated data in buffer
                GPS_Info()  # get time, latitude, longitude

                # print("lat in degrees:", lat_in_degrees, " long in degree: ", long_in_degrees, '\n')
                final_data = '[{},{},{},{},{},{},{},{}]'.format(Ax, Ay, Az, Gx, Gy, Gz,lat_in_degrees,long_in_degrees)
                print('Sending: %s' % final_data)
                device.send_data_async(remote_device, final_data)
                time.sleep(wait_time)
            else:

                final_data = '[{},{},{},{},{},{}]'.format(Ax, Ay, Az, Gx, Gy, Gz)
                print('Sending: %s' % final_data)
                print("GPS is not working")
                device.send_data_async(remote_device, final_data)
                time.sleep(wait_time)

            if abs(Gx) > 15 or abs(Gy) > 15 or abs(Gz) > 15:
                if motor_count == 0:
                    motor_count += 1
                    print("Start Motor")

            if motor_count == 1:
                pwm1.ChangeDutyCycle(95)
                GPIO.output(PIN1, GPIO.HIGH)
                GPIO.output(PIN2, GPIO.LOW)
                print("Motor is working")

    except OSError:
        pass
    except IOError:
        pass
    except KeyboardInterrupt:
        GPIO.output(PIN1, GPIO.LOW)
        GPIO.output(PIN2, GPIO.LOW)
        GPIO.cleanup()
        device.close()
        sys.exit(0)

    except Exception as e:
        print("Transmit Fail : %s" % str(e))
        pass
