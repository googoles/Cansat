import smbus  # import SMBus module of I2C
from time import sleep  # import
from digi.xbee.devices import *
import RPi.GPIO as GPIO

device = XBeeDevice("/dev/ttyUSB0",9600)

device.open()
device.set_sync_ops_timeout(100)
remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20040CA046A"))

# Set time
current_time = 0.
start_time = time.time()

# some MPU6050 Registers and their Address
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
# Set GPIO
def init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.OUT)  # MOTOR 1 A,B Setup
    GPIO.setup(24, GPIO.OUT)

def MPU_Init():
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

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address
MPU_Init()

cycles = 10000000000
for x in range(cycles): # Sending data to Land
    ##  #Read Accelerometer raw value
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
    messages = '[{},{},{},{},{},{}]'.format(Ax, Ay, Az, Gx, Gy, Gz)
    print('Sending: %s' % messages)
    try:
        device.send_data_async(remote_device,messages)
        print('Data sent success')
    except Exception as e:
        print('Transmit Fail : %s' % str(e))

    sleep(wait_time)

def motor_control_right(tf):
    init()
    GPIO.output(23,GPIO.HIGH)
    GPIO.output(24,GPIO.LOW)
    sleep(tf)
    GPIO.cleanup()

def motor_control_left(tf):
    init()
    GPIO.output(23,GPIO.LOW)
    GPIO.output(24,GPIO.HIGH)
    sleep(tf)
    GPIO.cleanup()

def motor_control_stop(tf):
    init()
    GPIO.output(23,GPIO.LOW)
    GPIO.output(24,GPIO.LOW)
    sleep(tf)
    GPIO.cleanup()

while True:

    if abs(Gx) >= 6 or abs(Gy) >= 6 or abs(Gz) >= 6:
        motor_control_right(10)
    else:
        motor_control_stop(5)

    current_time = time.time() - start_time
    print(current_time)



