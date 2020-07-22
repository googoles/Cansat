import sys
from digi.xbee.devices import *
import time
import os
import RPi.GPIO as GPIO
sys.path.append('/home/pi/python_project/')
import smbus
import serial
import get_gyro


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
pin1 = 23
pin2 = 24
EN1 = 18
# for pwm

# def init():
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin2, GPIO.OUT)  # MOTOR 1 A,B Setup
GPIO.setup(pin1, GPIO.OUT)
GPIO.setup(EN1, GPIO.OUT)

pwm1 = GPIO.PWM(EN1,1000) # 100Hz, 약함 .
pwm1.start(90)

# pwm1 = GPIO.PWM(EN1,1000) # 100hz sampling
# pwm1.start(90) # 90% Throlttle

# Gyro Settings
# MPU6050 Registers


# bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
# Device_Address = 0x68  # MPU6050 device address
# PWR_MGMT_1 = 0x6B
# SMPLRT_DIV = 0x19
# CONFIG = 0x1A
# GYRO_CONFIG = 0x1B
# INT_ENABLE = 0x38
# ACCEL_XOUT_H = 0x3B
# ACCEL_YOUT_H = 0x3D
# ACCEL_ZOUT_H = 0x3F
# GYRO_XOUT_H = 0x43
# GYRO_YOUT_H = 0x45
# GYRO_ZOUT_H = 0x47
# wait_time = 0.05
#
# def MPU_Init(self):
#     # write to sample rate register
#     bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
#
#     # Write to power management register
#     bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
#
#     # Write to Configuration register
#     bus.write_byte_data(Device_Address, CONFIG, 0)
#
#     # Write to Gyro configuration register
#     bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
#
#     # Write to interrupt enable register
#     bus.write_byte_data(Device_Address, INT_ENABLE, 1)
#
# def read_raw_data(addr):
#     # Accelero and Gyro value are 16-bit
#     high = bus.read_byte_data(Device_Address, addr)
#     low = bus.read_byte_data(Device_Address, addr + 1)
#
#     # concatenate higher and lower value
#     value = ((high << 8) | low)
#
#     # to get signed value from mpu6050
#     if (value > 32768):
#         value = value - 65536
#     return value


# def motor_control_right():
#     init()
#     pwm1 = GPIO.PWM(EN1, 1000)  # 100hz sampling
#     pwm1.start(90)  # 90% Throlttle
#     pwm1.ChangeDutyCycle(90)
#     GPIO.output(pin1,GPIO.HIGH)
#     GPIO.output(pin2,GPIO.LOW)
#     GPIO.cleanup()
#
# def motor_control_left():
#     init()
#     pwm1 = GPIO.PWM(EN1, 1000)  # 100hz sampling
#     pwm1.start(90)  # 90% Throlttle
#     pwm1.ChangeDutyCycle(90)
#     GPIO.output(pin1,GPIO.LOW)
#     GPIO.output(pin2,GPIO.HIGH)
#     GPIO.cleanup()
#
# def motor_control_stop():
#     init()
#     pwm1 = GPIO.PWM(EN1, 1000)  # 100hz sampling
#     pwm1.start(90)  # 90% Throlttle
#     pwm1.ChangeDutyCycle(0)
#     GPIO.output(pin1,GPIO.LOW)
#     GPIO.output(pin2,GPIO.LOW)
#     GPIO.cleanup()

# get_gyro()


if __name__ == "__main__":
    # MPU_Init()
    # parse = get_gyro.MPU_Init()

    while 1:
        gyro = get_gyro.Gyro()
        while 1: # 실행한다음 멈추는 구문
            print("Not complete")
            # 여기에 모터 집어넣자
            # if abs(Gx) > 0.1 or abs(Gy) > 0.1 or abs(Gz) > 0.1: # Motor Switch
            # break


    # final_data = data
        print(gyro)

    # while 1:
    #     try:
    #
    #     # # Get GPS Values
    #     # dataout = pynmea2.NMEAStreamReader()  # Getting Info from GPS Module
    #     # newdata = ser.readline()
    #     #
    #     # if newdata[0:6] == "$GPRMC":
    #     #     newmsg = pynmea2.parse(newdata)
    #     #     lat = newmsg.latitude
    #     #     lng = newmsg.longitude
    #     #     gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
    #     #     print(gps)
    #     # else:
    #     #     lat = 0
    #     #     lng = 0
    #
    #     #Read Accelerometer raw value
    #     acc_x = read_raw_data(ACCEL_XOUT_H)
    #     acc_y = read_raw_data(ACCEL_YOUT_H)
    #     acc_z = read_raw_data(ACCEL_ZOUT_H)
    #
    #     # Read Gyroscope raw value
    #     gyro_x = read_raw_data(GYRO_XOUT_H)
    #     gyro_y = read_raw_data(GYRO_YOUT_H)
    #     gyro_z = read_raw_data(GYRO_ZOUT_H)
    #
    #     # Full scale range +/- 250 degree/C as per sensitivity scale factor
    #     Ax = acc_x / 16384.0
    #     Ay = acc_y / 16384.0
    #     Az = acc_z / 16384.0
    #
    #     Gx = gyro_x / 131.0
    #     Gy = gyro_y / 131.0
    #     Gz = gyro_z / 131.0
    #
    #     final_data = '[{},{},{},{},{},{}]'.format(Ax, Ay, Az, Gx, Gy, Gz)
    #     print('Sending: %s' % final_data)
    #     time.sleep(wait_time)
    #     try:
    #         device.send_data_async(remote_device,final_data)
    #         print('Data sent Success')
    #         # if abs(Gx) > 0.1 or abs(Gy) > 0.1 or abs(Gz) > 0.1:  # Motor Switch
    #         #     motor_control_right()
    #     except Exception as e:
    #         print('Transmit Fail : %s' % str(e))
    #         pass
    #
    #     except KeyboardInterrupt:
    #         GPIO.output(pin1, GPIO.LOW)
    #         GPIO.output(pin2, GPIO.LOW)
    #         GPIO.cleanup()
    #
    # if abs(Gx) > 0.1 or abs(Gy) > 0.1 or abs(Gz) > 0.1: # Motor Switch
    #     print("Motor Start")
    #     pwm1.ChangeDutyCycle(90)
    #     GPIO.output(pin1,GPIO.HIGH)
    #     GPIO.output(pin2,GPIO.LOW)

    # MPU_Init()



# except OSError:
# pass
#
# except IOError:
# pass
