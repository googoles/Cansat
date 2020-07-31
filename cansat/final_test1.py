import sys
from digi.xbee.devices import *
import time
import os
import RPi.GPIO as GPIO
sys.path.append('/home/pi/python_project/')
import smbus
import serial


# I should integrate GPS Signal

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
    # GPIO.cleanup()

if __name__ == "__main__":
    try:
        while 1:
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

            Ax = round(Ax,4)
            Ay = round(Ay,4)
            Az = round(Az,4)
            Gx = round(Gx,4)
            Gy = round(Gy,4)
            Gz = round(Gz,4)

            # data float 처리 4자리?
            csum = Ax+Ay+Az+Gx+Gy+Gz
            # print(csum)
            csum_raw = '{:.4f}'.format(csum)
            print(csum_raw)
            # mpu_data_raw = '*,{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}'.format(Gx, Gy, Gz,Ax, Ay, Az)
            mpu_data_raw = '*,{},{},{},{},{},{}'.format(Gx, Gy, Gz, Ax, Ay, Az)
            front_key = bytearray([0x76,0x00,0x60,0x00])
            mpu_data_byte = mpu_data_raw.encode()
            mpu_data_bytearray = bytearray(mpu_data_byte)
            colon = bytearray(b',')
            csum_encode = csum_raw.encode()
            csum_bytearr = bytearray(csum_encode)
            final_key = bytearray([0x0D,0x0A])
            send_data = front_key + mpu_data_bytearray + colon + csum_bytearr + final_key
            # print(mpu_data_bytearray)
            # sent_to_land = [0x76,0x00,0x60,0x00,mpu_data,',',csum,0x0D,0x0A]
            print('Sending: %s' % send_data)
            # print(type(csum_bytearr))

            device.send_data_async(remote_device, send_data)
            time.sleep(wait_time)
            if abs(Gx) > 15 or abs(Gy) > 15 or abs(Gz) > 15:
                if motor_count == 0:
                    motor_count += 1
                    print("Start Motor")

            if motor_count == 1:
                pwm1.ChangeDutyCycle(95)
                GPIO.output(PIN1, GPIO.HIGH)
                GPIO.output(PIN2, GPIO.LOW)
                # motor_control_right()
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

    except Exception as e:
        print("Transmit Fail : %s" % str(e))
        pass



    # final_data = data


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
