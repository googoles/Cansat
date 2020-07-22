import smbus # import SMBus module of I2C
from time import sleep
import datetime as dt
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import matplotlib.animation as animation


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
A = 23
B = 24

# Gyroscope setup

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

GPIO.setup(A,GPIO.OUT) # MOTOR 1 A,B Setup
GPIO.setup(B,GPIO.OUT)

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

def get_gyro_acc_data(i,xs, ys):
    print('i : ', i)

    xs.append(dt.datetime.now().strftime('%H%M%S'))

    ##	#Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0 # ACC X Axis Data
    Ay = acc_y / 16384.0 # ACC Y Axis Data
    Az = acc_z / 16384.0 # Acc Z Axis Data

    Gx = gyro_x / 131.0 # Gyro X Axis Data
    Gy = gyro_y / 131.0 # Gyro Y Axis Data
    Gz = gyro_z / 131.0 # Gyro Z Axis Data

    print("Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz, u'\u00b0' + "/s",
          "\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)

    ys.append(Az)  # we are plotting Ax timebeing

    sleep(1)

    # Limit x and y lists to 40 items

    xs = xs[-40:]
    ys = ys[-40:]

    print("ys: ", ys)
    print("xs : ", xs)
    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    print("Format plot")
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.20)

    plt.title('acceleration data over Time')
    plt.ylabel('m/s')

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

MPU_Init()

def motor_control():
    ##	#Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0  # ACC X Axis Data
    Ay = acc_y / 16384.0  # ACC Y Axis Data
    Az = acc_z / 16384.0  # Acc Z Axis Data

    Gx = gyro_x / 131.0  # Gyro X Axis Data
    Gy = gyro_y / 131.0  # Gyro Y Axis Data
    Gz = gyro_z / 131.0  # Gyro Z Axis Data

    # First Condition / if any axis is over 120 degree than start's motor


    try:
        while 1: # If you want to add switch, please use this function
            if abs(Gx) >= 6 or abs(Gy) >= 6 or abs(Gz) >= 6:
                GPIO.output(A, GPIO.HIGH)  # Rotate Right
                GPIO.output(B, GPIO.LOW)

            # GPIO.output(A, GPIO.LOW)  # Stop Rotate
            # GPIO.output(B, GPIO.LOW)
            #
            # sleep(2)
            #
            # GPIO.output(A, GPIO.LOW)  # Rotate Left
            # GPIO.output(B, GPIO.HIGH)
            #
            # sleep(5)
            #
            # GPIO.output(A, GPIO.LOW)  # Stop Rotate
            # GPIO.output(B, GPIO.LOW)
            # sleep(2)
    except KeyboardInterrupt:
        GPIO.output(A, GPIO.LOW)
        GPIO.output(B, GPIO.LOW)
        GPIO.cleanup()


print(" Reading Data of Gyroscope and Accelerometer")
ani = animation.FuncAnimation(fig, get_gyro_acc_data, fargs=(xs, ys), interval=1)
plt.show()
