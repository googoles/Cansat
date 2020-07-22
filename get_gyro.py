import RPi.GPIO as GPIO
import smbus

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


class Gyro:
    # some MPU6050 Registers and their Address

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

    MPU_Init()
    # def get_data(self):
    #     cycles = 10000000000000
    #     for x in range(cycles):  # Sending data to Land
    #         ##  #Read Accelerometer raw value
    #         acc_x = self.read_raw_data(ACCEL_XOUT_H)
    #         acc_y = self.read_raw_data(ACCEL_YOUT_H)
    #         acc_z = self.read_raw_data(ACCEL_ZOUT_H)
    #
    #         # Read Gyroscope raw value
    #         gyro_x = self.read_raw_data(GYRO_XOUT_H)
    #         gyro_y = self.read_raw_data(GYRO_YOUT_H)
    #         gyro_z = self.read_raw_data(GYRO_ZOUT_H)
    #
    #         # Full scale range +/- 250 degree/C as per sensitivity scale factor
    #         Ax = acc_x / 16384.0
    #         Ay = acc_y / 16384.0
    #         Az = acc_z / 16384.0
    #
    #         Gx = gyro_x / 131.0
    #         Gy = gyro_y / 131.0
    #         Gz = gyro_z / 131.0
    #         mpu_data = '[{},{},{},{},{},{}]'.format(Ax, Ay, Az, Gx, Gy, Gz)
    #         print('Sending: %s' % mpu_data)
    # cycles = 10000000000000
    while 1:
        # try:

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
        # mpu_data = '[{},{},{},{},{},{}]'.format(Ax, Ay, Az, Gx, Gy, Gz)
        # print('Sending: %s' % mpu_data)
        # except OSError:
        #     pass
    MPU_Init()

    # for _ in range(cycles):  # Sending data to Land
    #     ##  #Read Accelerometer raw value
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
    #     mpu_data = '[{},{},{},{},{},{}]'.format(Ax, Ay, Az, Gx, Gy, Gz)
    #     print('Sending: %s' % mpu_data)

