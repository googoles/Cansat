import smbus  # import SMBus module of I2C
from time import sleep  # import
from digi.xbee.devices import *

device = XBeeDevice("/dev/ttyS0",9600)

device.open()
device.set_sync_ops_timeout(1000)
remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20040CA046A"))

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

wait_time = 0.01

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

# cycles = 1000000000
while 1:

    # Read Accelerometer raw value
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
    messages = '{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{},{},{}'.format(Ax, Ay, Az, Gx, Gy, Gz, 34.610854, 127.205932, 0)
    # print('Sending Data: %s' % messages)
    #
    # csum_raw = '{:.4f}'.format(csum)
    # print(csum_raw)
    # mpu_data_raw = '*,{},{},{},{},{},{}'.format(Gx, Gy, Gz, Ax, Ay, Az)
    front_key = bytearray([0x76, 0x00, 0x60, 0x00])
    encoded_messages = messages.encode()
    messages_bytearray = bytearray(encoded_messages)
    colon = bytearray(b',')
    # csum_encode = csum_raw.encode()
    # csum_bytearr = bytearray(csum_encode)
    final_key = bytearray([0x0D, 0x0A])
    send_data = front_key + messages_bytearray + colon + final_key
    # # print(mpu_data_bytearray)
    # # sent_to_land = [0x76,0x00,0x60,0x00,mpu_data,',',csum,0x0D,0x0A]

    # print(type(csum_bytearr))
    device.send_data_async(remote_device, send_data)
    print(send_data)
    # try:
    #
    #     # device.send_data_async(remote_device,'Sending Gyro Data')
    #     device.send_data_async(remote_device,send_data)
    #     print('Sending: %s' % send_data)
    #     print('Data sent success')
    # except Exception as e:
    #     print('Transmit Fail : %s' % str(e))
    #     pass
    sleep(wait_time)



