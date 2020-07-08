import sys
import get_gyro
from digi.xbee.devices import *
import motor
import time
sys.path.append('/home/pi/python_project/final')

device = XBeeDevice("/dev/ttyUSB0",9600)
device.open()
device.set_sync_ops_timeout(100)
remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20040CA046A"))
start_time = time.time()

if __name__ == "__main__":

    while 1:
        try:
            gyro = get_gyro.Gyro()
            # gyro.get_data()
            device.send_data_async(remote_device, gyro.mpu_data)
            print('Data sent success')
            if abs(gyro.Gx) >= 1 or abs(gyro.Gy) >= 1 or abs(gyro.Gz) >= 1:
                motor.motor_control_right(5)
            else:
                motor.motor_control_stop(5)

            current_time = time.time() - start_time
            print(current_time)
        except OSError:
            pass


