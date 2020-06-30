from digi.xbee.devices import XBeeDevice

device = XBeeDevice("/dev/ttyUSB0",9600)

device.open()
device.set_sync_ops_timeout(10)

while True:
    try:

        a = input()
        device.send_data_broadcast(a)
    except KeyboardInterrupt:
        break
device.close()