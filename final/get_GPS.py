import serial

port = '/dev/ttyAMA0'

def parseGPS(data):

    if data[0:6] == "$GPGGA":
        s = data.split(',')
        if s[7] == '0':
            print('No satellite data available')
            return

            time = s[1][0:2] + ':' +s[1][2:4] + ':' + s[1][4:6]
            lat = decode(s[2])
            dirLat = s[3]
            lon = decode(s[5])
            dirLon = s[5]
            alt = s[9] + 'm'
            sat = s[7]
            print('')