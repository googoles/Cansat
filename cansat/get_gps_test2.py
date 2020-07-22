import serial
import time
import string
import pynmea2

port = "/dev/ttyAMA0"
ser = serial.Serial(port, baudrate=9600, timeout=0.5)

strData = ser.readline()

if strData.startswith('$GPRMC'):
	try:
		arrFields = strData[:strData.index('*')].strip().split(',')

		self.mStrSentenceType = 'RMC'
		strUtcTime = arrFields[1]

		# 품질 : A(작동), V(에러)
		self.mStrGpsStatus = arrFields[2]

		self.mFltLat = float(self.dmToSd(arrFields[3]) * (1 if arrFields[4] == 'N' else -1))
		self.mFltLon = float(self.dmToSd(arrFields[5]) * (1 if arrFields[6] == 'E' else -1))

		# 속도 ( 단위는 Knots, Km/h 할려면 * 1.852)
		strSpeedKnots = arrFields[7]
		if strSpeedKnots :
			self.mFltSpeed = float(arrFields[7]) * 1.852
		else :
			self.mFltSpeed = 0

		# 방향
		strDirection = arrFields[8]
		if strDirection :
			self.mFltDirection = float(strDirection)
		else :
			self.mFltDirection = 0

		# 날짜
		strDate = arrFields[9]
		if strDate:
			intDay          = int(strDate[0:2])
			intMonth        = int(strDate[2:4])
			intYear         = int(strDate[4:6]) + 2000

			intHour     = int(strUtcTime[0:2])
			intMinute   = int(strUtcTime[2:4])
			intSecond   = int(strUtcTime[4:6])

			dtGpsDateTime = datetime.datetime(intYear, intMonth, intDay, intHour, intMinute, intSecond)
			self.mDtDateTime = DateUtil.utcToLocal(dtGpsDateTime)

	except ValueError:
		return 	False