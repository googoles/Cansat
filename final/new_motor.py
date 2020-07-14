import sys

import RPi.GPIO as GPIO
from time import sleep  # import

pin1 = 23
pin2 = 24
EN1 = 18 # for pwm

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin1, GPIO.OUT)
GPIO.setup(pin2,GPIO.OUT)
GPIO.setup(EN1,GPIO.OUT)


# GPIO.setup(23, GPIO.OUT)  # MOTOR 1 A,B Setup
# GPIO.setup(24, GPIO.OUT)

pwm1 = GPIO.PWM(EN1,50)
pwm1.start(50)

try:
    while True:

        pwm1.ChangeDutyCycle(90)
        GPIO.output(pin1,True)
        GPIO.output(pin2,False)
        sleep(3)
        pwm1.ChangeDutyCycle(30)
        sleep(3)
        pwm1.ChangeDutyCycle(40)
        GPIO.output(pin1,False)
        GPIO.output(pin2,True)
        sleep(3)
        pwm1.ChangeDutyCycle(80)
        sleep(3)
        pwm1.ChangeDutyCycle(30)
        sleep(3)
        GPIO.output(pin1,False)
        GPIO.output(pin2,False)
        sleep(2)

except KeyboardInterrupt:
    pwm1.stop()
    GPIO.cleanup()
    sys.exit()



# # GPIO.setwarnings(False)
# # GPIO.setmode(GPIO.BCM)
# def init():
#     GPIO.setwarnings(False)
#     GPIO.setmode(GPIO.BCM)
#     GPIO.setup(GPIO_RP, GPIO.OUT)  # MOTOR 1 A,B Setup
#     GPIO.setup(GPIO_RN, GPIO.OUT)
#     GPIO.setup(GPIO_EN, GPIO.OUT)
#
# def setSpeed(speed,p):
#     p.ChangeDutyCycle(speed*10)
#
# try:
#     init()
#     p = GPIO.PWM(GPIO_EN,1000) # 100Hz
#     p.start(0)
#
#
#     while True:
#         for i in range(10):
#             GPIO.output(GPIO_RP,True)
#             GPIO.output(GPIO_RN,False)
#             setSpeed(i,p)
#             sleep(1)
#
# finally:
#     GPIO.cleanup()

