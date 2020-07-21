import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
AIN1 = 23
AIN2 = 24
PWMA = 18
c_step = 10
GPIO.setwarnings(False)
GPIO.setup(AIN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(AIN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PWMA, GPIO.OUT, initial=GPIO.LOW)
p = GPIO.PWM(PWMA, 100)
p.start(0)
try:
    while 1:
        GPIO.output(AIN1, GPIO.HIGH)
        for pw in range(0, 101, c_step):
            p.ChangeDutyCycle(pw)
            time.sleep(0.5)
        for pw in range(100, -1, c_step * -1):
            p.ChangeDutyCycle(pw)
            time.sleep(0.5)
        GPIO.output(AIN1, GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(AIN2, GPIO.HIGH)
        for pw in range(0, 101, c_step):
            p.ChangeDutyCycle(pw)
            time.sleep(0.5)
        GPIO.output(AIN2, GPIO.LOW)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
p.stop()
GPIO.cleanup()

import sys

import RPi.GPIO as GPIO
from time import sleep  # import

# pin1 = 23
# pin2 = 24
# EN1 = 18 # for pwm

# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(pin1, GPIO.OUT)
# GPIO.setup(pin2,GPIO.OUT)
# GPIO.setup(EN1,GPIO.OUT)

# pwm1 = GPIO.PWM(EN1,100) # configure
# pwm1.start(80)
#
#
# try:
#     while True:
#
#         pwm1.ChangeDutyCycle(70)
#         GPIO.output(pin1,True)
#         GPIO.output(pin2,False)
#         sleep(3)
#         pwm1.ChangeDutyCycle(70)
#         sleep(3)
#         pwm1.ChangeDutyCycle(70)
#         GPIO.output(pin1,False)
#         GPIO.output(pin2,True)
#         sleep(3)
#         pwm1.ChangeDutyCycle(80)
#         sleep(3)
#         pwm1.ChangeDutyCycle(70)
#         sleep(3)
#         GPIO.output(pin1,False)
#         GPIO.output(pin2,False)
#         sleep(2)
#
# except KeyboardInterrupt:
#     pwm1.stop()
#     GPIO.cleanup()
#     sys.exit()


# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)


# def init():
#     GPIO.setwarnings(False)
#     GPIO.setmode(GPIO.BCM)
#     GPIO.setup(23, GPIO.OUT)  # MOTOR 1 A,B Setup
#     GPIO.setup(24, GPIO.OUT)
#     GPIO.setup(18, GPIO.OUT)
#
# def setSpeed(speed,p):
#     p.ChangeDutyCycle(speed*10)
#
# try:
#     init()
#     p = GPIO.PWM(18, 1000)  # 100Hz
#     p.start(0)
#
#
#     while True:
#         for i in range(10):
#             GPIO.output(23,True)
#             GPIO.output(24,False)
#             setSpeed(i,p)
#             sleep(1)
#
# finally:
#     GPIO.cleanup()
