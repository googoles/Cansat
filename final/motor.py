import RPi.GPIO as GPIO
from time import sleep  # import

pin1 = 23
pin2 = 24
EN1 = 18 # for pwm

def init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin1, GPIO.OUT)  # MOTOR 1 A,B Setup
    GPIO.setup(pin2, GPIO.OUT)
    GPIO.setup(EN1, GPIO.OUT)

    pwm1 = GPIO.PWM(EN1,1000)
    pwm1.start(90)

class Motor:

    def motor_control_right(tf):
        init()
        GPIO.output(23,GPIO.HIGH)
        GPIO.output(24,GPIO.LOW)
        sleep(tf)
        GPIO.cleanup()

    def motor_control_left(tf):
        init()
        GPIO.output(23,GPIO.LOW)
        GPIO.output(24,GPIO.HIGH)
        sleep(tf)
        GPIO.cleanup()

    def motor_control_stop(tf):
        init()
        GPIO.output(23,GPIO.LOW)
        GPIO.output(24,GPIO.LOW)
        sleep(tf)
        GPIO.cleanup()