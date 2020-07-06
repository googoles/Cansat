import RPi.GPIO as GPIO
from time import sleep  # import

def init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.OUT)  # MOTOR 1 A,B Setup
    GPIO.setup(24, GPIO.OUT)

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