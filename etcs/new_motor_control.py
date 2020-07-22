import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

A = 24
B = 23
EN1 = 18

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(A,GPIO.OUT) # Motor A,B Setup
GPIO.setup(B,GPIO.OUT)
GPIO.setup(EN1,GPIO.OUT)

pwm1 = GPIO.PWM(EN1,1000) # 100, 약함 .
pwm1.start(90)

def motor_control_right():

    pwm1 = GPIO.PWM(EN1, 1000)  # 100hz sampling
    pwm1.start(90)  # 90% Throlttle
    pwm1.ChangeDutyCycle(90)
    GPIO.output(pin1,GPIO.HIGH)
    GPIO.output(pin2,GPIO.LOW)
    GPIO.cleanup()

try:
    while 1:
        pwm1.ChangeDutyCycle(90)
        GPIO.output(A,GPIO.HIGH) # Rotate Right
        GPIO.output(B,GPIO.LOW)

        # time.sleep(2)
        #
        # GPIO.output(A,GPIO.LOW) # Stop Rotate
        # GPIO.output(B,GPIO.LOW)
        #
        # time.sleep(2)
        #
        # GPIO.output(A,GPIO.LOW) # Rotate Left
        # GPIO.output(B,GPIO.HIGH)
        #
        # time.sleep(2)
        #
        # GPIO.output(A, GPIO.LOW)  # Stop Rotate
        # GPIO.output(B, GPIO.LOW)
        # time.sleep(2)
except KeyboardInterrupt:
    GPIO.output(A,GPIO.LOW)
    GPIO.output(B,GPIO.LOW)
    GPIO.cleanup()