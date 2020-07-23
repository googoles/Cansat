import RPi.GPIO as GPIO
import time



A = 24
B = 23
EN1 = 18 #
# def init():
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(EN1, GPIO.OUT) # EN
GPIO.setup(A,GPIO.OUT) # Motor A,B Setup
GPIO.setup(B,GPIO.OUT)

pwm1 = GPIO.PWM(EN1, 1000)  # 100, 약함 .
pwm1.start(90)
try:
    while 1:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pwm1.ChangeDutyCycle(90)
        GPIO.setup(EN1, GPIO.OUT)  # EN
        GPIO.setup(A, GPIO.OUT)  # Motor A,B Setup
        GPIO.setup(B, GPIO.OUT)
        GPIO.output(A,GPIO.HIGH) # Rotate Right
        GPIO.output(B,GPIO.LOW)
        GPIO.cleanup()
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
