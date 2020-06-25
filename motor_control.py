import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

A = 23
B = 24

GPIO.setup(A,GPIO.OUT) # Motor A,B Setup
GPIO.setup(B,GPIO.OUT)

try:
    while 1:
        GPIO.output(A,GPIO.HIGH) # Rotate Right
        GPIO.output(B,GPIO.LOW)

        time.sleep(5)

        GPIO.output(A,GPIO.LOW) # Stop Rotate
        GPIO.output(B,GPIO.LOW)

        time.sleep(2)

        GPIO.output(A,GPIO.LOW) # Rotate Left
        GPIO.output(B,GPIO.HIGH)

        time.sleep(5)

        GPIO.output(A, GPIO.LOW)  # Stop Rotate
        GPIO.output(B, GPIO.LOW)
        time.sleep(2)
except KeyboardInterrupt:
    GPIO.output(A,GPIO.LOW)
    GPIO.output(B,GPIO.LOW)
    GPIO.cleanup()