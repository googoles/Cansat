import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
Motor1E = 23
Motor1A = 24
Motor1B = 18

GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

forward=GPIO.PWM(Motor1A,100)
reverse=GPIO.PWM(Motor1B,100)

forward.start(0)
reverse.start(0)

print("Go backward")
GPIO.output(Motor1E,GPIO.HIGH)
forward.ChangeDutyCycle(0)
reverse.ChangeDutyCycle(80)



