import RPi.GPIO as GPIO
from time import sleep  # import

GPIO_RP = 4
GPIO_RN = 25
GPIO_EN = 12


def init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_RP, GPIO.OUT)  # MOTOR 1 A,B Setup
    GPIO.setup(GPIO_RN, GPIO.OUT)
    GPIO.setup(GPIO_EN, GPIO.OUT)

def setSpeed(speed,p):
    p.ChangeDutyCycle(speed*10)

try:
    p = GPIO.PWM(GPIO_EN,100) # 100Hz
    p.start(0)

    while True:
        for i in range(10):
            GPIO.output(GPIO_RP,True)
            GPIO.output(GPIO_RN,False)
            setSpeed(i,p)
            sleep(1)

finally:
    GPIO.cleanup()
