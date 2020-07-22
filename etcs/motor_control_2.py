



def motor_control_right():
    pwm1 = GPIO.PWM(EN1, 1000)  # 100hz sampling
    pwm1.start(90)  # 90% Throlttle
    pwm1.ChangeDutyCycle(90)
    GPIO.output(pin1, GPIO.HIGH)
    GPIO.output(pin2, GPIO.LOW)
    GPIO.cleanup()