# Raspberry-PI-RC-car

```python
#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#Definition of  motor pins 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Definition of  ultrasonic module pins
EchoPin = 0
TrigPin = 1

#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24

#Definition of servo pin
ServoPin = 23

#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#Ultrasonic pin,RGB pin,servo pin initialization
#infrared obstacle avoidance module pin
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo

    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)

    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)

#The servo rotates to the specified angle
def servo_appointed_detection(pos):
    for i in range(18):
        if pos == 90:
            pwm_servo.ChangeDutyCycle(1.85 + 10 * pos/180
        else:
            pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
def Autonomous_driving():

    #blue
    GPIO.output(LED_R, GPIO.LOW)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.HIGH)
    l = 1 #왼쪽 바퀴속도
    r = 1 #오른쪽 바퀴속도
    sr= 10  #spin right
    sl = 10  #spin left
    rt = 0.5  #time sleep
    lt = 0.5
    check=0
    distance = Distance_test()

    if distance <= 27:
        brake()

        servo_appointed_detection(0)
        time.sleep(1)
        rightdistance = Distance_test()

        servo_appointed_detection(180)
        time.sleep(1)
        leftdistance = Distance_test()

        servo_appointed_detection(90)
        time.sleep(1)
        frontdistance = Distance_test()

        if leftdistance >= rightdistance:

            #Magenta
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)

            spin_left(sl,sr)
            time.sleep(lt)
            run(l,r)
        elif leftdistance < rightdistance:

            #Magenta
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)

            spin_right(sl,sr)
            time.sleep(rt)
            run(l,r)

        elif leftdistance <= 15 and rightdistance <= 15 and frontdistance <= 15:
            check=1

            #Magenta
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)

            spin_right(sl,sr)
            time.sleep(1)
            run(l,r)

        elif check==1 and frontdistance <= 90:
            check=0

            #Magenta
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)

            spin_right(sl,sr)
            time.sleep(rt)
            run(l,r)

    else:
        run(l,r)

#delay 2s   
time.sleep(2)

#advance

def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#back

def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn left
def left(leftspeed, rightspeed):

    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed) 

#trun right 

def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
 
#turn left in place
def spin_left(leftspeed, rightspeed):

    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn right in place

def spin_right(leftspeed, rightspeed):

    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#brake
def brake():

   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)

#Ultrasonic function

def Distance_test():

    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)

    GPIO.output(TrigPin,GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()

    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    time.sleep(0.01)

    return ((t2 - t1)* 340 / 2) * 100

#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.

try:
    init()
    while True:
        Autonomous_driving()

except KeyboardInterrupt:
    pass

pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()
```
