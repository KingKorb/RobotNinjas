import RPi.GPIO as GPIO
import pymap3d as pm
import numpy as np
import serial
import time

Motor1PWM = 12
Motor1D = 24
Motor1E= 22
Motor2PWM = 13
Motor2D = 25
Motor2E= 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(Motor1PWM, GPIO.OUT)
GPIO.setup(Motor1D, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)
GPIO.setup(Motor2PWM, GPIO.OUT)
GPIO.setup(Motor2D, GPIO.OUT)
GPIO.setup(Motor2E, GPIO.OUT)

pwm = GPIO.PWM(Motor1PWM, 1000)
pwm2 = GPIO.PWM(Motor2PWM, 1000)
pwm.start(0)
pwm2.start(0)

def forward():
    print ("Moving Forward")
    GPIO.output(Motor1PWM,GPIO.HIGH)
    GPIO.output(Motor1D,GPIO.LOW)
    GPIO.output(Motor1E,GPIO.HIGH)
    pwm.ChangeDutyCycle(80)

    GPIO.output(Motor2PWM,GPIO.HIGH)
    GPIO.output(Motor2D,GPIO.HIGH)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm2.ChangeDutyCycle(80)
    #time.sleep(2)

def backward():
    print ("Moving Backwards")
    #GPIO.output(Motor1PWM,GPIO.HIGH)
    GPIO.output(Motor1D,GPIO.HIGH)
    GPIO.output(Motor1E,GPIO.HIGH)
    pwm.ChangeDutyCycle(80)

    #GPIO.output(Motor2PWM,GPIO.HIGH)
    GPIO.output(Motor2D,GPIO.LOW)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm2.ChangeDutyCycle(80)
    #time.sleep(2)

def right():
    print ("Moving Right")
    #GPIO.output(Motor1PWM,GPIO.HIGH)
    GPIO.output(Motor1D,GPIO.LOW)
    GPIO.output(Motor1E,GPIO.HIGH)
    pwm.ChangeDutyCycle(60)

    #GPIO.output(Motor2PWM,GPIO.HIGH)
    GPIO.output(Motor2D,GPIO.LOW)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm2.ChangeDutyCycle(60)
    #time.sleep(2)

def left():
    print ("Moving Left")
    #GPIO.output(Motor1PWM,GPIO.HIGH)
    GPIO.output(Motor1D,GPIO.HIGH)
    GPIO.output(Motor1E,GPIO.HIGH)
    pwm.ChangeDutyCycle(60)
    #GPIO.output(Motor2PWM,GPIO.HIGH)
    GPIO.output(Motor2D,GPIO.HIGH)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm2.ChangeDutyCycle(60)

def stop():
    print ("Now stop")
    GPIO.output(Motor1E,GPIO.LOW)
    GPIO.output(Motor2E,GPIO.LOW)

        
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.reset_input_buffer()
target_lat = [35.08331,35.08335,35.08331,35.08343,35.08343,3508343,35.08341]
target_lon = [-92.45882,-92.45871,-92.45862,-92.45859,-92.45867,-92.45879,-92.45881]
i=0

while True:
#for _ in range(2000): 
    if ser.in_waiting > 0:
        ser.write(b"Hello from Raspberry Pi!\n")
        line = ser.readline().decode('utf-8').rstrip().split(',')
        Lat = float(line[0])
        Long = float(line[1])
        Ori = float(line[2])
        
        #x,y,z = pm.geodetic2enu(35.08341, -92.45879 , 95.0976, Lat, Long, 95.0976)
        x,y,z = pm.geodetic2enu(target_lat[i], target_lon[i] , 95.0976, Lat, Long, 95.0976)
        
        theta = np.deg2rad(Ori)
        if theta > np.pi:
            theta = theta - 2*np.pi
        elif theta > 2*np.pi:
            theta = theta - 2*np.pi
        phe = np.arctan2(y, x)
        del_theta = phe - theta
        
        if del_theta > np.pi/6:
            left()
        elif del_theta < -np.pi/6:
            right()
        else:
            forward()
        
        if np.sqrt(x**2+y**2) < 2:  # TODO: tweak this number to get a reasonable target circle
             i += 1
#         print(x,y,z)
#         print(del_theta)
        print(f'distance: {np.linalg.norm((x,y))}')
        print(f'GPS Coords: {Lat, Long, Ori}')
        print(f'current target: {i}')
        print('\n')

        time.sleep(.05)
        
GPIO.cleanup()
