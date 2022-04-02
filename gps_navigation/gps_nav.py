import pymap3d as pm
import numpy as np
import serial
import time

TARGET_COORDS = []
# fill in targets' latitude and longitude, e.g. TARGET_COORDS.append([35.08235, -92.45758])

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.reset_input_buffer()

i=0
while True:

    if ser.in_waiting > 0:
        ser.write(b"Hello from Raspberry Pi!\n")
        line = ser.readline().decode('utf-8').rstrip().split(',')
        Lat = float(line[0])  # latitude
        Lon = float(line[1])  # longitude
        Ori = float(line[2])  # angle to east
        
        # fit target into EAST-NORTH-UP frame attached on robot
        x,y,z = pm.geodetic2enu(TARGET_COORDS[i][0], TARGET_COORDS[i][1] , 95.0976, Lat, Long, 95.0976)
        if np.sqrt(x**2+y**2) < 1:  # check if target reached, threshold: 1 m
            i += 1  # use next target
            continue

        # compute angle offset
        theta = np.deg2rad(Ori)
        phe = np.arctan2(y, x)
        del_theta = phe - theta  # we use this angle to correct robot pointing direction
        
        # control robot's movement based on offset angle
#         if del_theta > np.pi/18:
#             robot.right()
#         elif del_theta < -np.pi/18:
#             robot.left()
#         else:
#             robot.forward()
        
        # print debugging info
        print(Lat, Lon, Ori)
        print(x,y,z)
        print(del_theta)
        print(np.linalg.norm((x,y)))
        print('---\n')

        time.sleep(.5)
    
