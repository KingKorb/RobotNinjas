"""
Test robot's velocity control using PI controller.
This test cathes motors' resonses toward a step signal.
"""
from time import sleep, time
from gpiozero import PhaseEnableRobot, Robot, LED
import serial
import matplotlib.pyplot as plt
import numpy as np

# setup motors
le = LED(22)
re = LED(23)
bot = PhaseEnableRobot((24, 12), (25, 13))
le.on()
re.on()
# setup serial port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

# setup constants
WHEEL_SEPARATION = 0.48
WHEEL_RADIUS = 0.05
K_P = 0.0003
K_I = 0.000011
lerr_buf = np.zeros(400)
rerr_buf = np.zeros(400)
# init variables
counter = 0
linear = 0  # robot linear velocity
angular = 0  # robot angular velocity
left_omega = 0  # wheel angular speed
right_omega = 0
left_dutycycle = 0
right_dutycycle = 0
lomg_list = []
romg_list = []
target_linear = 0
target_angular = 0
# main loop
while counter < 400:  # 6 seconds
    if ser.in_waiting > 0:
        line = ser.readline()
        if b'\xff' in line or b'\xfe' in line:
            continue
        speeds = line.decode('utf-8').rstrip().split(',')
        linear = float(speeds[0])
        angular = float(speeds[1])
        left_omega = float(speeds[2])
        right_omega = float(speeds[3])
        lomg_list.append(left_omega)
        romg_list.append(right_omega)
        # set target velocity
        if counter >= 100:  # at 1 sec
            target_linear = .5
        # compute target revolute speed
        left_target_omega = (target_linear - (target_angular * WHEEL_SEPARATION) / 2) / WHEEL_RADIUS
        right_target_omega = (target_linear + (target_angular * WHEEL_SEPARATION) / 2) / WHEEL_RADIUS
        # compute error
        left_error = left_target_omega - left_omega
        if left_omega > left_target_omega / 2:
            lerr_buf[counter] = left_error
        right_error = right_target_omega - right_omega
        if right_omega > right_target_omega / 2:
            rerr_buf[counter] = right_error
        # compute dutycycle increment
        left_dutycycle_inc = K_P * left_error + K_I * lerr_buf.sum()
        left_dutycycle += left_dutycycle_inc
        if left_dutycycle > 1:
            left_dutycycle = 1
        elif left_dutycycle < 0:
            left_dutycycle = 0
        right_dutycycle_inc = K_P * right_error + K_I * rerr_buf.sum()
        right_dutycycle += right_dutycycle_inc
        if right_dutycycle > 1:
            right_dutycycle = 1
        elif right_dutycycle < 0:
            right_dutycycle = 0
        # drive motors
        bot.left_motor.forward(left_dutycycle)
        bot.right_motor.forward(right_dutycycle)
        sleep(.02)
        print(
            "---\n",
            f"target_leftwheel_angular: {left_target_omega}, leftwheel_angular: {left_omega} \n", \
            f"target_rightwheel_angular: {right_target_omega}, rightwheel_angular: {right_omega}"
        )
        counter += 1

bot.stop()
le.off()
re.off()

# plot
letargomg_list = [0]*100 + [left_target_omega]*300
ritargomg_list = [0]*100 + [right_target_omega]*300
x = list(range(400))
plt.figure()
plt.subplot(211)
plt.plot(x, letargomg_list, color='tab:orange', linestyle='--')
plt.plot(x, lomg_list, color='lime')
plt.subplot(212)
plt.plot(x, ritargomg_list, color='tab:orange', linestyle='--')
plt.plot(x, romg_list, color='red')
plt.savefig('step_pi.png')
