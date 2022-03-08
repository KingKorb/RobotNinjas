from time import sleep
from gpiozero import PhaseEnableRobot, Robot, LED

le = LED(22)
re = LED(23)
bot = PhaseEnableRobot((24, 12), (25, 13))
le.on()
re.on()
while True:
    bot.forward(.3)
    sleep(2)
    bot.backward(.3)
    sleep(2)
    bot.left(.3)
    sleep(2)
    bot.right(.3)
    sleep(2)
