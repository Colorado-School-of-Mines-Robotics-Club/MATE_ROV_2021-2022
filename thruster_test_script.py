from adafruit_servokit import ServoKit
from time import sleep
kit = ServoKit(channels=16)
for i in range(0,16):
	kit.continuous_servo[i].set_pulse_width_range(500,2500)


try:
	while True:
		i = int(input("What thruster?: "))
		kit.continuous_servo[i].throttle = 0.1
		sleep(0.5)
		kit.continuous_servo[i].throttle = 0
except:
	for i in range(0,16):
		kit.continuous_servo[i].throttle = 0
