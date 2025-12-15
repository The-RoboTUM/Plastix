#!/usr/bin/env python3
from gpiozero import Robot
from gpiozero import DistanceSensor
from time import sleep
import sys
from signal import signal, SIGINT

#Variables
left_motor = (17,18)
right_motor = (22,23)
echo = 24
trigger = 12
minimum_distance = 20

robby = Robot(left_motor, right_motor)
sensor = DistanceSensor(echo, trigger)

def end_program(signum, frame):
    print("\nEnding control...")
    robby.close()
    sensor.close()
    sys.exit(0)

def main():
    signal(SIGINT, end_program)

    while True:
        if ((sensor.distance * 100) >= minimum_distance):
            robby.forward(0.3)
        else:
            robby.right(0.3)
            sleep(0.8)
            robby.stop()
        print((robby.left_motor.value, robby.right_motor.value))

if __name__ == "__main__":
    main()