#!/usr/bin/env python
import keyboard
from gpiozero import Robot
from gpiozero import DistanceSensor
from time import sleep
import sys
from signal import signal, SIGINT

#Pins
left_motor = (17,18)
right_motor = (22,23)
echo = 24
trigger = 12

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
        fast = False
        linear = 0
        angular = 0

        if ((sensor.distance * 100) >= 20):
            if keyboard.is_pressed("shift"):
                fast = True

            if keyboard.is_pressed("w"):
                linear += 0.5 + 0.3 * fast
            elif keyboard.is_pressed("s"):
                linear -= 0.5 + 0.3 * fast

            if keyboard.is_pressed("a"):
                angular += 0.3 + 0.2 * fast
            elif keyboard.is_pressed("d"):
                angular -= 0.3 + 0.2 * fast

        left_speed = linear - angular
        right_speed = linear + angular
        robby.values = (left_speed, right_speed)
        print(left_speed, right_speed)
        sleep(0.5)

if __name__ == "__main__":
    main()