#!/usr/bin/env pybricks-micropython

import math
import numpy as np

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Initialize the EV3 brick.
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the drive base. In this example, the wheel diameter is 56mm.
# The distance between the two wheel-ground contact points is 112mm.
# drive_base = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)

# Set volume to 100% and make a beep to signify program has started
ev3.speaker.set_volume(50)
ev3.speaker.beep()
sonic_sensor = UltrasonicSensor(Port.S1)

def controlSystem(currentPose, targetPose):
    posErrorX = targetPose[0] - currentPose[0]
    posErrorY = targetPose[1] - currentPose[1]
    linDist = sqrt(posErrorX**2 + posErrorY**2)
    angCurrent = currentPose[2]

    angToTarget = math.atan(posErrorX/posErrory)
    angError = angToTarget - angCurrent

    derivative = linDist - previousLinDist
    previousLinDist = derivative

    Kp = 500
    Kd = 100
    Kp_turn = 500

    speed = (Kp * linDist) + (Kd * derivative)
    # speed = -speed

    left_motor.dc(speed + (Kp_turn * angError)) #speeds up the left wheel when angError is positive
    right_motor.dc(speed - (Kp_turn * angError)) #slows the right wheel when angError is positive

previousLinDist = 0
currentPose = [[0.1], [0.2]]
targetPose = [[0.5], [0.3]]

while True:
    controlSystem(currentPose, targetPose)
    # print("Distance: ", sonic_sensor.distance())
    print("Left Speed: ", leftMotor.speed())
    print("Right Speed: ", rightMotor.speed())
    wait(1)
