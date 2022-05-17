#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile, Image, SoundFile


ev3 = EV3Brick()

left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
medium_motor = Motor(Port.A)


WHEEL_DIAMETER = 42

AXLE_TRACK = 200

robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)


gyro_sensor = GyroSensor(Port.S1)



steering = 60
overshoot = 5

reflection = 0


def turn_right_angle(degrees):
   
    gyro_sensor.reset_angle(0)

    robot.drive(0, steering)

    ev3.speaker.beep()

    while gyro_sensor.angle() < (degrees-2) - overshoot:
        wait(1)
    robot.drive(0, 0)
    wait(100)


def turn_left_angle(degrees):
    
    gyro_sensor.reset_angle(0)

    robot.drive(0, -steering)

    ev3.speaker.beep()

    while gyro_sensor.angle() > overshoot - (degrees-2):
        wait(1)
    robot.drive(0, 0)
    wait(100)


def forward(n):
    distance = 25.4*n
    target = gyro_sensor.angle()
    gain = 2
    robot.reset()
    while robot.distance() <= distance:
        correction = (target - gyro_sensor.angle()) * gain
        robot.drive(100, correction)
        wait(10)
    robot.stop()


rack_num = 12
if (rack_num == 7):
    a = 0
    b = 5
elif (rack_num == 8):
    a = 1
    b = 4
elif (rack_num == 9):
    a = 2
    b = 3
elif (rack_num == 10):
    a = 3
    b = 2
elif (rack_num == 11):
    a = 4
    b = 1
elif (rack_num == 12):
    a = 5
    b = 0
   

forward(36.35)

turn_right_angle(90)

forward(9)
forward(a*6)

robot.stop()
wait(5000)

forward(b*6)

forward(12)
forward(36)
forward(10.5)

turn_right_angle(90)

forward(36)

