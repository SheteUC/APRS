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

def backward(n):
    distance = 25.4*n
    target = gyro_sensor.angle()
    gain = 2
    robot.reset()
    while -robot.distance() <= distance:
        correction = (target - gyro_sensor.angle()) * gain
        robot.drive(-100, correction)
        wait(10)
    robot.stop()

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


backward(12)
turn_right_angle(90)
forward(97)
turn_left_angle(90)
forward(12)

