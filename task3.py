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

color_sensor = ColorSensor(Port.S3)

steering = 60
overshoot = 5

reflection = 0


def barcode():
    
    total = 0
    item_num = 0

    for i in range (0, 4):
        a = 0
        reflection = 0
        reflection = color_sensor.reflection()

        if (reflection < 35):
            a = 1
        print(a)
        total = (a * (2**(i))) + total
        medium_motor.hold()
        ev3.speaker.beep()
        wait(1000)
        forward(0.175)
        wait(500)

    if total == 8:
        return 1
    elif total == 10:
        return 2
    elif total == 12:
        return 3
    elif total == 9:
        return 4
    else:
        return 0

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


rack_num = 9
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

forward(15.45)

target = 3
wait(100)
item = barcode()
ev3.screen.draw_text(50, 50, str(item))

wait(5000)
ev3.screen.clear()

if (item == target):
    ev3.screen.load_image(ImageFile.ACCEPT)
    ev3.speaker.play_file(SoundFile.CHEERING)
else:
    ev3.screen.load_image(ImageFile.DECLINE)
    ev3.speaker.play_file(SoundFile.BOO)
    

