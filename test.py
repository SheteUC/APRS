#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile, Image, SoundFile
import threading

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Configure 2 motors on Ports B and C.  Set the motor directions to
# counterclockwise, so that positive speed values make the robot move
# forward.  These will be the left and right motors of the Tank Bot.
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
medium_motor = Motor(Port.A)


# The wheel diameter of the Tank Bot is about 54 mm.
WHEEL_DIAMETER = 42

# The axle track is the distance between the centers of each of the
# wheels.  This is about 200 mm for the Tank Bot.
AXLE_TRACK = 200

# The Driving Base is comprised of 2 motors.  There is a wheel on each  
# motor.  The wheel diameter and axle track values are used to make the
# motors move at the correct speed when you give a drive command.
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

# Set up the Gyro Sensor. 
gyro_sensor = GyroSensor(Port.S1)

# Set up the Color Sensor.
color_sensor = ColorSensor(Port.S3)

# # Set up the Ultrasonic Sensor.
ultrasonic_sensor = UltrasonicSensor(Port.S2)

# Initialize the steering and overshoot variables.
steering = 60
overshoot = 5

reflection = 0

# turns no. of degrees towards the right direction 
def turn_right_angle(degrees):
    # Reset the Gyro Sensor angle.
    gyro_sensor.reset_angle(0)

    # Turn clockwise until the angle is 90 degrees.
    robot.drive(0, steering)

    ev3.speaker.beep()

    while gyro_sensor.angle() < (degrees-2) - overshoot:
        wait(1)
    robot.drive(0, 0)
    wait(100)

# turns no. of degrees towards the left direction 
def turn_left_angle(degrees):
    # Reset the Gyro Sensor angle.
    gyro_sensor.reset_angle(0)

    # Turn counter-clockwise until the angle is 90 degrees.
    robot.drive(0, -steering)

    ev3.speaker.beep()

    while gyro_sensor.angle() > overshoot - (degrees-2):
        wait(1)
    robot.drive(0, 0)
    wait(100)

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

    print(total)
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

def lift_up(time):
    medium_motor.run_time(150, time*1000)
    robot.stop()

def lift_down(time):
    medium_motor.run_time(-150, time*1000)
    robot.stop()


# given barcode
target = 3

# rack number for the box
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



forward(36)

turn_right_angle(90)

forward(8)
forward(a*6)


while ((color_sensor.color() == None) or not(color_sensor.ambient() < 3 and color_sensor.reflection() < 10 )):
    forward(0.125)
    robot.stop()
    wait(750)



# backward(a*6)
# backward(9)

# turn_left_angle(90)
# backward(36)

# robot.stop()
# wait(5000)
# target = 4

# item = barcode()
# ev3.screen.draw_text(50, 50, str(item))

# wait(5000)
# ev3.screen.clear()

# if (item == target):
#     ev3.screen.load_image(ImageFile.ACCEPT)
#     ev3.speaker.play_file(SoundFile.CHEERING)
# else:
#     ev3.screen.load_image(ImageFile.DECLINE)
#     ev3.speaker.play_file(SoundFile.BOO)

# forward(b*6)

# # forward(12)
# # forward(36)
# # forward(6)
# forward(9)

# turn_right_angle(90)

# forward(36)



        







