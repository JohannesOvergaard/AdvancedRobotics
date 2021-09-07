#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import random


distance = 100
threshold = 25 #threshold for determining if black or silver
ninety_degree_turn = 540

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize a motor at port B & C.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
left_color = ColorSensor(Port.S2)
right_color = ColorSensor(Port.S3)
center_color = ColorSensor(Port.S4)

# Create your objects here
base = DriveBase(left_motor, right_motor, 57, 155)
#print(base.settings())
# base.settings(200, 530, 98, 392) SPEED
# Write your program here

def ninety_degree_left_turn():
    right_motor.run_angle(200, ninety_degree_turn)


def ninety_degree_right_turn():
    left_motor.run_angle(200, ninety_degree_turn)

def random_left_right():
    return random.choice(["left","right"])


while (True):
    base.straight(distance)
    base.stop() #kill base temporarily to be able to move motors independently

    left_is_black = left_color.reflection() < threshold
    right_is_black = right_color.reflection() < threshold
    center_is_black = center_color.reflection() < threshold

    if (left_is_black and not right_is_black): # left turn
        ninety_degree_left_turn()

    if (right_is_black and not left_is_black): # right turn
        ninety_degree_right_turn()

    if (left_is_black and right_is_black): #T-intersection
        if (random_left_right() == "left"):
            ninety_degree_left_turn()
        else:
            ninety_degree_right_turn()

    if (left_is_black and right_is_black and center_is_black): # normal intersection
        #vælg tilfældigt mellem tre directions
    


   


