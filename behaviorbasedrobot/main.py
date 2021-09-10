#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import random


distance = 20
threshold = 25 #threshold for determining if black or silver
ninety_degree_turn = 540
forthyfive_degree_turn = 270
left_is_black = False
right_is_black = False

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize a motor at port B & C.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
left_color = ColorSensor(Port.S2)
right_color = ColorSensor(Port.S3)
center_color = ColorSensor(Port.S4)

# Create your objects here
base = DriveBase(left_motor, right_motor, 57, 175) 
#print(base.settings())
#base.settings(132, 300, 50, 100) 
# Write your program here

def random_left_right():
    return random.choice(["left","right"])

def random_dir():
    return random.choice(["left","right", "center"])

def update_sensor_values():
    right_is_black = right_color.reflection() < threshold
    left_is_black = left_color.reflection() < threshold
    center_is_black = center_color.reflection() < threshold

 
while (True):
    base.drive(100, 0) 
    right_is_black = right_color.reflection() < threshold
    left_is_black = left_color.reflection() < threshold
    center_is_black = center_color.reflection() < threshold

    if (left_is_black and not right_is_black and center_is_black): # left turn - T-intersection
        base.turn(-10)
        base.straight(10)
        continue

    if (right_is_black and not left_is_black and center_is_black): # right turn - T-intersection
        base.turn(10)
        base.straight(10)
        continue

    if (left_is_black and not right_is_black and not center_is_black): # left turn - Corner, adjustment
        while (not center_is_black):
            base.turn(-20)
            center_is_black = center_color.reflection() < threshold
        continue

    if (right_is_black and not left_is_black and not center_is_black): # right turn 
        while (not center_is_black):
            base.turn(20)
            center_is_black = center_color.reflection() < threshold
        continue

    if (left_is_black and right_is_black and not center_is_black): #T-intersection 
        base.straight(40)
        if (random_left_right() == "left"):
            base.turn(-90)
            continue
        else:
            base.turn(90)
            continue

    if (left_is_black and right_is_black and center_is_black): # normal intersection
        direction = random_dir()
        base.straight(40)
        if (direction == "left"): 
            base.turn(-90)
            continue
        elif (direction == "right"):
            base.turn(90)
            continue
        else :
            continue
    
    #if (not left_is_black and not right_is_black and not center_is_black):
