#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import random


straight_distance = 65
threshold = 25 #threshold for determining if black or silver
ninety_degree_turn = 84

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
 
while (True):
    base.drive(130, 0) 
    right_is_black = right_color.reflection() < threshold
    left_is_black = left_color.reflection() < threshold
    center_is_black = center_color.reflection() < threshold

    if (not center_is_black): #adjust direction
        if (left_is_black): #adjust left
            base.turn(-8)
            continue
        if (right_is_black): #adjust right
            base.turn(8)
            continue
    
    if (center_is_black and right_is_black and left_is_black): #T-intersection or X-intersection
        base.straight(straight_distance)
        center_is_black = center_color.reflection() < threshold
        if (center_is_black): # X-intersection
            direction = random_dir()
            if (direction == "left"): 
                base.turn(-ninety_degree_turn)
                continue
            elif (direction == "right"):
                base.turn(ninety_degree_turn)
                continue
            else :
                continue
        else : # T-intersection
            if (random_left_right() == "left"):
                base.turn(-ninety_degree_turn)
                continue
            else:
                base.turn(ninety_degree_turn)
                continue
    
    if (center_is_black and not right_is_black and left_is_black): #path left
        base.straight(straight_distance)
        center_is_black = center_color.reflection() < threshold
        if (not center_is_black): #Turn left
            base.turn(-ninety_degree_turn)
            continue
        else: #Left of straight
            if (random_left_right() == "left"):
                base.turn(-ninety_degree_turn)
                continue
            else:
                continue

    if (center_is_black and right_is_black and not left_is_black): #path right
        base.straight(straight_distance)
        center_is_black = center_color.reflection() < threshold
        if (not center_is_black): #Turn right
            base.turn(ninety_degree_turn)
            continue
        else: #Left of straight
            if (random_left_right() == "right"):
                base.turn(ninety_degree_turn)
                continue
            else:
                continue