#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import random

'''
right
backward
right
right
right
forward
backward
forward
right
right
forward
right
right
right
backward
left
left
left
forward
'''

straight_distance = 55
backward_distance = -125
threshold = 15 #threshold for determining if black or silver
ninety_degree_turn = 84
ROBOT_ORIENT = ""
INSTRUCTION_STRING = "up up left left up down right up left left right down down up left down right right"
QUEUE_COUNTER = 1

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

def create_instruction_queue(instructions):
    global ROBOT_ORIENT
    instr = instructions.split(" ")
    ROBOT_ORIENT = instr[0]
    return instr


def random_left_right():
    return random.choice(["left","right"])

def random_dir():
    return random.choice(["left","right", "forward", "backward"])

def translate_dir(new_dir):
    global ROBOT_ORIENT
    if (ROBOT_ORIENT == "right"):
        if (new_dir == "right"):
            return "forward"
        if (new_dir == "left"):
            ROBOT_ORIENT = "left"
            return "backward"
        if (new_dir == "up"):
            ROBOT_ORIENT = "up"
            return "left"
        if (new_dir == "down"):
            ROBOT_ORIENT = "down"
            return "right"

    if (ROBOT_ORIENT == "left"):
        if (new_dir == "right"):
            ROBOT_ORIENT = "right"
            return "backward"
        if (new_dir == "left"):
            return "forward"
        if (new_dir == "up"):
            ROBOT_ORIENT = "up"
            return "right"
        if (new_dir == "down"):
            ROBOT_ORIENT = "down"
            return "left"
    
    if (ROBOT_ORIENT == "up"):
        if (new_dir == "right"):
            ROBOT_ORIENT = "right"
            return "right"
        if (new_dir == "left"):
            ROBOT_ORIENT = "left"
            return "left"
        if (new_dir == "up"):
            return "forward"
        if (new_dir == "down"):
            ROBOT_ORIENT = "down"
            return "backward"

    if (ROBOT_ORIENT == "down"):
        if (new_dir == "right"):
            ROBOT_ORIENT = "right"
            return "left"
        if (new_dir == "left"):
            ROBOT_ORIENT = "left"
            return "right"
        if (new_dir == "up"):
            ROBOT_ORIENT = "up"
            return "backward"
        if (new_dir == "down"):
            return "forward"

def DEBUG_PRINT(dir, orient, instr_num):
    print("**********************")
    print("DIRECTION: "+dir)
    print("ORIENTATION: "+ orient)
    print("INSTRUCTION NUMBER: "+str(instr_num)+"\n")

def DRIVE_IN_DIRECTION(dir):
    if dir == "left":
        base.straight(straight_distance)
        base.turn(-ninety_degree_turn)
        base.straight(straight_distance)
    elif dir == "right":
        base.straight(straight_distance)
        base.turn(ninety_degree_turn)
        base.straight(straight_distance)
    elif dir == "backward":
        base.straight(backward_distance)
        base.turn(ninety_degree_turn*2)
    elif dir == "forward":
        base.straight(straight_distance)



QUEUE = create_instruction_queue(INSTRUCTION_STRING)

while (True):
    base.drive(132, 0) 
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

    if (center_is_black and (right_is_black or left_is_black)):
        direction = translate_dir(QUEUE[QUEUE_COUNTER])
        DEBUG_PRINT(direction, ROBOT_ORIENT, QUEUE_COUNTER)
        QUEUE_COUNTER += 1
        DRIVE_IN_DIRECTION(direction)