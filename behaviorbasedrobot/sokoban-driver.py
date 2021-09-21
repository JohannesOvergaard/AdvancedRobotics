#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, 
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
ev3.speaker.set_volume(100, which='_all_')

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
        base.turn(-ninety_degree_turn)
        base.straight(0.4 * straight_distance)
    elif dir == "right":
        base.turn(ninety_degree_turn)
        base.straight(0.4 * straight_distance)
    elif dir == "backward":
        base.straight(backward_distance)
        base.turn(ninety_degree_turn*2)
    elif dir == "forward":
        base.straight(straight_distance)

def PANIC():
    ev3.speaker.say("PANIC MODE ENGAGED")
    while(True):
        base.turn(ninety_degree_turn)
        ev3.speaker.say("HELP ME")
        

ev3.speaker.say("MODERATE INTEREST ENGAGED")
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
        base.straight(0.2 * straight_distance)
        right_is_black = right_color.reflection() < threshold
        left_is_black = left_color.reflection() < threshold

        if (center_is_black and right_is_black and left_is_black): #T-intersection or X-intersection
            base.straight(0.8 * straight_distance)
            center_is_black = center_color.reflection() < threshold
            if (center_is_black): # X-intersection
                # print("X-intersection")
                #direction = random_dir()
                direction = translate_dir(QUEUE[QUEUE_COUNTER])
                DEBUG_PRINT(direction, ROBOT_ORIENT, QUEUE_COUNTER)
                QUEUE_COUNTER += 1
                if (direction == "left"): 
                    DRIVE_IN_DIRECTION("left")
                    continue
                elif (direction == "right"):
                    #print("X-intersection-RIGHT")
                    DRIVE_IN_DIRECTION("right")
                    continue
                elif (direction == "forward"):
                    # print("X-intersection-FORWARD")
                    continue
                elif (direction == "backward"):
                    # print("X-intersection-BACK")
                    DRIVE_IN_DIRECTION("backward")
                    continue
            else : # T-intersection
                # print("T-intersection")
                direction = translate_dir(QUEUE[QUEUE_COUNTER])
                DEBUG_PRINT(direction, ROBOT_ORIENT, QUEUE_COUNTER)
                QUEUE_COUNTER += 1
                if (direction == "left"):
                    # print("T-section-LEFT")
                    DRIVE_IN_DIRECTION("left")
                    continue
                elif (direction == "backward"):
                    DRIVE_IN_DIRECTION("backward")
                    continue
                elif (direction == "right"):
                    # print("T-section-RIGHT")
                    DRIVE_IN_DIRECTION("right")
                    continue
                else:
                    PANIC()
        
        if (center_is_black and not right_is_black and left_is_black): #path left
            base.straight(0.8 * straight_distance)
            center_is_black = center_color.reflection() < threshold
            if (not center_is_black): #Turn left
                # print("TURN LEFT")
                direction = translate_dir(QUEUE[QUEUE_COUNTER])
                DEBUG_PRINT(direction, ROBOT_ORIENT, QUEUE_COUNTER)
                QUEUE_COUNTER += 1
                if(direction == "left"):
                    DRIVE_IN_DIRECTION("left")
                elif(direction == "backward"):
                    DRIVE_IN_DIRECTION("backward")
                continue
            else: #Left or straight or backward
                direction = translate_dir(QUEUE[QUEUE_COUNTER])
                DEBUG_PRINT(direction, ROBOT_ORIENT, QUEUE_COUNTER)
                QUEUE_COUNTER += 1
                if (direction == "left"):
                    # print("LEFT-STRAIGHT: LEFT")
                    DRIVE_IN_DIRECTION("left")
                    continue
                elif (direction == "backward"):
                    DRIVE_IN_DIRECTION("backward")
                    continue
                else:
                    DRIVE_IN_DIRECTION("forward")
                    continue

        if (center_is_black and right_is_black and not left_is_black): #path right
            base.straight(0.8 * straight_distance)
            center_is_black = center_color.reflection() < threshold
            if (not center_is_black): #Turn right
                # print("TURN RIGHT: RIGHT")
                direction = translate_dir(QUEUE[QUEUE_COUNTER])
                DEBUG_PRINT(direction, ROBOT_ORIENT, QUEUE_COUNTER)
                QUEUE_COUNTER += 1
                if(direction == "right"):
                    DRIVE_IN_DIRECTION("right")
                    continue
                elif(direction == "backward"):
                    DRIVE_IN_DIRECTION("backward")
                    continue
                continue
            else: #right or straight or backward
                direction = translate_dir(QUEUE[QUEUE_COUNTER])
                DEBUG_PRINT(direction, ROBOT_ORIENT, QUEUE_COUNTER)
                QUEUE_COUNTER += 1
                if (direction == "right"):
                    # print("RIGHT-STRAIGHT: RIGHT")
                    DRIVE_IN_DIRECTION("right")
                    continue
                elif (direction == "backward"):
                    DRIVE_IN_DIRECTION("backward")
                    continue
                else:
                    # print("RIGHT-STRAIGHT: STRAIGHT")
                    continue