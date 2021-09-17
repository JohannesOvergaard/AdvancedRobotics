#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import random



straight_distance = 65
threshold = 25 #threshold for determining if black or silver
ninety_degree_turn = 84
ROBOT_POS = ""
INSTRUCTION_STRING = "right right down up right right down down left left left right right right down down left left left left up up right right down up left left down down right right right"
QUEUE_COUNTER = 0

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
    global ROBOT_POS
    instr = instructions.split(" ")
    ROBOT_POS = instr[0]
    return instr


def random_left_right():
    return random.choice(["left","right"])

def random_dir():
    return random.choice(["left","right", "forward", "backward"])

def translate_dir(new_dir):
    global ROBOT_POS
    if (ROBOT_POS == "right"):
        if (new_dir == "right"):
            return "forward"
        if (new_dir == "left"):
            ROBOT_POS = "left"
            return "backward"
        if (new_dir == "up"):
            ROBOT_POS = "up"
            return "left"
        if (new_dir == "down"):
            ROBOT_POS = "down"
            return "right"

    if (ROBOT_POS == "left"):
        if (new_dir == "right"):
            ROBOT_POS = "right"
            return "backward"
        if (new_dir == "left"):
            return "forward"
        if (new_dir == "up"):
            ROBOT_POS = "up"
            return "right"
        if (new_dir == "down"):
            ROBOT_POS = "down"
            return "left"
    
    if (ROBOT_POS == "up"):
        if (new_dir == "right"):
            ROBOT_POS = "right"
            return "right"
        if (new_dir == "left"):
            ROBOT_POS = "left"
            return "left"
        if (new_dir == "up"):
            return "forward"
        if (new_dir == "down"):
            ROBOT_POS = "down"
            return "backward"

    if (ROBOT_POS == "down"):
        if (new_dir == "right"):
            ROBOT_POS = "right"
            return "left"
        if (new_dir == "left"):
            ROBOT_POS = "left"
            return "right"
        if (new_dir == "up"):
            ROBOT_POS = "up"
            return "backward"
        if (new_dir == "down"):
            return "forward"


 
QUEUE = create_instruction_queue(INSTRUCTION_STRING)

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
            #direction = random_dir()
            direction = translate_dir(QUEUE[QUEUE_COUNTER])
            QUEUE_COUNTER += 1
            if (direction == "left"): 
                base.turn(-ninety_degree_turn)
                continue
            elif (direction == "right"):
                base.turn(ninety_degree_turn)
                continue
            elif (direction == "forward"):
                continue
            elif (direction == "backward"):
                base.straight(-50)
                base.turn(ninety_degree_turn*2)
                continue
        else : # T-intersection
            direction = translate_dir(QUEUE[QUEUE_COUNTER])
            QUEUE_COUNTER += 1
            if (direction == "left"):
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
        else: #Left or straight
            direction = translate_dir(QUEUE[QUEUE_COUNTER])
            QUEUE_COUNTER += 1
            if (direction == "left"):
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
        else: #right or straight
            direction = translate_dir(QUEUE[QUEUE_COUNTER])
            QUEUE_COUNTER += 1
            if (direction == "right"):
                base.turn(ninety_degree_turn)
                continue
            else:
                continue