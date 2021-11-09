#!/usr/bin/python3
import os
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from time import sleep
from random import random, uniform
import cv2
import sys
import dbus
from math import cos, sin, pi, floor
import dbus.mainloop.glib
from adafruit_rplidar import RPLidar
from threading import Thread
from picamera import PiCamera
import apriltag

# PROGRAM VARIABLES 
DEBUG = False

#signal scanning thread to exit
exit_now = False

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
#This is where we store the lidar readings
x = 0.0
y = 0.0
theta = 0.0

#World dimensions in mm, the superior SI unit:
H = 1120
W = 1920
Diagonal = 2233
SPEED = 400
#Q-learning
action_space = np.array([(-SPEED,SPEED),(SPEED,-SPEED),(SPEED,SPEED)], dtype="i,i") #Turn left, turn right, forward
state_space = np.array(["OL", "OR", "OF", "NO"]) #OL, OR, OF, NO

Q = np.zeros((len(state_space),len(action_space)))
print (Q)

epsilon = 1
decay = 0.999
lr = 0.95
gamma = 0.9

min_distance_wall = 140

def convert_sensor_values_to_distance(robot, prox_val):
    s0_dist = robot.convert_a_sensor_value_to_distance(prox_val[0], "s0")
    s1_dist = robot.convert_a_sensor_value_to_distance(prox_val[1], "s1")
    s2_dist = robot.convert_a_sensor_value_to_distance(prox_val[2], "s2")
    s3_dist = robot.convert_a_sensor_value_to_distance(prox_val[3], "s3")
    s4_dist = robot.convert_a_sensor_value_to_distance(prox_val[4], "s4")
    return [s0_dist, s1_dist, s2_dist, s3_dist, s4_dist]

def sensor_to_state(sensor_values):
    min_dist = min(sensor_values)
    if (min_dist < min_distance_wall):
        sensor_index = sensor_values.index(min_dist)
        if (sensor_index == 0 or sensor_index == 1):
            return state_space[0]
        if (sensor_index == 3 or sensor_index == 4):
            return state_space[1]
        if (sensor_index == 2):
            return state_space[2]
    else:
        return state_space[3]

def explore_or_exploit(state):
    global epsilon
    epsilon = epsilon*decay
    if uniform(0,1) < epsilon:  #Explore
        return np.random.choice(action_space)
    else:                       #Exploit
        return get_max_action(state)

def get_max_action(state):
    s = np.where(state_space == state)[0][0]
    a = np.argmax(Q[s,:])
    return action_space[a]

def updateQ(state, action, reward, new_state):
    s = np.where(state_space == state)
    a = np.where(action_space == action)
    ns = np.where(state_space == new_state)
    Q[s,a] = Q[s,a] + lr * (reward + gamma * np.max(Q[ns,:]) - Q[s,a])

def do_action(robot, state, action):
    left_wheel_velocity, right_wheel_velocity = action
    robot.drive(left_wheel_velocity,right_wheel_velocity)
    prox_val = robot.sens_horizontal() # New proximity values

    # Convert to dist
    new_state = sensor_to_state(convert_sensor_values_to_distance(robot, prox_val))
    reward = get_reward(state, action)

    updateQ(state,action,reward,new_state)

    return new_state

def get_reward(state, action):
    i = 0
    a = np.where(action_space == action)
    if (a[0][0] == 2):  #Motivation to drive forward
        i = 5
    if (state == "NO"):
        return i
    else:
        return -1

class Thymio:
    def __init__(self):
        PORT_NAME = '/dev/ttyUSB0'

        self.aseba = self.setup()

    def drive(self, left_wheel_speed, right_wheel_speed):
        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed
        
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def sens(self):
        while True:
            prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
            print("Sensing:")
            print(prox_horizontal[0])
            print(prox_horizontal[1])
            print(prox_horizontal[2])
            print(prox_horizontal[3])
            print(prox_horizontal[4])

    def convert_a_sensor_value_to_distance(self, x, sensor_id):
        if sensor_id == "s0":
            return 0.000000000802*pow(x, 3) - 0.00000706*pow(x, 2) - 0.0163*x+165
        elif sensor_id == "s1":
            return 0.00000000198*pow(x, 3) - 0.0000192*pow(x, 2) + 0.0121*x + 182
        elif sensor_id == "s2":
            return 0.00000000303*pow(x, 3) - 0.0000255*pow(x, 2) + 0.019*x + 181
        elif sensor_id == "s3":
            return 0.00000000223*pow(x, 3) - 0.000019*pow(x, 2) + 0.00662*x + 175
        elif sensor_id == "s4":
            return 0.00000000199*pow(x, 3) - 0.0000174*pow(x, 2) + 0.0062*x + 170

    def sens_horizontal(self):
        return self.aseba.GetVariable("thymio-II", "prox.horizontal")

    def sens_vertical(self):
        return self.aseba.GetVariable("thymio-II", "prox.ground.reflected")

############## Bus and aseba setup ######################################

    def setup(self):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SessionBus()
        asebaNetworkObject = bus.get_object('ch.epfl.mobots.Aseba', '/')

        asebaNetwork = dbus.Interface(
            asebaNetworkObject, dbus_interface='ch.epfl.mobots.AsebaNetwork'
        )
        # load the file which is run on the thymio
        asebaNetwork.LoadScripts(
            'thympi.aesl', reply_handler=self.dbusError, error_handler=self.dbusError
        )

        # scanning_thread = Process(target=robot.drive, args=(200,200,))
        return asebaNetwork

    def stopAsebamedulla(self):
        os.system("pkill -n asebamedulla")

    def dbusReply(self):
        # dbus replys can be handled here.
        # Currently ignoring
        pass

    def dbusError(self, e):
        # dbus errors can be handled here.
        # Currently only the error is logged. Maybe interrupt the mainloop here
        print("dbus error: %s" % str(e))
        exit_now = True
        os.system("pkill -n asebamedulla")
        

    #------------------ Sensor val ---------------#
    def convert_sensor_value_to_distance(self, x, sensor_id):
        if sensor_id == "s0":
            return 0.000000000802*pow(x, 3) - 0.00000706*pow(x, 2) - 0.0163*x+165
        elif sensor_id == "s1":
            return 0.00000000198*pow(x, 3) - 0.0000192*pow(x, 2) + 0.0121*x + 182
        elif sensor_id == "s2":
            return 0.00000000303*pow(x, 3) - 0.0000255*pow(x, 2) + 0.019*x + 181
        elif sensor_id == "s3":
            return 0.00000000223*pow(x, 3) - 0.000019*pow(x, 2) + 0.00662*x + 175
        elif sensor_id == "s4":
            return 0.00000000199*pow(x, 3) - 0.0000174*pow(x, 2) + 0.0062*x + 170

#------------------- Main ------------------------

#------------------- loop ------------------------

def mainLoop(robot):
    count = 1
    left_wheel_velocity = 0
    right_wheel_velocity = 0 
    # Get initial sensor values
    state = sensor_to_state(convert_sensor_values_to_distance(robot, robot.sens_horizontal()))

    while count < 20000:
        action = explore_or_exploit(state)
        state = do_action(robot, state, action)
        count += 1
    print(Q)
    robot.stop()

    # robot.drive(200, 200)


#----------------- loop end ---------------------

if __name__ == '__main__':
    try:
        robot = Thymio()
        mainLoop(robot)
        os.system("pkill -n asebamedulla")

    except:
        print(f"{sys.exc_info()[0]}: {sys.exc_info()[1]} : {sys.exc_info()[2]}")
        print("Stopping robot")
        exit_now = True
        sleep(5)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
