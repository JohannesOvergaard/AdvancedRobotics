#!/usr/bin/python3
import os
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from time import sleep
import random
import cv2
import sys
import dbus
from math import cos, sin, pi, floor
import dbus.mainloop.glib
from adafruit_rplidar import RPLidar
from threading import Thread
from picamera import PiCamera
import apriltag

class Thymio:
    def __init__(self):
        PORT_NAME = '/dev/ttyUSB0'

        self.aseba = self.setup()
        self.aseba.SendEventName("leds.prox.h", [0 for i in range(8)])
        self.aseba.SendEventName("leds.prox.v", [0 for i in range(2)])
        self.aseba.SendEventName("leds.rc", [0])
        self.aseba.SendEventName("leds.sound", [0])
        self.aseba.SendEventName("leds.temperature", [0,0])
        #self.camera = PiCamera()
        #self.lidar = RPLidar(None, PORT_NAME)

    def drive(self, left_wheel_speed, right_wheel_speed):
        #print("Left_wheel_speed: " + str(left_wheel_speed))
        #print("Right_wheel_speed: " + str(right_wheel_speed))
        
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

    def sens_horizontal(self):
        return self.aseba.GetVariable("thymio-II", "prox.horizontal")

    def sens_vertical(self):
        return self.aseba.GetVariable("thymio-II", "prox.ground.reflected")

    def setup(self):
        print("Setting up")
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

            
        asebaNetwork.SendEventName( "prox.comm.enable", [1]) #enables communication to other thymios
        

        # scanning_thread = Process(target=robot.drive, args=(200,200,))
        return asebaNetwork

    def resetMessage(self):
        self.aseba.SendEventName("prox.comm.rx",[0])

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
    
    def sendInformation(self, number):
        self.aseba.SendEventName("prox.comm.tx", [number])

    def receiveInformation(self):   
        rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")
        return rx[0]

    def glow(self, colour):
        rgb = [0,0,0]
        if (colour == "orange"):
            rgb = [32, 10, 0]
        if (colour == "red"):
            rgb = [32, 0, 0]
        if (colour == "purple"):
            rgb = [16, 0, 32]
        if (colour == "green"):
            rgb = [0, 32, 0]
        if (colour == "blue"):
            rgb = [0, 0, 32]
        self.aseba.SendEventName("leds.top", rgb)
        self.aseba.SendEventName("leds.bottom.left", rgb)
        self.aseba.SendEventName("leds.bottom.right", rgb)
        
        
            

def mainLoop(robot):
    while (True):
        #robot.sendInformation(1)
        #print(robot.receiveInformation())
        colours = ["orange", "red", "green", "purple", "blue"]
        for i in colours:
            robot.glow(i)
            sleep(5)
        # robot.resetMessage()


if __name__ == '__main__':
    try:
        robot = Thymio()

        #robot.sens() 
        '''
        thread = Thread(target=robot.sens)
        thread.daemon = True
        thread.start()
        '''
        mainLoop(robot)
    except:
        print(f"{sys.exc_info()[0]}: {sys.exc_info()[1]} : {sys.exc_info()[2]}")
        print("Stopping robot")
        exit_now = True
        sleep(5)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

