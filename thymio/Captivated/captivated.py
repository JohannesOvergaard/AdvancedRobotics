#!/usr/bin/python3
import os
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import numpy as np
import matplotlib.pyplot as plt
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

# PROGRAM VARIABLES 
DEBUG = False

#signal scanning thread to exit
exit_now = False

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)
#This is where we store the lidar readings
scan_data = [0]*360
x = 0.0
y = 0.0
theta = 0.0
pos = (x, y, theta)
camera = PiCamera()
IMG_WIDTH = 320
IMG_HEIGHT = 240

image = np.empty((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
apriltags = []
#the apriltags in the world
NORTH = [2, 3, 4, 5, 6]
EAST = [7, 8, 9]
SOUTH = [10, 11, 12, 13, 14]
WEST = [15, 0, 1]
#World dimensions in mm, the superior SI unit:
H = 1140
W = 1920



#NOTE: if you get adafruit_rplidar.RPLidarException: Incorrect descriptor starting bytes
# try disconnecting the usb cable and reconnect again. That should fix the issue

#if any value in scan_data = 0, it should not become minimum dist.
def punish_zeros():
    for idx, val in enumerate(scan_data):
        if val == 0:
            scan_data[idx] = 999999999

def set_up():
    global x,y, theta, pos
    sleep(15) #wait for lidarScan on background thread
    punish_zeros()
    index = np.argmin(scan_data)
    #find min dist and the left and right walls
    min_dist = scan_data[index]
    inv_min_dist = scan_data[(index+180)%359]
    right = scan_data[(index+90)%359]
    left = scan_data[abs(index-90)%359]
    print(f"index: {index}, min_dist: {min_dist}, left: {left}, right: {right}")
    #determine quadrant
    findAprilTag()
    april_to_wall_translation(left, right, min_dist, inv_min_dist)
    find_theta(min_dist)
    print(f"x: {x}, y: {y}, theta: {theta}")
    sys.exit()

def find_theta(min_dist):
    forward = scan_data[179] #straight ahead
    #TODO: START HERE FIX THIS

def april_to_wall_translation(left, right, min_dist, inv_min_dist):
    global x,y
    if (not apriltags):
        sys.exit()
    april = apriltags[0].tag_id
    print(april)
    forward = scan_data[179]
    backward = scan_data[0]
    if (min_dist + inv_min_dist <= H):
        print("LONG WALL")
        #min_dist is to a long wall
        if april in NORTH:
            if (forward > backward):
                print("april north, forward > backward")
            #lower 
                x = right
                y = H - min_dist
            else:
                print("april north, forward < backward")
            #upper
                x = left
                y = H - min_dist
        if april in EAST:
            if (forward > backward):
                print("april east, forward > backward")
            #upper 
                x = right
                y = H - min_dist
            else:
                print("april east, forward < backward")
            #lower
                x = right
                y = min_dist
        if april in SOUTH:
            if (forward > backward):
                print("april south, forward > backward")
            #upper
                x = left
                y = H - min_dist
            else: 
                print("april south, forward < backward")
                x = right
                y = min_dist
        if april in WEST:
            #lower
            if (forward > backward):
                print("april west, forward > backward")
                x = left
                y = H - min_dist
            else:
                print("april west, forward < backward")
                x = right
                y = H - min_dist
    else: 
        #min_dist is to a short wall
        print("SHORT WALL")
        if april in NORTH:
            if (forward > backward):
                print("april north, forward > backward")
            #lower 
                x = min_dist
                y = right
            else:
                print("april north, forward < backward")
            #upper
                x = W - min_dist
                y = left
        if april in EAST:
            if (forward > backward):
                print("april east, forward > backward")
            #upper 
                x = min_dist
                y = left
            else:
                print("april east, forward < backward")
            #lower
                x = W - min_dist 
                y = right
        if april in SOUTH:
            if (forward > backward):
                print("april south, forward > backward")
            #upper
                x = H - min_dist
                y = right
            else: 
                print("april south, forward < backward")
                x = W - min_dist
                y = right
        if april in WEST:
            #lower
            if (forward > backward):
                print("april west, forward > backward")
                x = W - min_dist
                y = left
            else:
                print("april west, forward < backward")
                x = H - min_dist
                y = left
             


def lidarScan():
    global scan_data
    print("Starting background lidar scanning")
    for scan in lidar.iter_scans():
        if(exit_now):
            return
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance

scanner_thread = Thread(target=lidarScan)
scanner_thread.daemon = True
scanner_thread.start()

sleep(1)

def takePhotos():
    global image
    camera.start_preview()
    while not exit_now:
        print("Camera test")
        sleep(5)
        #we capture to openCV compatible format
        #you might want to increase resolution
        camera.resolution = (IMG_WIDTH, IMG_HEIGHT)
        camera.framerate = 24
        sleep(2)
        image = np.empty((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        camera.capture(image, 'bgr')
        image = cv2.rotate(image, cv2.ROTATE_180)
        cv2.imwrite('out.png', image) 
    camera.stop_preview()

def findAprilTag():
    global apriltags
    img = cv2.imread('out.png',cv2.IMREAD_GRAYSCALE)     
    detector = apriltag.Detector()
    apriltags = detector.detect(img) #list of apriltags detected in image
    #print(results)

camera_thread = Thread(target=takePhotos)
camera_thread.daemon = True
camera_thread.start()

# def lidarScan():
#     print("Starting background lidar scanning")
#     test = lidar.iter_scans()
#     # print(f"Length of scan: {len(test)}")
#     for scan in test:
#         if(exit_now):
#             return
#         if DEBUG:
#             print(f"Length of scan: {len(scan)}")
#         for (_, angle, distance) in scan:
#             # if DEBUG:
#             #     print(f"Angle:{angle} Distance:{distance}")
#             scan_data[min([359, floor(angle)])] = distance
#     print(finished)
#     return scan_data

class Thymio:
    def __init__(self):
        PORT_NAME = '/dev/ttyUSB0'

        self.aseba = self.setup()
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

    # def do_lidar_scan(self):
    #     scan_data = scan_data = [0]*360
    #     for scan in self.lidar.iter_scans():
    #         if(exit_now):
    #             return
    #         for (_, angle, distance) in scan:
    #             scan_data[min([359, floor(angle)])] = distance     
    #     return scan_data
        
    # def take_photo(self, count):
    #     print("Camera test")
    #     self.camera.start_preview()
    #     sleep(2)
    #     #we capture to openCV compatible format
    #     #you might want to increase resolution
    #     self.camera.resolution = (320, 240)
    #     self.camera.framerate = 24
    #     sleep(2)
    #     image = np.empty((240, 320, 3), dtype=np.uint8)
    #     self.camera.capture(image, 'bgr')
    #     cv2.imwrite(f'Thymio_image_{count}.png', image)
    #     self.camera.stop_preview()
    #     print("saved image to out.png")


############## Bus and aseba setup ######################################

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
    global scan_data
    count = 1
    left_wheel_velocity = 0
    right_wheel_velocity = 0 
    min_distance_wall = 70
    set_up()
    while count < 3000:
        prox_val = robot.sens_horizontal()
        light_sensor_val = robot.sens_vertical()
        #robot.take_photo(count)
        #print(scan_data)
        if DEBUG:
            print(f"Left light: {light_sensor_val[0]}   and    Right light: {light_sensor_val[1]}")

        s0_dist = robot.convert_sensor_value_to_distance(prox_val[0], "s0")
        s1_dist = robot.convert_sensor_value_to_distance(prox_val[1], "s1")
        s2_dist = robot.convert_sensor_value_to_distance(prox_val[2], "s2")
        s3_dist = robot.convert_sensor_value_to_distance(prox_val[3], "s3")
        s4_dist = robot.convert_sensor_value_to_distance(prox_val[4], "s4")

        if (s2_dist < min_distance_wall):
            left_wheel_velocity = random.randrange(200, 400)
            right_wheel_velocity = -random.randrange(200, 400)
        elif (s0_dist < min_distance_wall or s1_dist < min_distance_wall):
            left_wheel_velocity = 200
            right_wheel_velocity = -200
        elif (s3_dist < min_distance_wall or s4_dist < min_distance_wall):
            left_wheel_velocity = -200
            right_wheel_velocity = 200
        else:                
            if count%100==0:
                left_wheel_velocity = random.randrange(200, 400)
                right_wheel_velocity = random.randrange(200, 400)

        robot.drive(left_wheel_velocity,right_wheel_velocity )
        count = count + 1

        
    robot.stop()
    
    # robot.drive(200, 200)


#----------------- loop end ---------------------



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
        print(f"{sys.exc_info()[0]}: {sys.exc_info()[1]}")
        print("Stopping robot")
        exit_now = True
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
