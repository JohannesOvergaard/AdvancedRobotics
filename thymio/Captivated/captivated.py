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

cell_size = 60

#print(belief)
#print(f' Size: {belief.size} \n Width: {belief.size/(Hcm/cell_size)} \n Height: {belief.size/(Wcm/cell_size)}')
#print((sum(sum(belief))))

#World
world = np.array([[0]*int(W/cell_size)]*int(H/cell_size))


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
    min_index = np.argmin(scan_data)
    #find min dist and the left and right walls
    min_dist = scan_data[min_index]
    inv_min_dist = scan_data[(min_index+180)%359]
    right = scan_data[(min_index+90)%359]
    left = scan_data[(min_index+270)%359]
    print(f"index: {min_index}, min_dist: {min_dist}, left: {left}, right: {right}")
    #determine quadrant
    findAprilTag()
    closest_wall = april_to_wall_translation(min_dist, inv_min_dist, min_index)
    find_theta(closest_wall, min_index)
    find_position(closest_wall,left,right,min_dist)
    print(f"x: {x}, y: {y}, theta: {theta}")
    #sys.exit()

def find_theta(closest_wall, min_index):
    global theta
    if (closest_wall == "north"):
        print (f'closest wall: {closest_wall}')
        theta = (360-min_index+180)%359
    elif (closest_wall == "east"):
        print (f'closest wall: {closest_wall}')
        theta = (360-min_index+270)%359
    elif (closest_wall == "south"):
        print (f'closest wall: {closest_wall}')
        theta = (360-min_index)%359
    elif (closest_wall == "west"):
        print (f'closest wall: {closest_wall}')
        theta = (360-min_index+90)%359

def find_position(closest_wall, left, right, min_dist):
    global x,y
    if (closest_wall == "north"):
        x = left
        y = H - min_dist
    elif (closest_wall == "east"):
        x = W - min_dist
        y = right
    elif (closest_wall == "south"):
        x = right
        y = min_dist  
    elif (closest_wall == "west"):
        x = min_dist
        y = left

def april_to_wall_translation(min_dist, inv_min_dist, min_index):
    if (not apriltags):
        return "None"
    april = apriltags[0].tag_id
    print(april)
    if (min_dist + inv_min_dist <= H):
        print("LONG WALL")
        #min_dist is to a long wall
        if april in NORTH:
            if (90 < min_index < 270):
                return "north"
            else:
                return "south"
        if april in EAST:
            if (min_dist <= 179):
                return "north"
            else:
                return "south"
        if april in SOUTH:
            if (min_index < 89 or 270 < min_index):
                return "north"
            else:
                return "south"
        if april in WEST:
            if (min_index >= 180):
                return "north"
            else:
                return "south"
    else: 
        #min_dist is to a short wall
        print("SHORT WALL")
        if april in NORTH:
            if (min_index <= 179):
                return "west"
            else:
                return "east"
        if april in EAST:
            if (min_index < 89 or 270 < min_index):
                return "west"
            else:
                return "east"
        if april in SOUTH:
            if (min_index >= 180):
                return "west"
            else:
                return "east"
        if april in WEST:
            if (90 < min_index < 270):
                return "west"
            else:
                return "east"

def lidarScan():
    global scan_data
    print("Starting background lidar scanning")
    for scan in lidar.iter_scans():
        if(exit_now):
            lidar.stop()
            lidar.disconnect()
            return
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance

scanner_thread = Thread(target=lidarScan)
scanner_thread.daemon = True
scanner_thread.start()
sleep(1)

# Create thread to update world
def UpdateWorld(light_sensor_values): # Index 0 = left      Index 1 = right
    global world
    # Take snapshot of current Lidar readings
    current_scan = scan_data[:]

    # Calculate estimated pos from snapshot data
    min_index = np.argmin(current_scan)

    #find min dist and the left and right walls
    min_dist = current_scan[min_index]
    inv_min_dist = current_scan[(min_index+180)%359]
    right = current_scan[(min_index+90)%359]
    left = current_scan[(min_index+270)%359]

    #determine quadrant
    findAprilTag()
    closest_wall = april_to_wall_translation(min_dist, inv_min_dist, min_index)
    if (not (closest_wall == 'None')):
        find_theta(closest_wall, min_index)
        find_position(closest_wall,left,right,min_dist)

        if (light_sensor_values[0] < 300 or light_sensor_values[1] < 300): # If black tag detected
            print(f"x: {x} y: {y}")
            world[int(y/cell_size)-1][int(x/cell_size)-1] = 1
            # print(f"Update pos thread - x: {int(x)}, y: {int(y)}, theta: {theta}")

# world_thread = Thread(target=UpdateWorld)
# world_thread.daemon = True
# sleep(1)

# def livePlotting():
#     global x, y, theta
#     sleep(20)
#     cnt = 0
#     #Live plotting
#     fig, ax = plt.subplots(figsize=((W/100)+10, (H/100)+10))
#     ax.axis([0-0.1, (W/100)+0.1, 0-0.1, (H/100)+0.1])
#     ax.add_patch( Rectangle((0, 0),
#                             (W/100), (H/100),
#                             fc ='none', 
#                             ec ='g',
#                             lw = 3))
#     while not exit_now:
#         omega = ((2 * pi)/360) * theta
#         # print(f"Plotting live data x: {x/100} y: {y/100} omega: {omega}...")
#         ax.quiver((x/100), (y/100), cos(omega)*0.05, sin(omega)*0.05, width=0.005)
#         plt.draw() 
#         plt.pause(1.0)


# plot_thread = Thread(target=livePlotting)
# plot_thread.daemon = True
# plot_thread.start()
# sleep(1)

def takePhotos():
    global image
    camera.start_preview()
    while not exit_now:
        sleep(5)
        #we capture to openCV compatible format
        #you might want to increase resolution
        camera.resolution = (IMG_WIDTH, IMG_HEIGHT)
        camera.framerate = 24
        image = np.empty((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        camera.capture(image, 'bgr')
        image = cv2.rotate(image, cv2.ROTATE_180)
        # print(f"saving image...")
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
    global x, y, theta, scan_data
    count = 1
    left_wheel_velocity = 0
    right_wheel_velocity = 0 
    min_distance_wall = 150
    set_up()                                    # Find global pos
    while count < 5000:
        prox_val = robot.sens_horizontal()
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
                left_wheel_velocity = random.randrange(200, 800)
                right_wheel_velocity = random.randrange(200, 800)
        
        if(count % 50 == 0):
            light_sensor_val = robot.sens_vertical()
            print(light_sensor_val)
            UpdateWorld(light_sensor_val)
            print(world)

        robot.drive(left_wheel_velocity,right_wheel_velocity)
        count = count + 1

        
    robot.stop()
    exit_now = True
    
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
        print(f"{sys.exc_info()[0]}: {sys.exc_info()[1]} : {sys.exc_info()[2]}")
        print("Stopping robot")
        exit_now = True
        sleep(5)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
