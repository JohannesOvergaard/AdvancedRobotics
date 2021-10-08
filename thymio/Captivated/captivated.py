#!/usr/bin/python3
import os
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import matplotlib.pyplot as plt
from time import sleep
import random
import sys
import dbus
import dbus.mainloop.glib
from threading import Thread


class Thymio:
    def __init__(self):
        self.aseba = self.setup()

    def drive(self, left_wheel_speed, right_wheel_speed):
        print("Left_wheel_speed: " + str(left_wheel_speed))
        print("Right_wheel_speed: " + str(right_wheel_speed))
        
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

    def sens_val(self):
        return self.aseba.GetVariable("thymio-II", "prox.horizontal")

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
    count = 1
    left_wheel_velocity = 0
    right_wheel_velocity = 0 
    min_distance_wall = 70
    while count < 10000:
        prox_val = robot.sens_val()
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
        print(sys.exc_info()[1])
        print("Stopping robot")
        exit_now = True
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
