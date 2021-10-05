import shapely
from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
from random import random

# A prototype simulation of a differential-drive robot with one sensor

# Constants
###########
R = 0.021  # radius of wheels in meters
L = 0.093  # distance between wheels in meters

W = 1.92  # width of arena
H = 1.14  # height of arena

robot_timestep = 0.1        # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])

# Variables 
###########

x = 0.0   # robot position in meters - x direction - positive to the right 
y = 0.0   # robot position in meters - y direction - positive up
q = 0.0   # robot heading with respect to x-axis in radians 

left_wheel_velocity =  random()   # robot left wheel velocity in radians/s
right_wheel_velocity =  random()  # robot right wheel velocity in radians/s

# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep():
    global x, y, q

    for step in range(int(robot_timestep/simulation_timestep)):     #step model time/timestep times
        v_x = cos(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2) 
        v_y = sin(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2)
        omega = (R*right_wheel_velocity - R*left_wheel_velocity)/(2*L)    
    
        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep

# Simulation loop
#################
file = open("trajectory.dat", "w")

for cnt in range(5000):
    #simple single-ray sensor
    ray_0 = LineString([(x, y), (x+cos(q+0.7)*2*W,(y+sin(q+0.7)*2*H)) ])
    ray_1 = LineString([(x, y), (x+cos(q+0.35)*2*W,(y+sin(q+0.35)*2*H)) ])
    ray_2 = LineString([(x, y), (x+cos(q)*2*W,(y+sin(q)*2*H)) ])
    ray_3 = LineString([(x, y), (x+cos(q-0.35)*2*W,(y+sin(q-0.35)*2*H)) ])
    ray_4 = LineString([(x, y), (x+cos(q-0.7)*2*W,(y+sin(q-0.7)*2*H)) ])  # a line from robot to a point outside arena in direction of q
    s0 = world.intersection(ray_0)
    s1 = world.intersection(ray_1)
    s2 = world.intersection(ray_2)
    s3 = world.intersection(ray_3)
    s4 = world.intersection(ray_4)
    distance0 = sqrt((s0.x-x)**2+(s0.y-y)**2)
    distance1 = sqrt((s1.x-x)**2+(s1.y-y)**2)
    distance2 = sqrt((s2.x-x)**2+(s2.y-y)**2)
    distance3 = sqrt((s3.x-x)**2+(s3.y-y)**2)
    distance4 = sqrt((s4.x-x)**2+(s4.y-y)**2)                    # distance to wall
    
    #simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then turn on spot
    if (distance2 < 0.15):
        left_wheel_velocity = -1
        right_wheel_velocity = 1
    elif (distance0 < 0.15 or distance1 < 0.15):
        left_wheel_velocity = 0.4
        right_wheel_velocity = -0.4
    elif (distance3 < 0.15 or distance4 < 0.15):
        left_wheel_velocity = -0.4
        right_wheel_velocity = 0.4
    else:                
        if cnt%100==0:
            left_wheel_velocity = 1#random()
            right_wheel_velocity = 1#random()
        
    #step simulation
    simulationstep()

    #check collision with arena walls 
    if (world.distance(Point(x,y))<L/2):
        break
        
    if cnt%50==0:
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q-0.7)*0.05) + ", " + str(sin(q-0.7)*0.05) + "\n")
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q-0.35)*0.05) + ", " + str(sin(q-0.35)*0.05) + "\n")
        file.write( str(x) + ", " + str(y) + ", " + str(cos(q)*0.05) + ", " + str(sin(q)*0.05) + "\n")
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q+0.35)*0.05) + ", " + str(sin(q+0.35)*0.05) + "\n")
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q+0.7)*0.05) + ", " + str(sin(q+0.7)*0.05) + "\n")

file.close()
    
