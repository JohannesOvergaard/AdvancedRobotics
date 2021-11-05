from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
import numpy as np
from random import random, uniform
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

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

min_distance_wall = 0.07

x = 1.0 #uniform(min_distance_wall,W-min_distance_wall)     # robot position in meters - x direction - positive to the right 
y = 1.0 #uniform(min_distance_wall,H-min_distance_wall)     # robot position in meters - y direction - positive up
q = uniform(0.0,2*pi)    # robot heading with respect to x-axis in radians 

left_wheel_velocity =  random()   # robot left wheel velocity in radians/s
right_wheel_velocity =  random()  # robot right wheel velocity in radians/s

#Q-learning
#action_space = 

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

cm = 1/2.54
fig, ax = plt.subplots(figsize=(W*20*cm, H*20*cm))
ax.axis([0-0.1, W+0.1, 0-0.1, H+0.1])
ax.add_patch( Rectangle((0, 0),
                        W, H,
                        fc ='none', 
                        ec ='g',
                        lw = 3) )


for cnt in range(1000):
    #simple single-ray sensor
    ray_0 = LineString([(x, y), (x+cos(q+0.7)*(W+H),(y+sin(q+0.7)*(W+H))) ])
    ray_1 = LineString([(x, y), (x+cos(q+0.35)*(W+H),(y+sin(q+0.35)*(W+H))) ])
    ray_2 = LineString([(x, y), (x+cos(q)*(W+H),(y+sin(q)*(W+H))) ])
    ray_3 = LineString([(x, y), (x+cos(q-0.35)*(W+H),(y+sin(q-0.35)*(W+H))) ])
    ray_4 = LineString([(x, y), (x+cos(q-0.7)*(W+H),(y+sin(q-0.7)*(W+H))) ])  # a line from robot to a point outside arena in direction of q
    s0 = world.intersection(ray_0)
    s1 = world.intersection(ray_1)
    s2 = world.intersection(ray_2)
    s3 = world.intersection(ray_3)
    s4 = world.intersection(ray_4)
    s0_dist = sqrt((s0.x-x)**2+(s0.y-y)**2)
    s1_dist = sqrt((s1.x-x)**2+(s1.y-y)**2)
    s2_dist = sqrt((s2.x-x)**2+(s2.y-y)**2)
    s3_dist = sqrt((s3.x-x)**2+(s3.y-y)**2)
    s4_dist = sqrt((s4.x-x)**2+(s4.y-y)**2)                    # distance to wall
    
    #simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then turn on spot
    if (s2_dist < min_distance_wall):
        left_wheel_velocity = random()
        right_wheel_velocity = -random()
    elif (s0_dist < min_distance_wall or s1_dist < min_distance_wall):
        left_wheel_velocity = 0.5
        right_wheel_velocity = -0.5
    elif (s3_dist < min_distance_wall or s4_dist < min_distance_wall):
        left_wheel_velocity = -0.5
        right_wheel_velocity = 0.5
    else:                
        if cnt%100==0:
            left_wheel_velocity = random()
            right_wheel_velocity = random()  

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

        # Animate live robots calculated location in sim
        # ax.quiver(x, y, cos(q)*0.05, sin(q-0.7)*0.05) #s0
        # ax.quiver(x, y, cos(q)*0.05, sin(q-0.35)*0.05) #s1
        ax.quiver(x, y, cos(q)*0.05, sin(q)*0.05, width=0.005) #s2
        # ax.quiver(x, y, cos(q)*0.05, sin(q+0.35)*0.05) #s3
        # ax.quiver(x, y, cos(q)*0.05, sin(q+0.7)*0.05) #s4

        #[ax.plot(x,y, 'bo') for (x,y) in getLaserScans()] 
        
        plt.draw() 
        plt.pause(0.01) #is necessary for the plot to update for some reason
        

file.close()
plt.show(block=True)
    
