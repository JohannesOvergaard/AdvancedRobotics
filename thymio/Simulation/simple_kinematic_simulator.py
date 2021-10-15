from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
import numpy as np
from random import random, uniform
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# style.use('fivethirtyeight')

# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)

# def animate(i):
#     graph_data = open('example.txt','r').read()
#     lines = graph_data.split('\n')
#     xs = []
#     ys = []
#     for line in lines:
#         if len(line) > 1:
#             x, y = line.split(',')
#             xs.append(float(x))
#             ys.append(float(y))
#     ax1.clear()
#     ax1.plot(xs, ys)

# ani = animation.FuncAnimation(fig, animate, interval=1000)
# plt.show()

# A prototype simulation of a differential-drive robot with one sensor

# Constants
###########
R = 0.021  # radius of wheels in meters
L = 0.093  # distance between wheels in meters

W = 1.92  # width of arena
H = 1.14  # height of arena
Wcm = round(W*100)
Hcm = round(H*100)

robot_timestep = 0.1        # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])

# Variables 
###########
min_distance_wall = 0.07 #in meters

x = uniform(min_distance_wall-W/2,W/2-min_distance_wall)     # robot position in meters - x direction - positive to the right 
y = uniform(min_distance_wall-H/2,H/2-min_distance_wall)     # robot position in meters - y direction - positive up
q = 0.0     # robot heading with respect to x-axis in radians 

left_wheel_velocity =  random()   # robot left wheel velocity in radians/s
right_wheel_velocity =  random()  # robot right wheel velocity in radians/s

#  Bayesian filter
##################
#initialize belief
cell_size = 2
belief = np.array([[1/((Wcm/cell_size)*(Hcm/cell_size))]*int(Wcm/cell_size)]*int(Hcm/cell_size))
print(belief)
print(f' Size: {belief.size} \n Width: {belief.size/(Hcm/cell_size)} \n Height: {belief.size/(Wcm/cell_size)}')
print((sum(sum(belief))))

#map
map = np.array([[0]*int(Wcm/cell_size)]*int(Hcm/cell_size))

# Laser Scan
############
def getLaserScans(resolution=360):
    full_scan = []
    
    # fhere we emulate the lidar
    for i in range(resolution+1):
        added_angle = i * ((2*pi)/resolution)
        #Lider turns clockwise
        current_angle = (q - added_angle) % (2*pi)
        ray = LineString([(x, y), (x+cos(current_angle)*(W+H),(y+sin(current_angle)*(W+H))) ])
        s = world.intersection(ray)
        
        # the individual ray distances is what would get from your lidar sensors
        distance = sqrt((s.x-x)**2+(s.y-y)**2)
        
        #here the intersect coords are calculated using the current angle and the measured distance
        x_coord = x + cos(current_angle) * distance
        y_coord = y + sin(current_angle) * distance 
        #full_scan.append((float(x_coord), float(y_coord))) #
        full_scan.append(distance)
    return full_scan

def laserScanToPosition(scan):
    min_dist = round(min(scan),3)
    min_index = np.argmin(scan)
    print(f'Minimum: Distance: {min_dist}, Index: {min_index}')
    right_dist = round(scan[(min_index+90)%360],3)
    print(f'Right: Distance: {right_dist}, Index: {(min_index+90)%360}')
    left_dist = round(scan[(min_index-90)%360],3)
    print(f'Left: Distance: {left_dist}, Index: {(min_index-90)%360}')
    full_side = right_dist + left_dist
    if (W-0.05 < full_side and full_side < W+0.05):
        nearest_side = "North/South"
    else:
        nearest_side = "West/East"
    print(nearest_side)

    #We know distance to the nearest wall and wether it is a north/south or a east/west side.
    #Further, we know the distance to the walls 90 degrees to left and right.
    #The orientation of the robot can be found by looking at the index zero of the scan and comparing it to the index of the minimum distance to a wall

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
ax.axis([-W/2-0.5, W/2+0.5, -H/2-0.5, H/2+0.5])
ax.add_patch( Rectangle((-W/2, -H/2),
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
#[ax.plot(x,y, 'bo') for (x,y) in getLaserScans()] 

laserScanToPosition(getLaserScans())

print(round(x,3),round(y,3))
ax.plot(x,y, "ro")
plt.draw()
plt.show(block=True)

