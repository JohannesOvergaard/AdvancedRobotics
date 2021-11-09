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

W = 1.92/2  # width of arena
H = 1.14/2  # height of arena

robot_timestep = 0.1        # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W,H),(0,H),(0,0),(W,0)])

# Variables 
###########
min_distance_wall = 0.07

x = uniform(min_distance_wall,W-min_distance_wall)     # robot position in meters - x direction - positive to the right 
y = uniform(min_distance_wall,H-min_distance_wall)     # robot position in meters - y direction - positive up
q = uniform(0.0,2*pi)    # robot heading with respect to x-axis in radians 

left_wheel_velocity =  random()   # robot left wheel velocity in radians/s
right_wheel_velocity =  random()  # robot right wheel velocity in radians/s

#Q-learning
action_space = np.array([(-1,1),(1,-1),(1,1)], dtype="i,i") #Turn left, turn right, forward
state_space = np.array(["OL", "OR", "OF", "NO"]) #OL, OR, OF, NO

Q = np.zeros((len(state_space),len(action_space)))
print (Q)

epsilon = 1
decay = 0.999
lr = 0.95
gamma = 0.9

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
    
        xx = x + v_x * simulation_timestep
        yy = y + v_y * simulation_timestep
        q += omega * simulation_timestep
        
        if (0<xx<W and 0<yy<H):
            x = xx
            y = yy

def setup():
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

    return sensor_to_state([s0_dist,s1_dist,s2_dist,s3_dist,s4_dist])

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

def do_action(state, action):
    global left_wheel_velocity, right_wheel_velocity, x, y, q

    left_wheel_velocity, right_wheel_velocity = action
    simulationstep()

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

    new_state = sensor_to_state([s0_dist,s1_dist,s2_dist,s3_dist,s4_dist])
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
    # i = 0
    # a = np.where(action_space == action)
    # if (a[0][0] == 2):
    #     i = 10
    # if ((state == "OL" or state == "OR" or state == "OF") and new_state == "NO"):
    #     return 100+i
    # elif (new_state == "NO"):
    #     return i
    # elif (state == new_state):
    #     return -1
    # elif (new_state == "OL" or new_state == "OR" or new_state == "OF"):
    #     return -5
    # else:
    #     return 0


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

state = setup()

for cnt in range(10000):
    action = explore_or_exploit(state)

    state = do_action(state, action)

    #check collision with arena walls 
    # if (world.distance(Point(x,y))<L/2):
    #     break
        
    if cnt%50==0:
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q-0.7)*0.05) + ", " + str(sin(q-0.7)*0.05) + "\n")
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q-0.35)*0.05) + ", " + str(sin(q-0.35)*0.05) + "\n")
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q)*0.05) + ", " + str(sin(q)*0.05) + "\n")
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
print(Q)
file.close()
plt.show(block=True)
    
