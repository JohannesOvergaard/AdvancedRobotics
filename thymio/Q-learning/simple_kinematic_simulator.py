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
action_space = np.array([(50,200),(200,50),(200,200)], dtype="i,i") #Turn left, turn right, forward
state_space = np.array(["OL", "OR", "OF", "NO"]) #OL, OR, OF, NO

Q = np.zeros((len(state_space),len(action_space)))
print (Q)

epsilon = 0.2
decay = 0.99
lr = 0.99
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
    
        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep

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
    if (sensor_values[0] < min_distance_wall):
        return state_space[0]
    if (sensor_values[4] < min_distance_wall):
        return state_space[1]
    if (sensor_values[1] < min_distance_wall or sensor_values[2] < min_distance_wall or sensor_values[3] < min_distance_wall):
        return state_space[2]
    else:
        return state_space[3]

def explore_or_exploit(state):
    global epsilon
    epsilon = epsilon*decay
    if uniform(0,1) < epsilon:  #Explore
       return np.random.choice(action_space)
    else:                       #Exploit
       return get_max_reward(state)

def get_max_reward(state):
    s = np.where(state_space == state)[0][0]
    print (s)
    a = np.argmax(Q[s,:])
    print(a)
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
    print (new_state)
    reward = get_reward(state)
    print (reward)

    updateQ(state,action,reward,new_state)

    return new_state

def get_reward(state):
    if (state == "NO"):
        return 5
    else:
        return -1


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
print (state)

for cnt in range(500):
    action = explore_or_exploit(state)
    print (action)

    state = do_action(state, action)

    #check collision with arena walls 
    if (world.distance(Point(x,y))<L/2):
        break
        
    if cnt%5==0:
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
    
