import matplotlib.pyplot as plt
import numpy as np
import math
from math import atan
from math import cos
from math import sin
from math import pi
import random

# fancy matplotlib plotting

def make_straight_line(start_xy, end_xy, num_points):
    x_vals = np.linspace(start_xy[0], end_xy[0], num_points)
    y_vals = np.linspace(start_xy[1], end_xy[1], num_points)

    path = np.hstack((x_vals.reshape(-1, 1), y_vals.reshape(-1, 1)))
    return path

def sigmoidFunction(s, a, x):
    return s/(1 + math.exp(-a*(x)))

def calculateObstacleOffset(s, a, dist):
    offset = sigmoidFunction(s, a, dist)
    return offset

def alterTargetForObstacle(left, s, a, dist, temp_target):
    angle = 0
    if left:
        angle = pi
    offset = calculateObstacleOffset(s, a, dist)
    sigmoid_target = [temp_target[0] + offset*cos(angle), temp_target[1] + offset*sin(angle)]
    return sigmoid_target



num_t = 10  # parameterize number of time steps

# make overall bounds and path
left_bound = make_straight_line((-4, 0), (-4, 20), num_t)
right_bound = make_straight_line((4, 0), (4, 20), num_t)
global_optimal_path = make_straight_line((0, 0), (0, 20), num_t)

# make actor cars
actor_1 = make_straight_line((-2, 10), (-2, 20), num_t)

actor_2_possibilities = [[(2, 10), (-1, 20)], [(-1, 10), (0, 20)], [(0.2, 10), (0.2, 20)]]
actor_2_decision = random.choice(actor_2_possibilities)
actor_2 = make_straight_line(actor_2_decision[0], actor_2_decision[1], num_t)

ego_path = [(0, 0)]  # initialize ego


for k in range(num_t):  # loop over time steps
    # time step data
    left_bound_k = left_bound[k]
    right_bound_k = right_bound[k]
    global_optimal_path_k = global_optimal_path[k]
    actor_1_k = actor_1[k]
    actor_2_k = actor_2[k]
    #     print(left_bound_k, right_bound_k, global_optimal_path_k, actor_1_k, actor_2_k)

    # plan optimal path
    ego_path.append((ego_path[-1][0], ego_path[-1][1] + 1))  # append next point

# get results
ego_path = np.asarray(ego_path)


sigmoid_path_1 = []
sigmoid_path_2 = []
for k in range(num_t):  # loop over time steps again to find optimal trajectory
    sigmoid_path_1 = []
    sigmoid_path_1.append([actor_1[k][0], actor_1[k][1] - 10])
    temp_path_1 = make_straight_line((sigmoid_path_1[0][0], sigmoid_path_1[0][1]), actor_1[k], num_t)

    sigmoid_path_2 = []
    sigmoid_path_2.append([actor_2[k][0], actor_2[k][1] - 10])
    temp_path_2 = make_straight_line((sigmoid_path_2[0][0], sigmoid_path_2[0][1]), actor_2[k], num_t)

    ego_path = []
    for i in range(num_t): # double iterate to draw avoidance paths
        temp_target = temp_path_1[i]
        left = False
        if global_optimal_path[k][0] < actor_1[k][0]:
            left = True
        print(left)
        target = alterTargetForObstacle(left, 0.5, 0.75, -(actor_1[k][1] - temp_target[1]) + 5, temp_target)
        sigmoid_path_1.append(target)

        left = False
        if global_optimal_path[k][0] < actor_2[k][0]:
            left = True
        temp_target = temp_path_2[i]
        target = alterTargetForObstacle(left, 0.5, 0.75, -((actor_2[k][1] - temp_target[1])) + 5, temp_target)
        sigmoid_path_2.append(target)
        
    for j in range(num_t):
        if (sigmoid_path_1[j][0] <= global_optimal_path[j][0] and global_optimal_path[j][0] <= actor_1[k][0]) or (sigmoid_path_1[j][0] >= global_optimal_path[j][0] and global_optimal_path[j][0] >= actor_1[k][0]):
            ego_path.append(sigmoid_path_1[j])
        elif (sigmoid_path_2[j][0] <= global_optimal_path[j][0] and global_optimal_path[j][0] <= actor_2[k][0]) or (sigmoid_path_2[j][0] >= global_optimal_path[j][0] and global_optimal_path[j][0] >= actor_2[k][0]):
            ego_path.append(sigmoid_path_2[j])
        else:
            ego_path.append(global_optimal_path[j])
    
    ego_path = np.array(ego_path)
    sigmoid_path_1 = np.array(sigmoid_path_1)
    sigmoid_path_2 = np.array(sigmoid_path_2)
    #plt.plot(sigmoid_path_1[:, 0], sigmoid_path_1[:, 1], color="green", marker="o")
    #plt.plot(sigmoid_path_2[:, 0], sigmoid_path_2[:, 1], color="green", marker="o")
    plt.plot(ego_path[:, 0], ego_path[:, 1], color="green", marker="o")




# plot results
plt.plot(left_bound[:, 0], left_bound[:, 1], color="k")
plt.plot(right_bound[:, 0], right_bound[:, 1], color="k")
plt.plot(global_optimal_path[:, 0], global_optimal_path[:, 1], color="blue", marker="o", alpha=0.25)
plt.plot(actor_1[:, 0], actor_1[:, 1], color="red", marker="o")
plt.plot(actor_2[:, 0], actor_2[:, 1], color="red", marker="o")
#plt.plot(ego_path[:, 0], ego_path[:, 1], color="orange", marker="o")

plt.show()