
# Team Mates: Navjyot Purohit and Yash Kulkarni
# Github Link : https://github.com/navjyotp/A_Star

import numpy as np
import math as m
import heapq as hp
import cv2
import pygame
import time


x_start = int(input("Enter the x coordinate of the start:  "))
y_start = int(input("Enter the y coordinate of the start:  "))
start_orientation = int(input("Enter the Orientation at start :  "))
x_goal = int(input("Enter the x coordinate of the goal:  "))
y_goal = int(input("Enter the y coordinate of the goal:  "))
radius = int(input("Enter the radius of the robot:  "))
clear = int(input("Enter the clearance of the robot: "))
step_size = int(input("Enter the step (1-10): "))
start = [x_start, y_start]
goal = [x_goal, y_goal]


def check_obstacle(x, y):

    a, b, c, s = 0, 0, 0, 0
    if ((x - 310+clear)*(x - 300+clear)) + ((y + clear - 185)*(y + clear - 185)) <= (40*40):
        a = 1

    if x+clear <= 235 and x+clear >= 165 and (74*(x+clear) - 175*(y+clear) + 8825 >=0) and (74*(x+clear) + 175*(y+clear) - 38425 <=0) and (74*(x+clear) - 175*(y+clear) - 3425 <=0) and (74*(x+clear) + 175*(y+clear) - 26175 >=0):
        b = 1

    if ((25*(x+clear) - 79*(y+clear) + 13715 >= 0) and (6*(x+clear) - 7*(y+clear) + 780 <= 0) and (85*(x+clear) + 69*(y+clear) - 15825 >= 0)) or ((85*(x+clear) + 69*(y+clear) -15825>=0) and (16*(x+clear) + 5*(y+clear) - 2180<=0) and (25*(x+clear) - 79*(y+clear) + 13715 >=0)):
        c = 1

    s = a + b + c

    if s > 0:
        return True

    else:
        return False


def approx(x):
    if round(x)+0.5 == x:
        y = x
    elif round(x) < x:
        y = round(x)
    else:
        y = round(x)
    return y


def heuristic(a,b):
    x1, y1 = a[0], a[1]
    x2, y2 = b[0], b[1]

    distance = approx(m.sqrt((x2-x1)**2 + (y2-y1)**2))
    return distance

def move0(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(0)*step + x)
    yr = approx(np.sin(0)*step + y)
    new = (xr, yr)

    return new

def move30(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(30))*step + x)
    yr = approx(np.sin(np.deg2rad(30))*step + y)
    new = (xr, yr)

    return new

def move60(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(60))*step + x)
    yr = approx(np.sin(np.deg2rad(60))*step + y)
    new = (xr, yr)

    return new

def move90(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(90))*step + x)
    yr = approx(np.sin(np.deg2rad(90))*step + y)
    new = (xr, yr)

    return new

def move120(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(-np.cos(np.deg2rad(60))*step + x)
    yr = approx(np.sin(np.deg2rad(60))*step + y)
    new = (xr, yr)

    return new

def move150(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(-np.cos(np.deg2rad(30))*step + x)
    yr = approx(np.sin(np.deg2rad(30))*step + y)
    new = (xr, yr)

    return new

def move180(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(-np.cos(np.deg2rad(0))*step + x)
    yr = approx(np.sin(np.deg2rad(0))*step + y)
    new = (xr, yr)

    return new

def move210(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(-np.cos(np.deg2rad(30))*step + x)
    yr = approx(-np.sin(np.deg2rad(30))*step + y)
    new = (xr, yr)

    return new

def move240(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(-np.cos(np.deg2rad(60))*step + x)
    yr = approx(-np.sin(np.deg2rad(60))*step + y)
    new = (xr, yr)

    return new

def move270(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(-np.cos(np.deg2rad(90))*step + x)
    yr = approx(-np.sin(np.deg2rad(90))*step + y)
    new = (xr, yr)

    return new

def move300(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(60))*step + x)
    yr = approx(-np.sin(np.deg2rad(60))*step + y)
    new = (xr, yr)

    return new

def move330(vertex, step):
    x = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(30))*step + x)
    yr = approx(-np.sin(np.deg2rad(30))*step + y)
    new = (xr, yr)

    return new


map_points = []
for i in range(801):
    for j in range(501):
        map_points.append((i/2,(j/2)))
print(len(map_points))


obstacle_space = []

for pt in map_points:

    x = pt[0]
    y = pt[1]


    if (((x - 300)*(x - 300)) + ((y - 185)*(y - 185)) <= (40*40)):
        obstacle_space.append((x,y))


    if x <= 235 and x >= 165 and (74*x - 175*y + 8825 >=0) and (74*x + 175*y - 38425 <=0) and (74*x - 175*y - 3425 <=0) and (74*x + 175*y - 26175 >=0):
        obstacle_space.append((x,y))


    if ((25*x - 79*y + 13715 >= 0) and (6*x - 7*y + 780 <= 0) and (85*x + 69*y - 15825 >= 0)) or ((85*x + 69*y -15825>=0) and (16*x + 5*y - 2180<=0) and (25*x - 79*y + 13715 >=0)):
        obstacle_space.append((x,y))


obstacle_space.sort()

def cost_of_nodes(node, size, step):
    a,b = node[0], node[1]
    cost = {}
    if 0 <= a <= size[0] and 0 <= b <= size[1]:
        p1 = move0(node, step)

        if check_obstacle(p1[0], p1[1]) is False:
            cost[p1] = 1

        p2 = move30(node, step)
        if check_obstacle(p2[0], p2[1]) is False and p2 != p1:
            cost[p2] = 1.2

        p3 = move60(node, step)
        if check_obstacle(p3[0], p3[1]) is False and p3 != p1 and p3 != p2:
            cost[p3] = 1.5

        p4 = move90(node, step)
        if check_obstacle(p4[0], p4[1]) is False and p4 != p3 and p4 != p2 and p4 != p1:
            cost[p4] = 1

        p5 = move120(node, step)
        if check_obstacle(p5[0], p5[1]) is False and p5 != p4 and p5 != p3 and p5 != p2 and p5 != p1:
            cost[p5] = 1.2

        p6 = move150(node, step)
        if check_obstacle(p6[0], p6[1]) is False and p6 != p5:
            cost[p5] = 1.5

        p7 = move180(node, step)
        if check_obstacle(p7[0], p7[1]) is False:
            cost[p6] = 1

        p8 = move210(node, step)
        if check_obstacle(p8[0], p8[1]) is False and p8 != p7:
            cost[p7] = 1.2

        p9 = move240(node, step)
        if check_obstacle(p9[0], p9[1]) is False and p9 != p8 and p9 != p7:
            cost[p8] = 1.5

        p10 = move270(node, step)
        if check_obstacle(p10[0], p10[1]) is False and p10 != p9 and p10 != p8 and p9 != p7:
            cost[p9] = 1

        p11 = move300(node, step)
        if check_obstacle(p11[0], p11[1]) is False and p11 != p10 and p11 != p9 and p11 != p8 and p11 != p7:
            cost[p10] = 1.2

        p12 = move330(node, step)
        if check_obstacle(p12[0], p12[1]) is False and p12 != p11 and p12 != p10 and p12 != p9 and p12 != p8 and p12 != p6 and p12 != p7:
            cost[p11] = 1.5

        return cost        


min_dist_list = {}

visited_nodes = []

node_cost = np.array(np.ones((800, 500)) * np.inf)

goal_dist = np.array(np.ones((800,500)) * np.inf)

total_cost = np.array(np.ones((800,500)) * np.inf)

p_q = []

hp.heappush(p_q, (heuristic(goal, start), start))

node_cost[int(x_start)][int(y_start)] = 0

total_cost[int(x_start)][int(y_start)] = heuristic(goal, start)


def a_star():

    backtracking = {}
    size = [800, 500]
    if check_obstacle(x_start,y_start) or check_obstacle(x_goal, y_goal) is True:
        print('Invalid input co-ordinates for goal or robot')
        print('Run again')
        exit()
    star = 1
    while star != 0:

        c, curr_ver = hp.heappop(p_q)


        visited_nodes.append(curr_ver)
        print(visited_nodes[-1])

        tree = cost_of_nodes(curr_ver, size, step_size)

        for point, values in tree.items():
            c = values
            x = point[0]
            y = point[1]

            if point in visited_nodes or point in obstacle_space:
                continue

            new_cost = c + node_cost[int(curr_ver[0])][int(curr_ver[1])]

            if new_cost < node_cost[int(x)][int(y)]:
                node_cost[int(x)][int(y)] = new_cost

                backtracking[point] = curr_ver

            eucldiean = heuristic([x, y], goal)

            if eucldiean < goal_dist[int(x)][int(y)]:
                goal_dist[int(x)][int(y)] = eucldiean

            combined = node_cost[int(x)][int(y)] + goal_dist[int(x)][int(y)]

            if combined < total_cost[int(x)][int(y)]:
                total_cost[int(x)][int(y)] = combined

            hp.heappush(p_q, (total_cost[int(x)][int(y)], point))

            d = ((x-goal[0])**2+(y-goal[1])**2)
            if d <= (1.5**2)*step_size:
                star = 0
                break

    return point, backtracking


def backtrack(back, goal, start):

    path_list = []
    path_list.append(start)
    g = 0
    g1 = goal[0]
    g2 = goal[1]
    while g == 0:
        for k, v in back.items():

            p3 = int(v[0])
            p4 = int(v[1])
            y = [p3, p4]
            p1 = int(k[0])
            p2 = int(k[1])
            z = [p1, p2]
            if z == start:
                start = y
                path_list.append(start)
        if start == [g1, g2]:
            break

    return path_list


t = time.time()
new_goal_rounded, b_dict = a_star()
x = int(new_goal_rounded[0])
y = int(new_goal_rounded[1])
backtrackfinal = backtrack(b_dict, start, [x, y])
print(backtrackfinal)
print('Time: ', time.time()-t)


new_list_visited = []
for visited_node in visited_nodes:
    new_x = int(visited_node[0]/2)
    new_y = int(visited_node[1]/2)
    new_list_visited.append((new_x,new_y))


new_backtracked = []
for back in backtrackfinal:
    new_x_b = int(back[0]/2)
    new_y_b = int(back[1]/2)
    new_backtracked.append((new_x_b,new_y_b))


new_canvas = np.zeros((250,400,3),np.uint8)

for c in obstacle_space:
    x = c[1]
    y = c[0]
    new_canvas[(int(x), int(y))] = [100, 0, 255]

new_canvas = np.flipud(new_canvas)

new_canvas_copy_backtrack = new_canvas.copy()

new_canvas_copy_visited = new_canvas.copy()

line_image = new_canvas.copy()


cv2.imshow('new_canvas',new_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()

for visit_path in new_list_visited:

    x = int(visit_path[0])
    y = int(visit_path[1])
    new_canvas_copy_visited[(int(x),int(y))]=[255,255,255]

cv2.imshow('visited', new_canvas_copy_visited)
cv2.waitKey(0)
cv2.destroyAllWindows()


for i in range(len(new_backtracked)-1):
    point_1 = new_backtracked[i]
    point_2 = new_backtracked[i+1]

    x1, y1 = point_1[0], point_1[1]
    x2, y2 = point_2[0], point_2[1]

    image = cv2.line(line_image, (y2, x2), (y1, x1), (0, 255, 0), 1)

cv2.imshow('backtracked', image)
cv2.waitKey(0)
cv2.destroyAllWindows()


pygame.init()


gameDisplay = pygame.display.set_mode((400,250),pygame.FULLSCREEN)
pygame.display.set_caption('Animation')

black = (0,0,0)
white = (0,255,255)

surf = pygame.surfarray.make_surface(new_canvas_copy_visited)

clock = pygame.time.Clock()

done = False
while not done:
    
    for event in pygame.event.get(): 
        
        if event.type == pygame.QUIT:  
            done = True   
 
    gameDisplay.fill(black)
    for path in new_list_visited:
        if path not in new_canvas_copy_visited:
            pygame.time.wait(100)
            x = abs(250-path[1])
            y = path[0]
            pygame.draw.rect(gameDisplay, white, [x,y,1,1])
            pygame.display.flip()
            
    for path in new_backtracked:
        
        pygame.time.wait(100)
        x = abs(250-path[1])
        y = path[0]
        pygame.draw.rect(gameDisplay, (0,0,200), [x,y,1,1])
        pygame.display.flip()

    done = True

