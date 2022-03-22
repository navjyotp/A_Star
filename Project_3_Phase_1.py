import numpy as np
import math as m
import heapq as hp
import cv2
import pygame

#get user input
x_start= int(input("Enter the x coordinate of the start:  "))
y_start= int(input("Enter the y coordinate of the start:  "))

start_orientation = int(input("Enter the Orientation at start (enter in multiples of 30 degreees and less that 360 degrees), :  "))
x_goal= int(input("Enter the x coordinate of the goal:  "))
y_goal= int(input("Enter the y coordinate of the goal:  "))
#y_goal = 199-y_goal
radius= int(input("Enter the radius of the robot:  "))
clearance= int(input("Enter the clearance of the robot: "))
step_size = int(input("Enter the step (1-10): "))
start = (x_start,y_start)
goal = (x_goal,y_goal)


# fucntion to approximate the node value

def approx(x):
    if round(x)+0.5 == x:
        y = x
    elif   round(x) < x:
        y = round(x)
    else:
        y = round(x)
    return y

#heuristic (eculidean distance)
def heuristic(a,b):
    x1, y1 = a[0], a[1]
    x2, y2 = b[0], b[1]

    distance = approx(m.sqrt((x2-x1)**2 + (y2-y1)**2))
    return distance


#action set
def move0(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(0)*step + x)
    yr = approx(np.cos(0)*step + y)
    new = (xr,yr)

    return new

def move30(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(30))*step + x)
    yr = approx(np.cos(np.deg2rad(30))*step + y)
    new = (xr,yr)

    return new

def move60(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(60))*step + x)
    yr = approx(np.cos(np.deg2rad(60))*step + y)
    new = (xr,yr)

    return new

def move90(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(90))*step + x)
    yr = approx(np.cos(np.deg2rad(90))*step + y)
    new = (xr,yr)

    return new

def move120(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(30))*step + x)
    yr = approx(np.cos(np.deg2rad(30))*step + y)
    new = (xr,yr)

    return new

# calulating all possible points 
map_points = []
for i in range(801):
    for j in range(501):
        map_points.append((i/2,(j/2)))
print(len(map_points))

#all possible points in obstacle space

obstacle_space = []

for pt in map_points:

    x = pt[0]
    y = pt[1]

    #circle shape for path traversal
    if (((x - 300)*(x - 300)) + ((y - 185)*(y - 185)) <= (40*40)):
        obstacle_space.append((x,y))


    #hexagon shape for path traversal
    if x <= 235 and x >= 165 and (74*x - 175*y + 8825 >=0) and (74*x + 175*y - 38425 <=0) and (74*x - 175*y - 3425 <=0) and (74*x + 175*y - 26175 >=0):
        obstacle_space.append((x,y))


    #quad shape for traversal
    if ((25*x - 79*y + 13715 >= 0) and (6*x - 7*y + 780 <= 0) and (85*x + 69*y - 15825 >= 0)) or ((85*x + 69*y -15825>=0) and (16*x + 5*y - 2180<=0) and (25*x - 79*y + 13715 >=0)):
        obstacle_space.append((x,y))

print(len(obstacle_space))


#fucntion to check point in obstacle space

'''
def check_obstacle(point):

    x = point[0]
    y = point[1]

    #check circle
    if (((x - 300)*(x - 300)) + ((y - 185)*(y - 185)) <= (55*55)):
        return False

    #check hexagon
    if x <= 250 and x >= 150 and (74*x - 175*y + 11675 >=0) and (74*x + 175*y - 41275 <=0) and (74*x - 175*y - 6275 <=0) and (74*x + 175*y - 23324 >=0):
        return False

    #check quad
    if ((25*x - 79*y + 14957 >= 0) and (6*x - 7*y + 641 <= 0) and (85*x + 69*y - 14182 >= 0)) or ((85*x + 69*y -14182>=0) and (16*x + 5*y - 2431<=0) and (25*x - 79*y + 14957 >=0)):
        return False

    #check boundry
    if x <= 15 or x >= 385 or y <= 15 or y >= 235:
        return False
'''

#points for drawing the graph
map_int_points = []

graph_points = []


for i in range(801):
    for j in range(501):
        map_int_points.append((i/2,(j/2)))

for pt in map_int_points:
    x = pt[0]
    y=  pt[1]

    if (((x - 300)*(x - 300)) + ((y - 185)*(y - 185)) <= (55*55)):
        graph_points.append((x,y))

    #check hexagon
    if x <= 250 and x >= 150 and (74*x - 175*y + 11675 >=0) and (74*x + 175*y - 41275 <=0) and (74*x - 175*y - 6275 <=0) and (74*x + 175*y - 23324 >=0):
        graph_points.append((x,y))

    #check quad
    if ((25*x - 79*y + 14957 >= 0) and (6*x - 7*y + 641 <= 0) and (85*x + 69*y - 14182 >= 0)) or ((85*x + 69*y -14182>=0) and (16*x + 5*y - 2431<=0) and (25*x - 79*y + 14957 >=0)):
        graph_points.append((x,y))



#sorting out the points 
obstacle_space.sort()




def cost_of_nodes(node, size, step, angle):
    a,b = node[0], node[1]
    cost = {}
    if 0 <= a <= size[0] and 0 <= b <= size[1]:
        p1 = move0(node,step)
        if p1 not in obstacle_space:
            cost[p1] = (1,angle)

        p2 = move30(node,step)
        if p2 not in obstacle_space:
            cost[p2] = (1.5,angle+30)

        p3 = move60(node,step)
        if p3 not in obstacle_space:
            cost[p3] = (1.5,angle+60)

        p4 = move90(node,step)
        if p4 not in obstacle_space:
            cost[p4] = (1, angle+90)

        p5 = move120(node,step)
        if p5 not in obstacle_space:
            cost[p5] = (1.5,angle+120)

        cost_copy = cost.copy()

        # for k,v in cost_copy.items():
        #     if k == node:
        #         del cost[k]

        return cost        


min_dist_list = {}

visited_nodes = []

node_cost = np.array(np.ones((800,500,12)) * np.inf)

direction = {0:0,30:1,60:2,90:3,120:4,150:5,180:6,210:7,240:8,270:9,300:10,330:11, 360:0}

goal_dist = np.array(np.ones((800,500,12)) * np.inf)

total_cost = np.array(np.ones((800,500,12)) * np.inf)

p_q = []

hp.heappush(p_q,(0,start,start_orientation))

node_cost[int(x_start)][int(y_start)][direction[int(start_orientation)]] = 0

total_cost[int(x_start)][int(y_start)][direction[int(start_orientation)]] = 0



def a_star(a,b):

    backtracking = {}
    size = [800,500]
    if a in obstacle_space or b in obstacle_space:
        print('Invalid input co-ordinates for goal or robot')
        print('Run again')
        exit()
    star = 1
    while star != 0:

        c, curr_ver,orient = hp.heappop(p_q)


        visited_nodes.append(curr_ver)

        tree = cost_of_nodes(curr_ver,size,step_size,orient)
        print(tree)

        for point, values in tree.items():
            c = values[0]
            pose = values[1]
            x = point[0]
            y = point[1]

            if point in visited_nodes:
                continue
            new_cost = c + node_cost[curr_ver[0]][curr_ver[1]][direction[int(orient)]]
            print(new_cost)
            if new_cost < node_cost[int(x)][int(y)][direction[int(pose)]]:
                node_cost[int(x)][int(y)][direction[int(pose)]] = new_cost
                print(node_cost)
                backtracking[point] = curr_ver

            goal_dist[int(x)][int(y)][direction[int(pose)]] = heuristic((3,2),(250,400))
            
            

            total_cost[int(x)][int(y)][direction[int(pose)]] = node_cost[x][y][direction[pose]] + goal_dist[x][y][direction[pose]]

            hp.heappush(p_q,(total_cost[int(x)][int(y)][direction[int(pose)]],point,pose))

            d = ((x-goal[0])**2+(y-goal[1]**2))
            if d <= 1.5**2:
                star = 0
                break
            print(tree)


    return point, backtracking


# backtracking 

def backtrack(backtracking,goal,start):
    path_list = []
    path_list.append(start)

    for k, v in backtracking_dict.items():

        if k == start:
            start = v
            path_list.append(start)

            if v == goal:
                break
    return path_list

new_goal_rounded,backtracking_dict= a_star(start,goal)
backtrackfinal = backtrack(backtracking_dict,start,goal)
print(backtracking_dict)
print(backtrackfinal)
print(visited_nodes)

exit()

new_list_visited = []
for visited_node in visited_nodes:
    new_x = visited_node[0]*2
    new_y = visited_node[1]*2
    new_list_visited.append((new_x,new_y))



new_backtracked = []
for back in backtrackfinal:
    new_x_b = back[0]*2
    new_y_b = back[1]*2
    new_backtracked.append((new_x_b,new_y_b))


# #defining a blank canvas
new_canvas = np.zeros((500,800,3),np.uint8) 

for c in obstacle_space: #change the name of the variable l
    x = 2*c[1]
    y = 2*c[0]
    new_canvas[(int(x),int(y))]=[255,0,255] #assigning a yellow coloured pixel


#flipping the image for correct orientation
new_canvas = np.flipud(new_canvas)
#making a copy for backtracking purpose
new_canvas_copy_backtrack = new_canvas.copy()
#making a copy for showing the visited nodes on the obstacle space
#can be used for the animation
new_canvas_copy_visited = new_canvas.copy()

#showing the obstacle map

cv2.imshow('new_canvas',new_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()


#visited path
for visit_path in new_list_visited:
    
    #print(path)
    x = int(visit_path[0])
    y = int(visit_path[1])
    new_canvas_copy_visited[(int(x),int(y))]=[255,255,255] #setting every backtracked pixel to white


#showing the image
cv2.imshow('visited',new_canvas_copy_visited)
cv2.waitKey(0)
cv2.destroyAllWindows()

#backtracked path
for path in new_backtracked:
    
    #print(path)
    x = int(path[0])
    y = int(path[1])
    new_canvas_copy_backtrack[(int(500-y),int(x))]=[255,255,255] #setting every backtracked pixel to white


cv2.imshow('backtracked',new_canvas_copy_backtrack)
cv2.waitKey(0)
cv2.destroyAllWindows()


#showing animation
pygame.init()

display_width = 250
display_height = 400

gameDisplay = pygame.display.set_mode((display_width,display_height),pygame.FULLSCREEN)
pygame.display.set_caption('Covered Nodes- Animation')

black = (0,0,0)
white = (0,255,255)
#new = np.array(new_canvas_copy_visited)
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
            pygame.time.wait(1)
            x = path[0]
            y = abs(500-path[1])
            pygame.draw.rect(gameDisplay, white, [x,y,1,1])
            pygame.display.flip()
            
    for path in new_backtracked:
        
        pygame.time.wait(5)
        x = path[0]
        y = abs(500-path[1])
        pygame.draw.rect(gameDisplay, (0,0,255), [x,y,1,1])
        pygame.display.flip()

    done = True
pygame.quit()


















