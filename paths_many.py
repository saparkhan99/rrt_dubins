
import random
import math
import json
import cv2
import numpy as np
import sys
import PIL.Image as Image 
from threading import *


class Envir:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions
        self.image_shape = (self.Maph, self.Mapw, 3)       
        
        self.nodeRad =0
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        #Colors
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)
        self.white = (255, 255, 255)
        
        # window selecting
        self.canvas = np.ones(self.image_shape, dtype=np.uint8) * 0
        cv2.circle(self.canvas, start, self.nodeRad+7, self.green, -1, 8, 0)
        cv2.circle(self.canvas, goal, self.nodeRad+13, self.green, -1, 8, 0)


        


    def drawPath(self,pathoriginal, color, points_counter):
        path = pathoriginal[::-1]

        for i in range(len(path)):
            if i == 0:
                cv2.circle(self.canvas, self.start, self.nodeRad+7, self.blue, -1, 8, 0)
                img = Image.fromarray(self.canvas, 'RGB')
                img.save(r"dataset_224/images/img" + str(points_counter+1000000) + ".png")
                    
                data_p = dict()
                data_p = {'isstart': 1, 'isgoal': 0,'start': self.start, 'goal': self.goal, 'previous_points': path[:i], 'next_piont': path[i+1], 'path_coordinates': path, "obstacles": self.obstacles}
                with open(r"dataset_224/parametric/mydata" + str(points_counter+1000000) + ".json", 'w') as f:
                    json.dump(data_p, f)
                points_counter+=1
                
            if i != 0:
                if i != len(path)-1:
                    cv2.circle(self.canvas, path[i], self.nodeRad+7, (255*((i+1)/(len(path)-1)) ,0, 0), -1, 8, 0)
                    img = Image.fromarray(self.canvas, 'RGB')
                    img.save(r"dataset_224/images/img" + str(points_counter+1000000) + ".png")
                    
                    data_p = dict()
                    if i == len(path)-2:
                        data_p = {'isstart': 0, 'isgoal': 1, 'start': self.start, 'goal': self.goal, 'previous_points': path[:i], 'next_piont': path[i+1], 'path_coordinates': path, "obstacles": self.obstacles}
                    else:
                        data_p = {'isstart': 0, 'isgoal': 0, 'start': self.start, 'goal': self.goal, 'previous_points': path[:i], 'next_piont': path[i+1], 'path_coordinates': path, "obstacles": self.obstacles}
                    
                    with open(r"dataset_224/parametric/mydata" + str(points_counter+1000000) + ".json", 'w') as f:
                        json.dump(data_p, f)
                    points_counter+=1
                    
                    
                    
            
                
            

        return points_counter
    
    def drawobs(self, obs):
        for obstacle in obs:
            end_point = (obstacle[0] + self.obsdim, obstacle[1] + self.obsdim)
            cv2.rectangle(self.canvas, obstacle, end_point, self.white, -1)

class RRTGraph:

    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x,y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions
        self.coordinates = []
        self.parent = []
        self.radius = 80

        #initialize the tree
        self.coordinates.append((x,y))
        self.parent.append(0)

        # the obstacles
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        # path
        self.goalstate = None
        self.path = []





    def makeRandomRect(self):
        uppercornerx = int(random.uniform(25, self.Mapw - self.obsdim-25))
        uppercornery = int(random.uniform(20, self.Maph - self.obsdim-20))
        return (uppercornerx, uppercornery)

    def collidepoint(self, rectang, point):
        if rectang[0] < point[0] < rectang[0] + self.obsdim and rectang[1] < point[1] < rectang[1] + self.obsdim:
            return True
        else:
            return False

    def makeobs(self):
        obs = []
        for i in range(0, self.obsnum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                rectang = self.makeRandomRect()
                if self.collidepoint(rectang, self.start) or self.collidepoint(rectang, self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
                    obs.append(rectang)
            self.obstacles = obs.copy()
        return obs


    def add_node(self, n, x, y):
        self.coordinates.insert(n, (x,y))



    def remove_node(self,n):
        self.coordinates.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.coordinates)

    def distance(self, n1, n2):
        (x1,y1) = self.coordinates[n1]
        (x2,y2) = self.coordinates[n2]
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        return (px+py)**0.5

    def distance1(self, first, second):
        (x1,y1) = first
        (x2,y2) = second
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        return (px+py)**0.5

    def sample_envir(self):
        x = int(random.uniform(0, self.Mapw))
        y = int(random.uniform(0, self.Maph))
        return x, y


    def nearest(self,n):
        dmin = self.distance(0,n)
        nnear=0
        for i in range(0,n):
            if self.distance(i,n)<dmin:
                dmin=self.distance(i,n)
                nnear =i
        return nnear


    def isFree(self):
        n =self.number_of_nodes()-1
        (x,y) = self.coordinates[n]
        obs = self.obstacles.copy()
        while len(obs)>0:
            rectang = obs.pop(0)
            if self.collidepoint(rectang, (x,y)):
                self.remove_node(n)
                return False
        return True


    def crossObstacle(self, first, second):
        (x1, y1) = first
        (x2, y2) = second
        obs = self.obstacles.copy()
        while (len(obs)>0):
            rectang = obs.pop(0)
            for i in range(0,101):
                u = i/100
                x = x1*u+x2*(1-u)
                y = y1*u+y2*(1-u)
                if self.collidepoint(rectang, (x,y)):
                    return True
        return False


    def connect(self, n1, n2):
        (x1,y1) = self.coordinates[n1]
        (x2,y2) = self.coordinates[n2]
        if self.crossObstacle((x1, y1), (x2, y2)):
            self.remove_node(n2)
            self.goalFlag = False
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=20):
        d = self.distance(nnear,nrand)
        if d>dmax:
            u = dmax/d
            (xnear, ynear) = self.coordinates[nnear]
            (xrand, yrand) = self.coordinates[nrand]
            (px,py) = (xrand-xnear, yrand-ynear)
            theta = (1)*math.atan2(py,px)
            (x, y)=(int(xnear+dmax*math.cos(theta)), int(ynear+dmax*math.sin(theta)))
            self.remove_node(nrand)
            if self.goalFlag == False:
                if self.distance1((x,y), self.goal)<2*dmax:
                    self.add_node(nrand, self.goal[0], self.goal[1])
                    self.goalstate = nrand
                    self.goalFlag = True
                    #print("Found the goal and the distance:", self.distance1((x,y), self.goal) )
                else:
                    self.add_node(nrand, x, y)


    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear,n)
        if self.connect(nnear,n):
            self.rewire(n)
        return self.coordinates, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x,y = self.sample_envir()
        self.add_node(n,x,y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            if self.connect(xnearest, n):
                self.rewire(n)
        return self.coordinates, self.parent


    def inradius(self, n):
        inradius = []
        nnear=0
        for i in range(0,n):
            if self.distance(i,n)<self.radius:
                if not self.crossObstacle(self.coordinates[i], self.coordinates[n]):
                    inradius.append(i)
        return inradius

    def rewire(self, n):
        new_dist = 0.0
        closest = None
        lowest_cost = self.cost(n)
        inradius = self.inradius(n)
        for i in inradius:
            new_dist = self.cost(i)+self.distance(i,n)
            if lowest_cost > new_dist:
                lowest_cost = new_dist
                closest = i
        if closest != None:
            self.remove_edge(n)
            self.add_edge(closest, n)

    def cost(self, n):
        dist = 0.0
        while(n != 0):
            dist +=  self.distance(n, self.parent[n])
            n = self.parent[n]
        return dist



    def path_to_goal(self):
        if self.goalFlag:
            self.path = []


            if len(self.parent) == self.goalstate:
                #print("Parent:", len(self.parent), "Goals state:", self.goalstate)
                self.path.append(self.goalstate-1)
                newpos = self.parent[self.goalstate-1]

            else:
                self.path.append(self.goalstate)
                newpos = self.parent[self.goalstate]


            while(newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x,y = self.coordinates[node]
            pathCoords.append((x,y))
        dist = self.cost(self.number_of_nodes()-1)
        return pathCoords, dist

    pass





size = 224
dimensions = (700, 1200)
start = (20, int(random.uniform(0, size)))
goal = (size-20, int(random.uniform(0, size)))
obsdim = 30
obsnum = 15



iteration = 0
iteration_outer = 0
counter = 1500
points_counter = 27
print("Works!!!")



while(iteration_outer<30000):
    
    start = (20, int(random.uniform(0, size)))
    goal = (size-20, int(random.uniform(0, size)))



    environment = Envir(start, goal, (size, size), obsdim, obsnum)
    graph = RRTGraph(start, goal, (size, size), obsdim, obsnum)

    obstacles = graph.makeobs()
    environment.drawobs(obstacles)
    running = True
    feasible_paths = []
    optimal_path = []
    while(iteration<counter):

        """if iteration % 20 == 0:
            Coordinates, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (Coordinates[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (Coordinates[-1]), (Coordinates[Parent[-1]]),
            map.edgeThickness)
        else:"""
        Coordinates, Parent = graph.expand()
        #pygame.draw.circle(map.map, map.grey, (Coordinates[-1]), map.nodeRad+2, 0)
        #pygame.draw.line(map.map, map.blue, (Coordinates[-1]), (Coordinates[Parent[-1]]), map.edgeThickness)

        if graph.path_to_goal():
            path, dist = graph.getPathCoords()

            feasible_paths.append((path, dist))
            #print("Hey",dist)
            #distances.append(whole_dist)
            #print("FOUND!!!")
            graph.goalFlag = False
            #pygame.image.save(map.map, "path"+=.png"

        iteration += 1
        #print("Iteration:", iteration)



        
    environment.obstacles = graph.obstacles

    mindist = sys.maxsize
    for one in feasible_paths:
        #map.drawPath(one[0], map.red)
        if mindist>one[1]:
            mindist = one[1]
            optimal_path = one[0]
    
    new_count = environment.drawPath(optimal_path, environment.red, points_counter)
    points_counter = new_count
    print("Image number:", points_counter)
    #pygame.image.save(map.map, "path"+str(points_counter)+".png")
    counter = iteration+1500
    
    iteration_outer += 1
    del environment
    del graph




