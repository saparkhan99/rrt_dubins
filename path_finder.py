import random
import math
import numpy as np
import pygame
import sys
import time


from rrt_graph import RRTGraph
from rrt_env import Envir


def distance(first, second):
	(x1,y1) = first
	(x2,y2) = second
	px = (float(x1)-float(x2))**2
	py = (float(y1)-float(y2))**2
	return (px+py)**0.5





dimensions = (700, 1200)
start = (50, int(random.uniform(100, 600)))
goal = (1050, int(random.uniform(100, 600)))
"""dimensions = (600,1200)
start = (100, 500)
goal = (1100, 500)"""
obsdim = 100
obsnum = 10
iteration = 0
iteration1=0




pygame.init
running = True
environment = Envir(start, goal, dimensions, obsdim, obsnum)
graph = RRTGraph(start, goal, dimensions, obsdim, obsnum, environment)
obstacles,_ = graph.makeobs()

environment.drawMap(obstacles)
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    #not graph.path_to_goal()
    while(not graph.path_to_goal()):

        """if iteration % 20 == 0:
            Coordinates, Parent, Dubins = graph.bias(goal)
            pygame.draw.circle(environment.map, environment.grey, Coordinates[-1], environment.nodeRad+2, 0)
            #print("location:", (X[-1], Y[-1]))
            #print("Entering in", math.degrees(Dubins[-1][17]))
            graph.drawcurves(Dubins[-1])
        else:"""
        Coordinates, Parent, Dubins = graph.expand()
        #pygame.draw.circle(environment.map, environment.grey, Coordinates[-1], environment.nodeRad+2, 0)
        #print("location:", (X[-1], Y[-1]))
        #print("Entering in", math.degrees(Dubins[-1][17]))
        environment.drawcurves(Dubins[-1], environment.blue)

        if iteration % 1 == 0:

            pygame.display.update()
        iteration += 1
        #pygame.draw.line(environment.map, environment.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),environment.edgeThickness)

    drawing = graph.getPathCoords()
    environment.drawPath(drawing, environment.red)
    """if iteration1 < 1:
        for i in drawing:
            print(i[0], "\n")
        iteration1 += 1
    """

    pygame.display.update()
