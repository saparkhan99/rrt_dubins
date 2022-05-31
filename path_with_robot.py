import random
import math
import numpy as np
import pygame
import sys
import time

from robot import Robot
from rrt_graph import RRTGraph
from rrt_env import Envir


def distance(first, second):
	(x1,y1) = first
	(x2,y2) = second
	px = (float(x1)-float(x2))**2
	py = (float(y1)-float(y2))**2
	return (px+py)**0.5





t = pygame.time.get_ticks()
dimensions = (700, 1200)
start = (50, int(random.uniform(100, 600)))
goal = (1050, int(random.uniform(100, 600)))
#start = (100, 350)
#goal = (1100, 350)
obsdim = 100
obsnum = 15
iteration = 0
iteration1=0
robot = Robot(start, r"car.png", 0.001*3779.52)

pygame.init()
running = True
environment = Envir(start, goal, dimensions, obsdim, obsnum)
graph = RRTGraph(start, goal, dimensions, obsdim, obsnum, environment)

obstacles, positions = graph.makeobs()
#obstacles1 = graph.resize_obs(positions)
environment.drawMap(obstacles)

while(not graph.path_to_goal()):
	"""if iteration % 20 == 0:
		Coordinates, Parent, Dubins = graph.bias(goal)
		#pygame.draw.circle(environment.map, environment.grey, Coordinates[-1], environment.nodeRad+2, 0)
		environment.drawcurves(Dubins[-1], environment.blue)
	else:"""
	Coordinates, Parent, Dubins = graph.expand()
	environment.drawcurves(Dubins[-1], environment.blue)

	#if iteration % 1 == 0:
	pygame.display.update()
	iteration += 1
	#pygame.draw.line(environment.map, environment.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),environment.edgeThickness)

drawing = graph.getPathCoords()
environment.drawPath(drawing, environment.red)
t2 = pygame.time.get_ticks()


iteration2 = 0
trajectory = []
current_trajectory = []
dt = 0
lasttime = pygame.time.get_ticks()
while running:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			running = False
	dt = (pygame.time.get_ticks()-lasttime)/1000
	lasttime = pygame.time.get_ticks()
	pygame.display.update()
	environment.map.fill(environment.white)
	environment.drawMap(obstacles)
	for i in range(graph.number_of_nodes()):
		#pygame.draw.circle(environment.map, environment.grey, graph.coordinates[i], environment.nodeRad+2, 0)
		environment.drawcurves(graph.dubins_paths[i], environment.blue)

	if iteration2 < 1:
		if (drawing[-1][1][0] == "LSL"):
			trajectory = ["L", "S", "L"]
			current_trajectory = "L"
		if (drawing[-1][1][0] == "RSR"):
			trajectory = ["R", "S", "R"]
			current_trajectory = "R"
		if (drawing[-1][1][0] == "LSR"):
			trajectory = ["L", "S", "R"]
			current_trajectory = "L"
		if (drawing[-1][1][0] == "RSL"):
			trajectory = ["R", "S", "L"]
			current_trajectory = "R"
		iteration2 += 1
		#print("\n WEEEEEELLLL, The last:", drawing[-1][1], "\n" )

	(tan_pt1_x,tan_pt1_y) = (drawing[-1][1][5], drawing[-1][1][6])
	(tan_pt2_x,tan_pt2_y) = (drawing[-1][1][7], drawing[-1][1][8])
	(c1_x,c1_y) = (drawing[-1][1][9], drawing[-1][1][10])
	(c2_x,c2_y) = (drawing[-1][1][11], drawing[-1][1][12])
	theta_i1 = drawing[-1][1][13]
	theta_i2 = drawing[-1][1][14]
	theta_f1 = drawing[-1][1][15]
	theta_f2 = drawing[-1][1][16]

	theta_center1 = (1)*math.atan2(robot.y - c1_y, robot.x - c1_x)
	theta_center2 = (1)*math.atan2(robot.y - c2_y, robot.x - c2_x)

	if drawing[-1][1][0] == "RSL":
		if current_trajectory == "R":
			if abs(round(math.degrees(theta_center1), 1) - round(math.degrees(theta_i2), 1)) < 1.3:
				current_trajectory = trajectory[1]
				(robot.x, robot.y) = (tan_pt1_x, tan_pt1_y)
				robot.theta = (-1)*np.arctan2(tan_pt2_y-tan_pt1_y, tan_pt2_x - tan_pt1_x)
			else:
				robot.theta-= robot.v/robot.rturn*dt
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		if current_trajectory == "S":
			if abs(round(math.degrees(theta_center2), 1) - round(math.degrees(theta_f1), 1)) < 1.3:
				current_trajectory = trajectory[2]
				(robot.x, robot.y) = (tan_pt2_x, tan_pt2_y)
			else:
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		if current_trajectory == "L":
			robot.theta += robot.v/robot.rturn*dt
			robot.x += robot.v*math.cos(robot.theta)*dt
			robot.y -= robot.v*math.sin(robot.theta)*dt

		robot.rotated = pygame.transform.rotozoom(robot.img, math.degrees(robot.theta), 1)
		robot.rect = robot.rotated.get_rect(center=(robot.x,robot.y))

	if drawing[-1][1][0] == "LSR":
		if current_trajectory == "R":
			robot.theta-= robot.v/robot.rturn*dt
			robot.x += robot.v*math.cos(robot.theta)*dt
			robot.y -= robot.v*math.sin(robot.theta)*dt

		if current_trajectory == "S":
			if abs(round(math.degrees(theta_center2), 1) - round(math.degrees(theta_f1), 1)) < 1.3:
				current_trajectory = trajectory[2]
				(robot.x, robot.y) = (tan_pt2_x, tan_pt2_y)
			else:
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		if current_trajectory == "L":
			if abs(round(math.degrees(theta_center1), 1) - round(math.degrees(theta_i2), 1)) < 1.3:
				current_trajectory = trajectory[1]
				(robot.x, robot.y) = (tan_pt1_x, tan_pt1_y)
				robot.theta = (-1)*np.arctan2(tan_pt2_y-tan_pt1_y, tan_pt2_x - tan_pt1_x)
			else:
				robot.theta+= robot.v/robot.rturn*dt
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		robot.rotated = pygame.transform.rotozoom(robot.img, math.degrees(robot.theta), 1)
		robot.rect = robot.rotated.get_rect(center=(robot.x,robot.y))


	if drawing[-1][1][0] == "LSL":
		if current_trajectory == "L":
			if abs(round(math.degrees(theta_center1), 1) - round(math.degrees(theta_i2), 1)) < 1.3:
				current_trajectory = trajectory[1]
				(robot.x, robot.y) = (tan_pt1_x, tan_pt1_y)
				robot.theta = (-1)*np.arctan2(tan_pt2_y-tan_pt1_y, tan_pt2_x - tan_pt1_x)
			else:
				robot.theta+= robot.v/robot.rturn*dt
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		if current_trajectory == "S":
			if abs(round(math.degrees(theta_center2), 1) - round(math.degrees(theta_f1), 1)) < 1.3:
				current_trajectory = trajectory[2]
				(robot.x, robot.y) = (tan_pt2_x, tan_pt2_y)
			else:
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		robot.rotated = pygame.transform.rotozoom(robot.img, math.degrees(robot.theta), 1)
		robot.rect = robot.rotated.get_rect(center=(robot.x,robot.y))

	if drawing[-1][1][0] == "RSR":
		if current_trajectory == "R":
			if abs(math.degrees(theta_center1) - math.degrees(theta_i2)) < 1.3:
				current_trajectory = trajectory[1]
				(robot.x, robot.y) = (tan_pt1_x, tan_pt1_y)
				robot.theta = (-1)*np.arctan2(tan_pt2_y-tan_pt1_y, tan_pt2_x - tan_pt1_x)
			else:
				robot.theta-= robot.v/robot.rturn*dt
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		if current_trajectory == "S":
			if abs(round(math.degrees(theta_center2), 1) - round(math.degrees(theta_f1), 1)) < 1.3:
				current_trajectory = trajectory[2]
				(robot.x, robot.y) = (tan_pt2_x, tan_pt2_y)
			else:
				robot.x += robot.v*math.cos(robot.theta)*dt
				robot.y -= robot.v*math.sin(robot.theta)*dt

		robot.rotated = pygame.transform.rotozoom(robot.img, math.degrees(robot.theta), 1)
		robot.rect = robot.rotated.get_rect(center=(robot.x,robot.y))

	if abs(round(math.degrees(theta_center2),1) - round(math.degrees(theta_f2), 1)) < 1.2:
		(robot.x, robot.y) = drawing[-1][0]
		robot.theta = drawing[-1][1][17]
		#print(math.degrees(theta_f2))
		#print(math.degrees(theta_center2))
		drawing.pop()
		iteration2 = 0


	if distance((robot.x, robot.y), goal)<0.1:
		robot.draw(environment)
		environment.trail((robot.x, robot.y))
		pygame.display.update()
		pygame.event.clear()
		pygame.event.wait(0)

	if not drawing:
		#robot.move(dt)
		robot.draw(environment)
		environment.trail((robot.x, robot.y))
		break
	else:
		environment.drawcurves(drawing[-1][1], environment.red)
		environment.drawPath(drawing, environment.red)
		robot.draw(environment)
		environment.trail((robot.x, robot.y))
t1 = pygame.time.get_ticks()
#print("Total time:",  (t1 - t)/1000)
print("RRT time:",  (t2 - t)/1000)
print("Number of nodes:", graph.number_of_nodes() )
print("Number of iterations:", iteration)







"""
pygame.display.update()
pygame.event.clear()
pygame.event.wait(0)
"""
