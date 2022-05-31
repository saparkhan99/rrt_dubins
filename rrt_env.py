import random
import math
import numpy as np
import pygame
import sys
import time



class Envir:
	def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
	  self.start = start
	  self.goal = goal
	  self.MapDimensions = MapDimensions
	  self.Maph, self.Mapw = self.MapDimensions

	  # window selecting
	  self.MapWindowName = 'Dubins-Based RRT path planning:'
	  pygame.display.set_caption(self.MapWindowName)

	  self.map = pygame.display.set_mode((self.Mapw,self.Maph))
	  self.map.fill((255,255,255))
	  self.nodeRad =2
	  self.nodeThickness = 0
	  self.edgeThickness = 1

	  self.obstacles = []
	  self.obsdim = obsdim
	  self.obsNumber = obsnum

	  #Colors
	  self.black = (0, 0, 0)
	  self.yel = (255, 255, 0)
	  self.grey = (70, 70, 70)
	  self.blue = (0, 0, 255)
	  self.green = (0, 255, 0)
	  self.red = (255, 0, 0)
	  self.white = (255, 255, 255)
	  self.trail_set = []
	  self.rturn = 8


	def drawMap(self, obstacles):
		pygame.draw.circle(self.map, self.green, self.start, self.nodeRad+5, 0)
		pygame.draw.circle(self.map, self.green, self.goal, self.nodeRad+20, 1)
		self.drawObs(obstacles)

	def drawPath(self,path, color):
		for node in path:
			if color == self.red:
				pygame.draw.circle(self.map, color,  node[0], self.nodeRad+3, 0)
			self.drawcurves(node[1], color)

	def drawObs(self, obstacles):
		obstacleslist = obstacles.copy()
		while(len(obstacleslist)>0):
			obstacle = obstacleslist.pop(0)
			pygame.draw.rect(self.map, self.grey, obstacle)

	def drawcurves(self, data, color):
		type = data[0]
		(tan_pt1_x,tan_pt1_y) = (data[5], data[6])
		(tan_pt2_x,tan_pt2_y) = (data[7], data[8])
		(c1_x,c1_y) = (data[9], data[10])
		(c2_x,c2_y) = (data[11], data[12])
		theta_i1 = data[13]
		theta_i2 = data[14]
		theta_f1 = data[15]
		theta_f2 = data[16]
		if type == 'LSL':
			pygame.draw.arc(self.map, color,  [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn,2*self.rturn],
			-theta_i1, -theta_i2, 1)
			pygame.draw.arc(self.map, color,  [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn,2*self.rturn],
			-theta_f1, -theta_f2, 1)
			pygame.draw.line(self.map, color,  (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)
		elif type == 'RSR':
			pygame.draw.arc(self.map, color,  [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_i2, -theta_i1, 1)
			pygame.draw.arc(self.map, color,  [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_f2, -theta_f1, 1)
			pygame.draw.line(self.map, color,  (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)
		elif type == 'LSR':
			pygame.draw.arc(self.map, color,  [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_i1, -theta_i2, 1)
			pygame.draw.arc(self.map, color,  [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_f2, -theta_f1, 1)
			pygame.draw.line(self.map, color,  (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)
		elif type=="RSL":
			pygame.draw.arc(self.map, color,  [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_i2, -theta_i1, 1)
			pygame.draw.arc(self.map, color,  [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_f1, -theta_f2, 1)
			pygame.draw.line(self.map, color,  (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)


	def trail(self, pos):
		for i in range(0,len(self.trail_set)-1):
			pygame.draw.line(self.map, self.green, (self.trail_set[i][0], self.trail_set[i][1]),
			(self.trail_set[i+1][0], self.trail_set[i+1][1]))
		if self.trail_set.__sizeof__()>sys.maxsize:
			self.trail_set.pop(0)
		self.trail_set.append(pos)
