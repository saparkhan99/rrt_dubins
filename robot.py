import random
import math
import numpy as np
import pygame
import sys
import time

class Robot():
	def __init__(self, startpos, robotImg, width):
		self.m2p = 3779.52 #from meters to pixels
		# robot dims
		self.w = width
		self.x = startpos[0]
		self.y = startpos[1]
		self.theta = 0
		self.v = 15
		self.rturn = 8

		self.maxspeed = 0.02*self.m2p
		self.minspeed = 0.02*self.m2p

		#graphics
		self.img = pygame.image.load(robotImg)
		self.rotated = self.img
		self.rect = self.rotated.get_rect(center = (self.x, self.y))

	def draw(self, environment):
		environment.map.blit(self.rotated, self.rect)

	def move(self, dt):
		self.x += self.v*math.cos(self.theta)*dt
		self.y -= self.v*math.sin(self.theta)*dt
		self.theta+= self.v/self.rturn*dt
		self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
		self.rect = self.rotated.get_rect(center=(self.x,self.y))
