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


class RRTGraph:
	def __init__(self, start, goal, MapDimensions, obsdim, obsnum, environment):
		(x,y) = start
		self.start = start
		self.goal = goal
		self.goalFlag = False
		self.MapDimensions = MapDimensions
		self.Maph, self.Mapw = self.MapDimensions
		self.coordinates = []
		self.parent = []
		self.dubins_paths = []

		#initialize the tree
		self.coordinates.append((x,y))
		self.parent.append(0)
		self.initial = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.dubins_paths.append(self.initial)

		# the obstacles
		self.obstacles = []
		self.obsdim = obsdim
		self.obsNumber = obsnum

		# path
		self.goalstate = None
		self.path = []
		self.rturn = 8
		self.radius = 300


	def makeRandomRect(self):
		uppercornerx = int(random.uniform(100, self.Mapw - self.obsdim-100))
		uppercornery = int(random.uniform(0, self.Maph - self.obsdim))
		return (uppercornerx, uppercornery)

	"""def makeobs(self):
		obs = []
		rectang1 = pygame.Rect((500,0), (self.obsdim, 4*self.obsdim))
		rectang2 = pygame.Rect((200,300), (self.obsdim, 4*self.obsdim))
		rectang3 = pygame.Rect((800,300), (self.obsdim, 4*self.obsdim))
		obs = [rectang1, rectang2, rectang3]
		self.obstacles = obs.copy()
		return obs"""
	def drawcurves(self, data):
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
			pygame.draw.arc(self.map, self.blue, [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn,2*self.rturn],
			-theta_i1, -theta_i2, 1)
			pygame.draw.arc(self.map, self.blue, [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn,2*self.rturn],
			-theta_f1, -theta_f2, 1)
			pygame.draw.line(self.map, self.blue, (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)
		elif type == 'RSR':
			pygame.draw.arc(self.map, self.blue, [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_i2, -theta_i1, 1)
			pygame.draw.arc(self.map, self.blue, [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_f2, -theta_f1, 1)
			pygame.draw.line(self.map, self.blue, (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)
		elif type == 'LSR':
			pygame.draw.arc(self.map, self.blue, [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_i1, -theta_i2, 1)
			pygame.draw.arc(self.map, self.blue, [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_f2, -theta_f1, 1)
			pygame.draw.line(self.map, self.blue, (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)
		elif type=="RSL":
			pygame.draw.arc(self.map, self.blue, [c1_x-self.rturn, c1_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_i2, -theta_i1, 1)
			pygame.draw.arc(self.map, self.blue, [c2_x-self.rturn, c2_y - self.rturn, 2*self.rturn, 2*self.rturn],
			-theta_f1, -theta_f2, 1)
			pygame.draw.line(self.map, self.blue, (tan_pt1_x, tan_pt1_y), (tan_pt2_x, tan_pt2_y),1)

	def makeobs(self):
		obs = []
		positions = []
		for i in range(0, self.obsNumber):
			rectang = None
			startgoalcol = True
			while startgoalcol:
				upper = self.makeRandomRect()
				rectang = pygame.Rect(upper, (self.obsdim, self.obsdim))
				if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
					startgoalcol = True
				else:
					startgoalcol = False
					positions.append(upper)
			obs.append(rectang)
		self.obstacles = obs.copy()
		return obs, positions

	def resize_obs(self, positions):
		obstacleslist = obstacles.copy()
		new_obs = []
		for i in positions:
			rectang = pygame.Rect((i[0]+self.obsdim/20, i[1]+self.obsdim/20), (0.9*self.obsdim, 0.9*self.obsdim))
			new_obs.append(rectang)
		return new_obs

	def add_node(self, n, x, y):
		self.coordinates.insert(n, (x,y))

	def remove_node(self,n):
		self.coordinates.pop(n)


	def add_edge(self, parent, child, data):
		self.parent.insert(child, parent)
		self.dubins_paths.insert(child,data)

	def remove_edge(self, n):
		self.parent.pop(n)
		self.dubins_paths.pop(n)

	def number_of_nodes(self):
		return len(self.coordinates)

	def distance(self, n1, n2):
		(x1,y1) = (self.coordinates[n1])
		(x2,y2) = (self.coordinates[n2])
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
		(x,y) = (self.coordinates[n])
		obs = self.obstacles.copy()
		while len(obs)>0:
			rectang = obs.pop(0)
			if rectang.collidepoint(x,y):
				self.remove_node(n)
				return False
		return True

	def crossObstacleLine(self, first, second):
		(x1, y1) = first
		(x2, y2) = second
		obs = self.obstacles.copy()
		while (len(obs)>0):
			rectang = obs.pop(0)
			for i in range(0,101):
				u = i/100
				x = x1*u+x2*(1-u)
				y = y1*u+y2*(1-u)
				if rectang.collidepoint(x,y):
					return True
		return False


	def crossObstacleArc(self, point, center, tangent, theta, theta_c, theta_t, type, part):
		obs = self.obstacles.copy()
		(x_c, y_c) = center
		(x_t, y_t) = tangent

		if type == 'LSL':
			if part == "start":
				theta1 = -theta_c
				theta2 = -theta_t
			else:
				theta1 = -theta_t
				theta2 = -theta_c
		elif type == 'RSR':
			if part == "start":
				theta1 = -theta_t
				theta2 = -theta_c
			else:
				theta1 = -theta_c
				theta2 = -theta_t
		elif type == 'LSR':
			if part == "start":
				theta1 = -theta_c
				theta2 = -theta_t
			else:
				theta1 = -theta_c
				theta2 = -theta_t
		elif type=="RSL":
			if part == "start":
				theta1 = -theta_t
				theta2 = -theta_c
			else:
				theta1 = -theta_t
				theta2 = -theta_c
		theta1 += 2*math.pi
		theta2 += 2*math.pi

		if theta1>theta2:
			temp = theta1
			theta1 = theta2
			theta2 = temp


		while (len(obs)>0):
			rectang = obs.pop(0)
			if distance(rectang.center, center) >(self.rturn+self.obsdim/2*math.sqrt(2)):
				continue
			for i in range(int(math.degrees(theta1))+1, int(math.degrees(theta2))-1):
				if type == 'LSL':
					x = x_c + self.rturn*math.cos(i)
					y = y_c - self.rturn*math.sin(i)
				elif type == 'RSR':
					x = x_c - self.rturn*math.cos(i)
					y = y_c + self.rturn*math.sin(i)
				elif type == 'LSR':
					if part == "start":
						x = x_c + self.rturn*math.cos(i)
						y = y_c - self.rturn*math.sin(i)
					else:
						x = x_c - self.rturn*math.cos(i)
						y = y_c + self.rturn*math.sin(i)
				elif type=="RSL":
					if part == "start":
						x = x_c - self.rturn*math.cos(i)
						y = y_c + self.rturn*math.sin(i)
					else:
						x = x_c + self.rturn*math.cos(i)
						y = y_c - self.rturn*math.sin(i)
				if rectang.collidepoint(x,y):
					return True

		return False

	def connect(self, n1, n2):
		(x1,y1) = (self.coordinates[n1])
		(x2,y2) = (self.coordinates[n2])
		data = None
		if self.crossObstacleLine((x1,y1), (x2,y2)):
			self.remove_node(n2)
			return False

		theta_i = self.dubins_paths[n1][17]
		goal_theta = random.uniform(-math.pi, math.pi)
		mindist = sys.maxsize
		paths = [self.LSL((x1,y1),(x2,y2), theta_i, goal_theta), self.RSR((x1,y1),(x2,y2), theta_i, goal_theta),
		self.LSR((x1,y1),(x2,y2), theta_i, goal_theta), self.RSL((x1,y1),(x2,y2), theta_i, goal_theta)]
		for i in paths:
			path = i
			if path[0] == "None":
				continue
			if self.crossObstacleLine((path[5], path[6]), (path[7],path[8])):
				continue
			#if self.crossObstacleLine((x1,y1), (path[5], path[6])):
			if self.crossObstacleArc((x1,y1), (path[9], path[10]), (path[5], path[6]), path[2], path[13], path[14], path[0], "start"):
				#print("YEEEEEEEY")
				continue
			#if self.crossObstacleLine((x2,y2), (path[7], path[8])):
			if self.crossObstacleArc((x2,y2), (path[11], path[12]), (path[7], path[8]), path[3], path[16], path[15], path[0], "end"):
				#print("YEEEEEEEY")
				continue
			#print("The distance of", path[0],"is", path[1])
			if mindist > path[1]:
				mindist = path[1]
				data = path

		if data == None:
			self.remove_node(n2)
			return False
		else:
			#print("The shortest is", data[0],"with:", data[1])
			self.add_edge(n1, n2, data)
			return True

	def step(self, nnear, nrand, dmax=60):
		d = self.distance(nnear,nrand)
		if d>dmax:
			u = dmax/d
			(xnear, ynear) = (self.coordinates[nnear])
			(xrand, yrand) = (self.coordinates[nrand])
			(px,py) = (xrand-xnear, yrand-ynear)
			theta = math.atan2(py,px)
			(x, y)=(int(xnear+dmax*math.cos(theta)), int(ynear+dmax*math.sin(theta)))
			self.remove_node(nrand)
			if abs(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax:
				self.add_node(nrand, self.goal[0], self.goal[1])
				self.goalstate = nrand
				self.goalFlag = True
			else:
				self.add_node(nrand, x, y)

	def bias(self, ngoal):
		n = self.number_of_nodes()
		self.add_node(n, ngoal[0], ngoal[1])
		nnear = self.nearest(n)
		self.step(nnear,n)
		if self.connect(nnear,n):
			self.rewire(n)
		return self.coordinates, self.parent, self.dubins_paths

	def expand(self):
		n = self.number_of_nodes()
		x,y = self.sample_envir()
		self.add_node(n,x,y)
		#if self.isFree():
		xnearest = self.nearest(n)
		self.step(xnearest, n)
		if self.connect(xnearest, n):
			self.rewire(n)
		return self.coordinates, self.parent, self.dubins_paths

	def inradius(self, n):

		inradius = []
		nnear=0
		for i in range(0,n):
			if self.distance(i,n)<self.radius:
				if self.parent[n] == i:
					continue
				inradius.append(i)
		return inradius

	def rewire(self, n):

		new_dist = 0.0
		closest = None

		available = []

		lowest_cost = self.cost(n)
		#print(lowest_cost)
		inradius = self.inradius(n)

		for i in inradius:
			paths = [self.LSL(self.coordinates[i], self.coordinates[n], self.dubins_paths[i][17], self.dubins_paths[n][17]),
			self.RSR(self.coordinates[i], self.coordinates[n], self.dubins_paths[i][17], self.dubins_paths[n][17]),
			self.LSR(self.coordinates[i], self.coordinates[n], self.dubins_paths[i][17], self.dubins_paths[n][17]),
			self.RSL(self.coordinates[i], self.coordinates[n], self.dubins_paths[i][17], self.dubins_paths[n][17])]
			mindist = sys.maxsize
			data = None
			for j in paths:
				path = j
				if path[0] == "None":
					continue
				if self.crossObstacleLine((path[5], path[6]), (path[7],path[8])):
					continue
				if self.crossObstacleArc(self.coordinates[i], (path[9], path[10]), (path[5], path[6]), path[2], path[13], path[14], path[0], "start"):
					continue
				if self.crossObstacleArc(self.coordinates[n], (path[11], path[12]), (path[7], path[8]), path[3], path[16], path[15], path[0], "end"):
					continue
				if mindist > path[1]:
					mindist = path[1]
					data = path
			if data != None:
				available.append((i,data))

		for i in available:
			if i != None:
				new_dist = self.cost(i[0])+i[1][1]
				if lowest_cost > new_dist:
					lowest_cost = new_dist
					closest = i
		if closest != None:
			self.remove_edge(n)
			self.add_edge(closest[0], n, closest[1])

	def cost(self, n):
		dist = 0.0
		while(n != 0):
			dist +=  self.dubins_paths[n][1]
			n = self.parent[n]
		return dist





	def path_to_goal(self):
		if self.goalFlag:
			self.path = []
			self.path.append((self.goalstate, self.dubins_paths[self.goalstate]))
			newpos = self.parent[self.goalstate]
			while(newpos != 0):
				self.path.append((newpos, self.dubins_paths[newpos]))
				newpos = self.parent[newpos]

		return self.goalFlag


	def getPathCoords(self):
		pathCoords = []
		for node in self.path:
			x,y = (self.coordinates[node[0]])
			pathCoords.append(((x,y), node[1]))
		return pathCoords


	def LSL(self, first, second, theta_i, goal_theta):
		(x_i, y_i) = first
		(x_goal, y_goal) = second
		c1_x = x_i-self.rturn*math.cos(theta_i-math.pi/2)	#center of first circle of min_turning radius
		c1_y = y_i+self.rturn*math.sin(theta_i-math.pi/2)

		c2_x=x_goal-self.rturn*math.cos(goal_theta-math.pi/2)	#center of second circle of min_turning radius
		c2_y=y_goal+self.rturn*math.sin(goal_theta-math.pi/2)

		theta = (-1)*(np.arctan2(c2_x-c1_x, c2_y-c1_y))

		tan_pt1_x = c1_x-self.rturn*math.cos(theta)		#tangent point in first circle of min_turning radius
		tan_pt1_y = c1_y-self.rturn*math.sin(theta)

		tan_pt2_x= c2_x-self.rturn*math.cos(theta)		#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y-self.rturn*math.sin(theta)
		S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

		#Finding Initial Turning angle
		theta_i1 = np.arctan2(y_i - c1_y, x_i - c1_x)
		theta_i2 = np.arctan2(tan_pt1_y - c1_y,tan_pt1_x - c1_x)
		Ti = theta_i1 + theta_i2

		#Finding Final Turning angle
		theta_f1 = np.arctan2(tan_pt2_y- c2_y, tan_pt2_x - c2_x)
		theta_f2 = np.arctan2(y_goal- c2_y, x_goal - c2_x,)
		Tf = theta_f1 - theta_f2

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.rturn + Tf*self.rturn + S 	#Total Distance from current to Goal

		ans=["LSL",
			total_dist, Ti, Tf, S, tan_pt1_x, tan_pt1_y, tan_pt2_x, tan_pt2_y, c1_x, c1_y, c2_x, c2_y, theta_i1, theta_i2,
			theta_f1, theta_f2, goal_theta]
		return ans


	def RSR(self, first, second, theta_i, goal_theta):
		(x_i, y_i) = first
		(x_goal, y_goal) = second

		c1_x = x_i + self.rturn*math.cos(theta_i-(math.pi/2))	#center of first circle of min_turning radius
		c1_y = y_i - self.rturn*math.sin(theta_i-(math.pi/2))

		c2_x=x_goal + self.rturn*math.cos(goal_theta-(math.pi/2))	#center of second circle of min_turning radius
		c2_y=y_goal - self.rturn*math.sin(goal_theta-(math.pi/2))

		theta = (-1)*(np.arctan2(c2_x-c1_x, c2_y-c1_y))

		tan_pt1_x = c1_x+self.rturn*math.cos(theta)		#tangent point in first circle of min_turning radius
		tan_pt1_y = c1_y+self.rturn*math.sin(theta)

		tan_pt2_x=c2_x+self.rturn*math.cos(theta)		#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y+self.rturn*math.sin(theta)
		S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

		#Finding Initial Turning angle
		theta_i1 = np.arctan2(y_i - c1_y, x_i - c1_x ) #between initial and center
		theta_i2 = np.arctan2(tan_pt1_y - c1_y, tan_pt1_x - c1_x) #between tangent and center
		Ti = -theta_i1+theta_i2

		#Finding Final Turning angle
		theta_f1 = np.arctan2(tan_pt2_y- c2_y, tan_pt2_x - c2_x) #between tangent and center
		theta_f2 = np.arctan2(y_goal- c2_y, x_goal - c2_x,) #between final and center
		Tf = -theta_f1 + theta_f2

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.rturn + Tf*self.rturn + S 	#Total Distance from current to Goal

		ans=["RSR",
			total_dist, Ti, Tf, S, tan_pt1_x, tan_pt1_y, tan_pt2_x, tan_pt2_y, c1_x, c1_y, c2_x, c2_y, theta_i1, theta_i2,
			theta_f1, theta_f2, goal_theta]
		return ans


	def LSR(self, first, second, theta_i, goal_theta):
		(x_i, y_i) = first
		(x_goal, y_goal) = second

		c1_x = x_i - self.rturn*math.cos(theta_i-(math.pi/2))	#center of first circle of min_turning radius
		c1_y = y_i + self.rturn*math.sin(theta_i-(math.pi/2))

		c2_x=x_goal + self.rturn*math.cos(goal_theta-(math.pi/2))	#center of second circle of min_turning radius
		c2_y=y_goal - self.rturn*math.sin(goal_theta-(math.pi/2))

		S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)
		if (-self.rturn**2 + (S1**2)/4.0 ) < 0:
			ans = ["None"]
			return ans
		S = 2*((-self.rturn**2 + (S1**2)/4.0 )**0.5)

		theta =   (-1)*(np.arctan2((c2_x-c1_x), c2_y-c1_y)) - np.arctan2(self.rturn,S/2.0)

		tan_pt1_x=c1_x-self.rturn*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y-self.rturn*math.sin(theta)

		tan_pt2_x=c2_x+self.rturn*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y+self.rturn*math.sin(theta)

		#Finding Initial Turning angle
		theta_i1 = np.arctan2(y_i - c1_y, x_i - c1_x ) #between initial and center
		theta_i2 = np.arctan2(tan_pt1_y - c1_y, tan_pt1_x - c1_x) #between tangent and center
		Ti = theta_i1-theta_i2

		#Finding Final Turning angle
		theta_f1 = np.arctan2(tan_pt2_y- c2_y, tan_pt2_x - c2_x) #between tangent and center
		theta_f2 = np.arctan2(y_goal- c2_y, x_goal - c2_x,) #between final and center
		Tf = -theta_f1 + theta_f2

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.rturn + Tf*self.rturn + S 	#Total Distance from current to Goal

		ans=["LSR",
			total_dist, Ti, Tf, S, tan_pt1_x, tan_pt1_y, tan_pt2_x, tan_pt2_y, c1_x, c1_y, c2_x, c2_y, theta_i1, theta_i2,
			theta_f1, theta_f2, goal_theta]
		return ans


	def RSL(self, first, second, theta_i, goal_theta):
		(x_i, y_i) = first
		(x_goal, y_goal) = second

		c1_x = x_i + self.rturn*math.cos(theta_i-(math.pi/2))	#center of first circle of min_turning radius
		c1_y = y_i - self.rturn*math.sin(theta_i-(math.pi/2))

		c2_x = x_goal - self.rturn*math.cos(goal_theta-(math.pi/2))	#center of second circle of min_turning radius
		c2_y = y_goal + self.rturn*math.sin(goal_theta-(math.pi/2))

		S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)
		if (-self.rturn**2 + (S1**2)/4.0 ) < 0:
			ans = ["None"]
			return ans
		S = 2*((-self.rturn**2 + (S1**2)/4.0 )**0.5)

		theta = (-1)*(np.arctan2((c2_x-c1_x), c2_y-c1_y)) + np.arctan2(self.rturn,S/2.0)

		tan_pt1_x = c1_x + self.rturn*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y = c1_y + self.rturn*math.sin(theta)

		tan_pt2_x = c2_x - self.rturn*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y = c2_y - self.rturn*math.sin(theta)

		#Finding Initial Turning angle
		theta_i1 = np.arctan2(y_i - c1_y, x_i - c1_x ) #between initial and center
		theta_i2 = np.arctan2(tan_pt1_y - c1_y, tan_pt1_x - c1_x) #between tangent and center
		Ti = -theta_i1+theta_i2

		#Finding Final Turning angle
		theta_f1 = np.arctan2(tan_pt2_y- c2_y, tan_pt2_x - c2_x) #between tangent and center
		theta_f2 = np.arctan2(y_goal- c2_y, x_goal - c2_x) #between final and center
		Tf = theta_f1 - theta_f2

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.rturn + Tf*self.rturn + S 	#Total Distance from current to Goal

		ans=["RSL",
			total_dist, Ti, Tf, S, tan_pt1_x, tan_pt1_y, tan_pt2_x, tan_pt2_y, c1_x, c1_y, c2_x, c2_y, theta_i1, theta_i2,
			theta_f1, theta_f2, goal_theta]
		return ans


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
