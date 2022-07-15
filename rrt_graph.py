import random
import math
import numpy as np
import pygame
import sys
import time
import cv2

def distance(first, second):
	(x1,y1) = first
	(x2,y2) = second
	px = (float(x1)-float(x2))**2
	py = (float(y1)-float(y2))**2
	return (px+py)**0.5

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
