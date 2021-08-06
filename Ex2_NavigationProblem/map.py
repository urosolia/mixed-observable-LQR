import numpy as np
import pdb 
from polytope import polytope

class MAP(object):

	def __init__(self, opt, printLevel = 0):
		# Define variables
		self.printLevel = printLevel
		self.bVecList = []
		self.obs_bVecList = []
		F = np.vstack((np.eye(2), -np.eye(2)))
		
		self.goal  = []
		self.poliObst = []

		recList  = []
		obstList = []
		if opt == 1:
			recList.append([-1, -0.12,  14, 0.24]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			recList.append([-5, -0.12,   4, 0.24]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!

			self.goal.append(np.array([12.0,  0.1, 0.0, 0.0]))
			self.goal.append(np.array([12.0, -0.1, 0.0, 0.0]))
		
		elif opt == 2:
			recList.append([ 0,   0,  5, 7.5]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			recList.append([10,   0,  5, 7.5])
			recList.append([ 0, 7.5, 15, 7.5])
		
			self.goal.append(np.array([12.5, 15.0, 0.0, 0.0]))
			self.goal.append(np.array([19.0, 10.0, 0.0, 0.0]))

		elif opt == 3:
			recList.append([   0,  0,   5,  20]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			recList.append([   0, 20,  10,   5])
			recList.append([  10, 20,  10,   5])
			recList.append([  10,7.5,   5,12.5])
			recList.append([   5,  5,  10, 2.5])
			recList.append([  10,2.5,  10, 2.5])
			recList.append([   5,  0,  15, 2.5])
			recList.append([17.5,  5, 2.5, 7.5])
		
			self.goal.append(np.array([12.5, 15.0, 0.0, 0.0]))
			self.goal.append(np.array([19.0, 10.0, 0.0, 0.0]))
		
		elif opt == 4:
			recList.append([-1, -10.0,  16, 20.0]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			recList.append([-5, -10.0,   4, 20.0]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!

			self.goal.append(np.array([14.0,  8.0, 0.0, 0.0]))
			self.goal.append(np.array([14.0, -8.0, 0.0, 0.0]))
		
		elif opt == 5:
			recList.append([   0,     0,  15,   5]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			recList.append([ 2.5,     5,   5,   5]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			recList.append([10.0,     5,   5,   5]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			recList.append([   0,    10,  15,   5]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!

			# self.goal.append(np.array([10.0, 27.0, 0.0, 0.0]))
			# self.goal.append(np.array([25.0, 15.0, 0.0, 0.0]))
			self.goal.append(np.array([ 2.0, 12.0, 0.0, 0.0]))
			self.goal.append(np.array([14.0,  8.0, 0.0, 0.0]))

			obstList.append([   0,   5,  2.5,  5]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
			obstList.append([  7.5,  5,  2.5,  5]) # Each rec = [ x, y, width, height ] where (x,y) = bottom left!
		
		self.poliList = []
		for rec in recList:
			self.bVecList.append(self.rec2bVec(rec))
			self.poliList.append(polytope(F, self.bVecList[-1], printLevel = printLevel))

		self.poliObst = []
		for rec in obstList:
			self.obs_bVecList.append(self.rec2bVec(rec))
			self.poliObst.append(polytope(F, self.obs_bVecList[-1], printLevel = printLevel))


	def rec2bVec(self, rec):
		# The rectangles will be represented as Polytopes Fx <= b. Here computing the vector b
		xmin = rec[0]
		ymin = rec[1]

		xmax = rec[0]+rec[2]
		ymax = rec[1]+rec[3]
		
		bVec = [xmax, ymax, -xmin, -ymin]
		if (self.printLevel >= 1): print(bVec)
		return np.array(bVec).astype(float)


	def plotMap(self, ax=[]):
		color =['b','r','y','g','r','k','b','r','k','b','r','k','b','r','k','b','r','k','b','r','k','b','r','k']
		color.append('k') 
		i = 0
		for poli in self.poliList:
			poli.plot2DPolytopePath(color[i], ax)
			i+=1

		for poli in self.poliObst:
			poli.plot2DPolytopeObstacles('k', ax)
			i+=1