import numpy as np
import pdb 
import scipy
from cvxpy import *
import datetime

class FTOCP(object):
	""" Finite Time Optimal Control Problem (FTOCP)
	Methods:
		- solve: solves the FTOCP given the initial condition x0, terminal contraints (optinal) and terminal cost (optional)
		- model: given x_t and u_t computes x_{t+1} = Ax_t + Bu_t

	"""
	def __init__(self, N, Nb, goal, A, B, Q, R, Qf, a_max, bVecList, p1 = 0.85, printLevel = 0):
		# Define variables
		self.N  = N # Horizon Length
		self.Nb = Nb

		# System Dynamics (x_{k+1} = A x_k + Bu_k)
		self.A = A 
		self.B = B 
		self.n = A.shape[1]
		self.d = B.shape[1]

		# Cost (h(x,u) = x^TQx +u^TRu)
		self.Q = Q**0.5
		self.R = R
		self.Qf = Qf**0.5
		self.goal = goal

		# Initialize Predicted Trajectory
		self.xPred = []
		self.uPred = []

		self.F = np.vstack((np.eye(2), -np.eye(2)))

		self.ax_max = a_max
		self.ay_max = self.ax_max
		self.bVecInput = np.array([self.ax_max, self.ay_max]*(2))

		self.bMatrix = np.array(bVecList).T
		self.btot    = np.max(np.array(bVecList).T, 1)
		self.numReg  = self.bMatrix.shape[1]

		self.printLevel = printLevel

		self.numO = 2
		self.eDim = 2

		self.D_reg = []
		self.M = np.zeros((2,2))
		self.M[0,0] = 1.0/p1
		self.M[1,0] = 1.0/(1-p1)
		self.M[0,1] = 1.0/(1-p1)
		self.M[1,1] = 1.0/p1
		self.D_reg.append(self.M)

		p = 0.85
		self.M1 = np.zeros((2,2))
		self.M1[0,0] = 1.0/p
		self.M1[1,0] = 1.0/(1-p)
		self.M1[0,1] = 1.0/(1-p)
		self.M1[1,1] = 1.0/p

		self.D_reg.append(self.M1)

		if N % Nb != 0:
			print("Error N % Nb != 0")
		else:
			self.P = N // Nb
			self.numSegments = 0
			for i in range(0, self.P):
				self.numSegments += self.numO**i

			print("Total number of segments: ", self.numSegments, " N_b: ", Nb)

	def initBelief(self, b):
		belief = [b]
		for j in range(1, self.numSegments):
			idx = (j-1) // self.numO
			o = (j-1) % self.numO
			bn = []
			# print("numSegments: ", j, "idx: ", idx, " o: ", o)
			for e in range(0, self.eDim):
				bn.append(1/self.M[o, e] * belief[idx][e])
			belief.append(bn)

		return belief

	def beliefUpdate(self, belief):
		o = random.randint(0,1)
		print("o: ", o)
		bn = []
		for e in range(0, self.eDim):
				bn.append(self.O[o, e] * belief[e])
		bn = bn / np.sum(bn)
		return bn

	def build(self):
		"""This method build the problem
		""" 
		startTimer = datetime.datetime.now()

		# Initialize Variables
		self.x      = [Variable((self.n, self.Nb+1)) for a in range(0, self.numSegments)]
		self.u      = [Variable((self.d, self.Nb)) for a in range(0, self.numSegments)]
		self.regVar = [Variable((self.numReg, 1), boolean=True) for a in range(0, self.numSegments - self.numO**(self.P-1))] # Initialize vector of variables
		self.x0     = Parameter(self.n)
		self.b0     = Parameter(self.numO)
		self.bPar   = [Variable(self.numO, nonneg=True) for a in range(0, self.numSegments)]
		self.aPar   = [Variable((self.numO, self.numReg)) for a in range(0, self.numSegments)]
		self.bigM   = Parameter()

		# State Constraints
		constr = []
		for j in range(0, self.numSegments):
			if j == 0:
				constr += [self.x[j][:,0] == self.x0] # This is for readibility this variable can be removed
			else:
				idx = (j-1) // self.numO
				constr += [self.x[j][:,0] == self.x[idx][:,-1]] # This is for readibility this variable can be removed
	
			for i in range(0, self.Nb):
				constr += [self.x[j][:,i+1] == self.A @ self.x[j][:,i] + self.B @ self.u[j][:,i],
							self.F@self.u[j][:,i]     <= self.bVecInput,
							self.F@self.x[j][0:2,i+1] <= self.btot,]							
			
			if j < self.numSegments - self.numO**(self.P-1):
				constr += [self.F@self.x[j][0:2,-1] <= self.bMatrix@self.regVar[j][:, -1],
							sum(self.regVar[j][:, -1]) == 1]

		# Belief constraint
		constr += [self.bPar[0] == self.b0] 
		# for j in range(1, self.numSegments):
		# 	o = (j-1) % self.numO
		# 	idx = (j-1) // self.numO
		# 	for e in range(0, self.numO):
		# 		constr += [self.bPar[j][e] == (1/self.O[o, e]) * self.bPar[idx][e]]
		for j in range(1, self.numSegments):
			o = (j-1) % self.numO
			idx = (j-1) // self.numO
			for k in range(0, self.numReg):
				for e in range(0, self.numO):
					constr += [self.aPar[j][e, k]  <= self.bigM * self.regVar[idx][k, -1],
								self.aPar[j][e, k] >= 0,
								self.aPar[j][e, k] <= (self.D_reg[k][o, e]) * self.bPar[idx][e],
								self.aPar[j][e, k] >= (self.D_reg[k][o, e]) * self.bPar[idx][e] - self.bigM*(1 - self.regVar[idx][k, -1]),]
			
		for j in range(1, self.numSegments):
			constr += [self.bPar[j] ==  self.aPar[j] @ np.ones(self.numReg)]

		# Cost Function
		self.cost = 0
		for j in range(0, self.numSegments):
			for i in range(0, self.Nb):
				self.cost += quad_form(self.u[j][:,i], self.R)
				for k in range(0, len(self.goal)):
					self.cost += quad_over_lin(self.Q @ (self.x[j][:,i+1]-self.goal[k]), self.bPar[j][k])

		for i in range(0, self.numO**(self.P-1)):
			idx = self.numSegments - i - 1
			for k in range(0, len(self.goal)):
				self.cost += quad_over_lin(self.Qf @ (self.x[idx][:,-1]-self.goal[k]), self.bPar[idx][k])

		# Build the Finite Time Optimal Control Problem
		self.problem = Problem(Minimize(self.cost), constr)
		endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
		if self.printLevel >= 1: print("Problem build in: ", deltaTimer.total_seconds(), " seconds.")


	def solve(self, x0, b0):
		"""This method solves an FTOCP given:
			- x0: initial condition
		""" 
		startTimer = datetime.datetime.now()
		
		# for bopt, b in zip(self.bPar, b0):
		# 	bopt.value = b
		# for b in b0:
		# 	print(b)
		self.b0.value = 1/b0[0]
		self.bigM.value = np.max(self.D_reg)**(self.P-1) * np.max(1/b0[0])
		self.x0.value   = x0
		# print(1/b0[0])

		self.problem.solve(verbose=0, warmstart=True)
		endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
		if self.printLevel >= 0: 
			print("Problem solved in: ", deltaTimer.total_seconds(), " seconds.")
			print("Cost: ", self.cost.value)
		# Store the open-loop predicted trajectory
		self.xPred = np.column_stack([x.value for x in self.x])
		self.uPred = np.column_stack([u.value for u in self.u])
		self.bPred = np.column_stack([b.value for b in self.bPar])

		self.regVarPred = np.column_stack([b.value for b in self.regVar])

		self.xPredList = [x.value for x in self.x]
		self.uPredList = [u.value for u in self.u]

		if self.printLevel >= 2: 
			print("xPred.T")
			print(np.round(self.xPred.T,4))
			print("uPred.T")
			print(np.round(self.uPred.T,4))
			print("bPred.T")
			print(np.round(self.bPred.T,4))
			print("self.regVarPred")
			print(np.array(self.regVarPred))


	

