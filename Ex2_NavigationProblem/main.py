import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from ftocp import FTOCP
from map import MAP
import pdb
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
# plt.rc('font', size=12)

# ==============================================================================================
# Initial Conditions
x0     = np.array([5,2,0,0])
bt     = [np.array([0.5, 0.5])]
# ==============================================================================================
# Build Map
opt = 5
mapEnv = MAP(opt, printLevel = 1)
fig = plt.figure()
ax = plt.gca()
mapEnv.plotMap(ax)
plt.plot(x0[0], x0[1], 'sk', label='Initial Condition')

for i in range(0, len(mapEnv.goal)):
	plt.plot(mapEnv.goal[i][0], mapEnv.goal[i][1], 's', label='Goal '+str(i+1))

plt.legend(loc='upper right')
plt.show()
# ==============================================================================================
# Define system dynamics
dt = 0.1
A = np.array([[1, 0, dt,  0], 
	          [0, 1,  0, dt],
	          [0, 0,  1,  0],
	          [0, 0,  0,  1]])

B = np.array([[ 0, 0], 
	          [ 0, 0],
	          [dt, 0],
	          [ 0, dt]])

n = A.shape[1]
d = B.shape[1]

# ==============================================================================================
# Define system MPC problem
Q  = 0.0001  * np.eye(n)
# Q  = 0.0  * np.eye(n)
# Q[1,1] = 0.1
# Q[2,2] = 0.1
# Q[3,3] = 0.1

R  = 0.01 * np.eye(d)
Qf = 10.0 * np.eye(n)
N  = 30
Nb = 10

a_max = 15.0
ftocp = FTOCP(N, Nb, mapEnv.goal, A, B, Q, R, Qf, a_max, mapEnv.bVecList, p1=0.7, printLevel = 0)
ftocp.build()
bPred  = ftocp.initBelief(bt[-1])
ftocp.solve(x0,bPred)

# ==============================================================================================
# Plot closed-loop trajectory
fig = plt.figure()
ax = plt.gca()
mapEnv.plotMap(ax)
for j in range(0, ftocp.numSegments):
	if j == 0:
		plt.plot(ftocp.xPredList[j][0,:], ftocp.xPredList[j][1,:], '--*g', label='Optimal Trajectory')
		plt.plot(ftocp.xPredList[j][0,0], ftocp.xPredList[j][1,0], 'sk', label='Initial Condition')
	else:
		plt.plot(ftocp.xPredList[j][0,:], ftocp.xPredList[j][1,:], '--*g')
		idx = (j-1) // ftocp.numO
		if j == 1:
			plt.plot([ftocp.xPredList[j][0,0], ftocp.xPredList[idx][0,-1]], [ftocp.xPredList[j][1,0], ftocp.xPredList[idx][1,-1]], '*r', label='Branching time')
		else:
			plt.plot([ftocp.xPredList[j][0,0], ftocp.xPredList[idx][0,-1]], [ftocp.xPredList[j][1,0], ftocp.xPredList[idx][1,-1]], '--*r')

for i in range(0, len(ftocp.goal)):
	plt.plot(ftocp.goal[i][0], ftocp.goal[i][1], 's', label='Goal for $e='+str(i)+'$')

plt.legend(loc='upper right')
plt.show()
