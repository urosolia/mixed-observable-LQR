## The Mixed-Observable Constrained Linear QuadraticRegulator Problem

# Abstract 

We study the problem of steering a linear time-invariant system towards a goal location that may be inferred only thought partial observations. We assume mixed observable settings, where the system's state is fully observable and the environment's state defining the goal location is only partially observable. In these settings, the control problem is an infinite-dimensional optimization problem where the objective is to minimize the expected cost. We show how to reformulate the control problem as a finite-dimensional deterministic problem by optimizing over a trajectory tree. Leveraging this result, we demonstrate that when the environment is static, the observation model piecewise, and cost function convex, the original control problem can be reformulated as a Mixed Integer Convex Program (MICP) that can be solved to global optimality using a branch-and-bound algorithm. The effectiveness of the proposed approach is demonstrated on navigation tasks, where the system has to reach a goal location identified from partial observations.

# Example 1: Mixed Observable Regulation Problem


# Example 2: Partially Observable Navigation Problem



