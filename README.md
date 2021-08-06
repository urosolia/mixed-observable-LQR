# The Mixed-Observable Constrained Linear QuadraticRegulator Problem

We study the problem of steering a linear time-invariant system towards a goal location that may be inferred only thought partial observations. We assume mixed observable settings, where the system's state is fully observable and the environment's state defining the goal location is only partially observable. This repo can be used to reproduce the result from [1].

## Example 1: Mixed Observable Regulation Problem
In this example, the environment is static and there are two environment states: e = 1 and e = 2 (each one associated with a goal location as shown in the figure below).
The environment state, and consequently the goal location, is unknwon and it should be inferred from partial observations. The accuracy of the sensors is a function of the system's position, for more details about the observation model please refer to [1].

<p align="center">
<img src="https://github.com/urosolia/mixed-observable-LQR/blob/main/figures/lqr.png" width="500" />
</p>

## Example 2: Partially Observable Navigation Problem

In this example, the environment is static and there are two environment states: e = 1 and e = 2 (each one associated with a goal location as shown in the figure below). The main difference compared to the previous example is that a few obstacles (black boxes) are present in the environment.
Also in this case, the environment state is unknown and it should be inferred from partial observations. The accuracy of the sensors is a function of the system's position, for more details about the observation model please refer to [1].

<p align="center">
<img src="https://github.com/urosolia/mixed-observable-LQR/blob/main/figures/navigation.png" width="700" />
</p>


## References

[1] "The Mixed-Observable Constrained Linear QuadraticRegulator Problem: the Exact Solution"