# The Mixed-Observable Constrained Linear QuadraticRegulator Problem

## Abstract 

We study the problem of steering a linear time-invariant system towards a goal location that may be inferred only thought partial observations. We assume mixed observable settings, where the system's state is fully observable and the environment's state defining the goal location is only partially observable. This repo can be used to reproduce the result from [1].

## Example 1: Mixed Observable Regulation Problem
In this example, the environment is static and the set of partially observable states $$\mathcal{E} = \{0, 1\}$$, and the goal location is a function of the partially observable environment state:
$$
    x_g^{(0)} = \begin{bmatrix*}[r] 14 \\ 8 \\ 0 \\ 0 \end{bmatrix*} \text{ and } x_g^{(1)} = \begin{bmatrix*}[r] 14 \\ -8 \\ 0 \\ 0 \end{bmatrix*}.
$$

The environment state, and consequently the goal location, is inferred though partial observations. Given the true environment state $e \in \mathcal{E}$ and the state of the system $x \in \mathbb{R}^n$, the probability of measuring the observation $o = e$ is given by the following piecewise observation model:
$$
    Z(o=e, e, x)=\mathbb{P}(o=e|e,x) = \begin{cases}p_1 & \mbox{If } x \in \mathcal{X}_1 \\
    p_2 = 0.85 & \mbox{If } x \in \mathcal{X}_2 \end{cases},
$$
where 
$$
    \mathcal{X}_1 &= \{[X, Y, v^x, v^y] \in \mathbb{R}^4 : -1 \leq X \leq ~15, ||Y||_\infty \leq 10 \} \\
    \mathcal{X}_2 &= \{[X, Y, v^x, v^y] \in \mathbb{R}^4 : -5 \leq X < -1, ||Y||_\infty \leq 10 \}.
$$

<p align="center">
<img src="https://github.com/urosolia/mixed-observable-LQR/blob/main/figures/lqr.gif" width="500" />
</p>

## Example 2: Partially Observable Navigation Problem

<p align="center">
<img src="https://github.com/urosolia/mixed-observable-LQR/blob/main/figures/navigation.gif" width="500" />
</p>


## References

[1] "The Mixed-Observable Constrained Linear QuadraticRegulator Problem: the Exact Solution"