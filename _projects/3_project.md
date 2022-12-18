---
layout: page
title: FMPC for Quadcopter Trajectory Tracking
description: Flatness-based Model Predictive Controller (FMPC) for quadrotor trajectory tracking.
img: assets/img/QuadcopterSim.png
importance: 3
category: research
---

The Flatness-based Model Predictive Control (FMPC) architecture, initially proposed by Greeff et al., coupled a MPC controller with feedforward linearization, in which the future optimal states are used to cancel the nonlinear terms. The model mapped the nonlinear dynamics of the quadcopter as equivalent linear dynamics by exploiting the differential flatness property found in many symmetric, mechanical systems, such as quadcopters. With the states of the model now expressed in terms of the linear flat output space, I was able to re-formulate the MPC optimization problem as a quadratic program and was able to run it well over 60 Hz using CVX’s MATLAB toolbox. When compared to Linear MPC and Nonlinear MPC, FMPC showed optimal results for real-time tracking of complex trajectories as it was able to account for many of the system’s nonlinear effects while being computationally efficient.


**Problem Formulation** 

The states and inputs for the nonlinear quadcopter dynamics are defined as:

$$
\begin{equation}
x = [x, y, z, \dot{x}, \dot{y}, \dot{z}, \phi, \theta, \Psi]^{T} \\
\end{equation}
$$

$$
\begin{equation}
u = [\dot{z}_{cmd}, \phi_{cmd}, \theta_{cmd}, \dot{\Psi}_{cmd}]^{T}
\end{equation}
$$

And the nonlinear system model is defined by:

$$
\begin{equation}
\dot{x} = f(x(t), u(t)) = \begin{bmatrix}
           \dot{x} \\
           \dot{y} \\
           \dot{z} \\
           \ddot{x} \\
           \ddot{y} \\
           \ddot{z} \\
           \dot{\phi} \\
           \dot{\theta} \\
           \dot{\Psi} \\
         \end{bmatrix} = \begin{bmatrix}
           \dot{x} \\
           \dot{y} \\
           \dot{z} \\
           \frac{sin(\phi)sin(\Psi)+cos(\phi)sin(\theta)cos(\Psi)(\ddot{z}+g)}{cos(\theta)cos(\phi)} \\
           \frac{-sin(\phi)sin(\Psi)+cos(\phi)sin(\theta)sin(\Psi)(\ddot{z}+g)}{cos(\theta)cos(\phi)} \\
           \frac{1}{\tau_{z}} (\dot{z}_{cmd} - \dot{z}) \\
           \frac{1}{\tau_{\phi}} (\phi_{cmd} - \phi) \\
           \frac{1}{\tau{}_{\theta}} (\theta_{cmd} - \theta) \\
           \dot{\Psi}_{cmd} \\
         \end{bmatrix}
\end{equation}
$$

Using quadcopter's differential flatness properties found in quadcopters, the differentially flat system could be represented using a Brunovský (flat) state. This means that the states and inputs can be expressed in terms of a flat output and its derivatives, as such:
$$
\begin{equation}
\boldsymbol{\sigma} = [x,y,z,\Psi]^{T}
\end{equation}
$$

With the flat state and flat inputs defined as:

$$
\begin{equation}
\boldsymbol{z} = [x, y, z, \dot{x}, \dot{y}, \dot{z}, \ddot{x}, \ddot{y}, \Psi]^{T} 
\end{equation}
$$

$$
\begin{equation}
\boldsymbol{v} = [\dddot{x}, \dddot{y}, \dddot{z}, \dot{\Psi}]^{T}
\end{equation}
$$

The mapping between flat state (5) to the actual state (1) is expressed as:
$$
\begin{equation}
z(t) = \Phi^{-1}(x(t)) = \begin{bmatrix}
            x \\
            y \\
            z \\
            \dot{x} \\
            \dot{y} \\
            \dot{z} \\
            g\left(\frac{sin(\Psi)tan(\phi)}{cos\theta} + cos(\Psi)tan(\theta)\right) \\
            g\left(\frac{-cos(\Psi)tan(\phi)}{cos\theta} + sin(\Psi)tan(\theta)\right) \\
            \Psi \\
         \end{bmatrix}
\end{equation}
$$

Note: a major assumption made is that $$\dot{z}$$ is constant ($$\ddot{z}$$ = 0). 

The mapping between the input $$u(t)$$ from the flat state $$z(t)$$ and flat input $$v(t)$$ is given by:

$$
\begin{equation}
u(t) = \Psi^{-1}(z(t), v(t)) = \begin{bmatrix}
            \tau_{z}\ddot{z} + \dot{z} \\
            \frac{\tau_{\phi}}{1+\alpha^2}\frac{d\alpha}{dt} + \tan(\alpha) \\
            \frac{\tau_{\theta}}{1+\beta^2}\frac{d\beta}{dt} + \tan(\beta) \\
            \dot{\Psi} \\
         \end{bmatrix}
\end{equation}
$$

Where $$\alpha$$ & $$\beta$$ are defined as:
$$
\begin{equation}
\alpha = \frac{1}{\ddot{z}+g}(cos(\Psi)\ddot{x} + sin(\Psi)\ddot{y})
\end{equation}
$$

$$
\begin{equation}
\beta = \frac{1}{\ddot{z}+g}(sin(\Psi)\ddot{x} - cos(\Psi)\ddot{y})
\end{equation}
$$

The overall system was then expressed as a discrete linear system in terms of the flat states and inputs:
$$
\begin{equation}
z_{k+1} = Az_{k} + Bv_k
\end{equation}
$$

The MPC optimization problem was then formulated as a quadratic cost function using the flat space:

$$
\begin{equation}
    \begin{aligned}
    & \min_{v_{0:k}} \sum_{k=1}^{N} \Big[ \frac{1}{2} (\zeta_{k} - \zeta_{ref,k})^T Q (\zeta_{k} - \zeta_{ref,k}) + \frac{1}{2} v_{k-1}^T R v_{k-1}  \Big] \\
    & \ni z_{k+1} = A z_k + B v_k,  \\
    &    u_{k} = Mz_{k} + Nv_{k}, \\
    &    \zeta_{k} = Cz_{k} 
    \end{aligned}
\end{equation}
$$

Where $$M$$, $$N$$, and $$Z$$ are permutation matrices that simply pick out the flat values required for formulation.

**Solving the problem**

The MPC optimization problem above can be formulated as a quadratic program:
$$
\begin{equation}
    \begin{aligned}
    & \min_{x} \frac{1}{2} x P^T x + q^{T}x \\
    & \ni Ax = b  \\
    \end{aligned}
\end{equation}
$$

The optimal input can then be calculated at each time step using the optimal flat state and flat input using equation (8):
$$
\begin{equation}
    \begin{aligned}
    u_{op}(t) = \Psi^{-1}(z_{op}, v_{op})
    \end{aligned}
\end{equation}
$$
