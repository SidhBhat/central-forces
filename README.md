# central-forces

## Introduction

This is a Octave/matlab script to calculate the trajectory a particle under a central
force \(force that acts in the direction toward \- or away from \- a fixed central
point\). Here the force is proportional to some power $k$ of the distance $r$ between
the central point and the particle.

## Theory

The [binet equation](https://en.wikipedia.org/wiki/Binet_equation) gives the relation
between "force law" and the trajectory of the particle.

$$
F = - m h^2 u^2 \left(\frac{d^2u}{d\theta^2} + u\right)
$$

where $u = 1/r$ and the force is proportional to some power $k$ or $r$: 

$$
F = -\mu r^k
$$

Combining the two we get:

$$
\frac{d^2u}{d\theta^2} = \frac{A}{u^{2 + k}} - u
$$

where $A = \frac{\mu}{ m h^2}$ is a constant.  

we use Keplers second law to calculate $h$ from the initial conditions, and than
ocatve's [lsode](https://octave.sourceforge.io/octave/function/lsode.html) to numerically solve the
non-linear ODE above.

## Running the script

As with all my octave/matla scripts we have to set some variables inside the script files to definve the
environment. With even basic scripting knowlage this should be easy to do. You are not supposed to edit
any other part outside the configuration section unless you know what you are doing.  

We define the particle itself:

```matlab
% Mass of the body
m  = 1;
% Initial Position
x = [0, 0.909];
% Initial Velocity
v = [2.5, 0];
```

Than we have to set a constant of proprtion `mu` . By default this constant is configured for gravity
but you can set it to any constant value.

```matlab
M  = 0.826; % Central mass
G  = 4.5;   % Gravitational Constant
% The constant of proportion
mu = G*M*m;
```

We also have to define the power on $r$, `-2` by default:

```matlab
% power of the distance to which the force is proportional
global k = -2;
```

lastly we tell the script when to stop, that is how many revolutions to around the central point to compute

```matlab
% Number of revolutions
n = 2;
% Number of data points (plot)
N = 1000;
```

## Gravitation

Newtons law of gravity is a special case where $k = -2$ and $\mu = G M m$[^1]. In this case the differential equation
reduces to a linear ODE, which we can solve:

$$
\begin{align}
\frac{d^2u}{d\theta^2} =& A - u \\
\implies u =& A + M \sin (\theta + \phi)
\end{align}
$$

I have another [script](https://gist.github.com/SidhBhat/fdb6178252a29f58f185c923bf40b8f8) that computes the particular
solution given initial conditions, without carring out the numerical integration of lsode.

Here it is important to note that the trajectory calculated thus is valid when `M >> m`. This is because the central
force does not take into account the movement of the certral body itself. In the script the value of `m` has absolutly no
effect on the path when it is run for gravitational force. `m` get nullified in $A = \frac{\mu}{m h^2} = \frac{G M}{h^2}$

To get correct results when the two masses are comparable, replace $M$ with $M \left( \frac{M}{M + m}\right)^2$. We can call
this the equivalent barycentric mass of a two body system[^2].

$$
\mu = G \left( M \left( \frac{M}{M + m}\right)^2 \right) m
$$

## Images

Keplerian orbit with centre at a focus:

![Keplerian Orbit](/images/Keplarian_orbit.png)

Hook's orbit with centre being the elliptic centre:

![Hooks Orbit](/images/Hooks_Orbit.png)

[^1]: `mu` must not be confused with the standard gravitational parameter `mu = GM`
[^2]: It can be shown that the force on each body in a two body system is also inversly propotional  
to the distance to barycentre
