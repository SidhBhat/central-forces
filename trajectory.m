% Motion under a Central force.

% A central force on a body is a force that attracts it to a central point.
% the motion of such a body always lies in a plane.

% In this script we try to derive the trajectory of a particle attracted to the origin
% where the force is proportional to some power of the distance between the particle and the
% origin.

% Clear Existing Values
clear all;
clf;

%##------------- Start Configuration ----------------##%

%  Mass of the body
m  = 1;
%% Initial Conditions
%  Initial Position
x = [0, 0.909];
%  Initial Velocity
v = [2.5, 0];

%% Environment
% For Gravity type relation
M  = 0.826; % Central mass
G  = 4.5;   % Gravitational Constant
% The constant of proportion
mu = G*M*m;
% ^ Change to some other value if you do not want gravity type force
% (here "gravity type" refers to a force proportional to the central mass)
% power of the distance to which the force is proportional
global k = 1;
% ^ You must keep it global

%% Simulation and plot
% Number of revolutions
n = 1;
% Number of data points (plot)
N = 1000;

%##------------- End Configuration ----------------##%

% x = initial postion (cartesian)
% v = initial velocity (cartesian)

% We use polar coordinates as it is more convienient for central forces
% r = distance to body from centre
% theta = angle to x axis
% u = 1/r
% the force is of type:
% F = K*r^k;
% k is the power on r. k = -2 for Newtons law of gravity.

% Initial r
r_0  = norm(x);
% Initial u
u_0  = 1/r_0;
% Initial theta
theta_0 = acos(dot(x,[1,0])/norm(x));
% Angular momentum per unit mass h = r^2*(dtheta/dt) = norm(cross(v,x)) = constant
% but we take a vector h
h    = cross([x, 0], [v, 0]);
% rdot = dr/dt = norm(projection of v on x)
rdot = dot(v,x)/norm(x);

%%%% -----------Setting up the differential equation-------------%%%%%
global A = mu / norm(h)^2 / m;
% seed for lsode (pls see lsode doc)
% U = [u, u'];
U0   = [u_0, -rdot/norm(h)];
% Udot: function that gives dU/dt for a initial U
% t is the independant variable theta here
% Since U is a vector we can encode any differential equation
function Udot = Udot(U, t)
    global A;
    global k;
    % the differential equation represented here is:
    % u'' = A/u^(2+k) - u;

    Udot(1) = U(2);
    Udot(2) = A/U(1)^(2+k) - U(1);
endfunction

%% solving the equation %%
theta = linspace (theta_0, theta_0 + 2*pi*n, N)';
U = lsode("Udot", U0,  theta);
%%%% --------------------------------------------------------%%%%%

% Only the first column of U is needed
u = U(:,1);
r = 1./u;

%% Set up data polt
subplot(1,2,1);
hold on;
plot (theta, u);
plot (theta, r);
% axis limits
p_max = max(max(u(:,1),max(r)));
p_min = min(min(u(:,1),min(r)));
margin = 0.5;
% finishing touches
axis([min(theta), max(theta), p_min - margin, p_max + margin] );
title("r and u");
xlabel("theta")
legend("u", "r");
grid("on");
hold off;

%% set up trejectory plot
subplot(1,2,2);
polar(theta, r, "--b");
% axis limits
lim    = max(r) + margin;
no     = 2; % number of grid spacings
tticks = 0:30:(360-30);
set (gca, "rtick", lim/no:lim/no:lim, "ttick", tticks);
axis([-lim, lim, -lim, lim], "equal");
title("Trajectory");
grid("on");
