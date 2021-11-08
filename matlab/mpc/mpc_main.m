
% Reference trajectory
x_ref = 10;
y_ref = 10;
theta_ref = deg2rad(45);
ref_traj = [x_ref; y_ref; theta_ref];

% MPC control constraints
constraints = [0 1; -2 2];

% System
sys = @differential_drive_discrete;

% Timing
dt = 0.1;
T = 20;
t = linspace(0,T,T/dt);
N = length(t);

% Actual trajectory
x_traj = zeros(3,N);
u_traj = zeros(2,N);

% Simulation
for k = 1:N
    % Get the current state
    xk = x_traj(:,k);
    
    % MPC
    u = mpc_synthesis(xk, ref_traj, sys, constraints, dt);
    
    % Dynamical system update
    x_traj(:,k+1) = xk + sys(xk, u, dt);
    u_traj(:,k+1) = u;
end

plot_mpc_traj