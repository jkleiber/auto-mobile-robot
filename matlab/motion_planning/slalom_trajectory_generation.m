close all
    
% Generate waypoints list
num_poles = 3;
dist_apart = 1;
num_waypoints = num_poles*4;
waypoints = zeros(3, num_waypoints);
obstacles = 1:2:num_poles*2;

x = 0;
list_idx = 1;
y_opts = [-1 0 1 0];
theta_opts1 = [0 pi/4 0 -pi/4]; % Forward
theta_opts2 = [pi 3*pi/4 pi 5*pi/4]; % Backward odd
theta_opts3 = [-pi -5*pi/4 -pi -3*pi/4]; % Backward even
for i = 1:(num_waypoints-1)  
    % Choose y from list
    y = y_opts(list_idx);
    
    % Choose theta and direction based on list and waypoint
    % Odd number of poles
    if mod(num_poles, 2) == 1
        if i > (num_waypoints/2)
           theta = theta_opts2(list_idx);
        elseif i < num_waypoints/2
           theta = theta_opts1(list_idx);
        else
           theta = pi/2;
        end
    % Even number of poles
    else
        if i > (num_waypoints/2)
           theta = theta_opts3(list_idx);
        elseif i < num_waypoints/2
           theta = theta_opts1(list_idx);
        else
           theta = -pi/2;
        end
    end
    
    % Choose x from list
    if i <= (num_waypoints / 2)
        x = x + 1;
    else 
        x = x - 1;
    end
    
    % update list index
    list_idx = list_idx + 1;
    if list_idx > 4
        list_idx = 1;
    end
    
    % Add waypoint to list
    waypoints(:,i) = [x; y; theta];
end

% Endpoint
if mod(num_poles,2) == 1
    waypoints(:,end) = [0; 0; pi];
else
    waypoints(:,end) = [0; 0; -pi];
end


% return

% Get ready to save trajectories
time_traj = [];
x_traj = [];
y_traj = [];
theta_traj = [];
v_traj = [];
w_traj = [];

% Timing constraints
t0 = 0;
traj_dt = 8;
dt = 0.5;

% Control constraints
% [v; w]
u_lower = [0; -2];
u_upper = [0.5; 2];

% Optimization Matrices
Qf = 10000*eye(3);
Q = 400*eye(3);
R = 800*eye(2);

x0 = [0; 0; 0];
for i = 1:num_waypoints
    % Get current waypoint
    xf = waypoints(:,i);
    
    % Find the best trajectory
    tf = t0 + traj_dt;
    [time, x, y, t, v, w] = direct_collocation(x0, xf, t0, tf, dt, Qf, Q, R, u_lower, u_upper);
    
    % Save trajectory to list
    time_traj = [time_traj; time];
    x_traj = [x_traj; x];
    y_traj = [y_traj; y];
    theta_traj = [theta_traj; t];
    v_traj = [v_traj; v];
    w_traj = [w_traj; w];
    
    % update x0 to the current waypoint
    x0 = [x(end); y(end); t(end); v(end); w(end)];
    
    % update time
    t0 = tf;
end

% Plot the optimized trajectory
% organize for quiver
X = waypoints(1,:);
Y = waypoints(2,:);
U = cos(waypoints(3,:));
V = sin(waypoints(3,:));

% 2D Trajectory
figure(1)
quiver(X,Y,U,V, 0.25)
hold on
plot(x_traj, y_traj)
scatter(obstacles, zeros(num_poles,1),'r', 'filled')
hold off
xlabel("X")
ylabel("Y")
title("Robot Trajectory")

% Save to CSV
TRAJ = double([time_traj x_traj y_traj theta_traj v_traj w_traj]);
writematrix(round(TRAJ,6), 'trajectory.csv', 'Delimiter', 'comma');

