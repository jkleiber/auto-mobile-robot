close all;

%% Plot trajectories

% Timing
t0 = 0;
traj_tf = num_waypoints * tf;
traj_t = linspace(t0, traj_tf, length(v_traj));
obstacles = 1:2:num_poles*2;

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


% Control
figure(2)
subplot(2,1,1)
plot(traj_t, v_traj)
title("V")

subplot(2,1,2)
plot(traj_t, w_traj)
title("w")


% States
figure(3)
subplot(3,1,1)
plot(traj_t, x_traj)
title("x")

subplot(3,1,2)
plot(traj_t, y_traj)
title("y")

subplot(3,1,3)
plot(traj_t, theta_traj)
title("$\theta$", "Interpreter", "latex")
