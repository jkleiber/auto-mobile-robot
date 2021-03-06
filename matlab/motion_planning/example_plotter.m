close all

% Gather results
x = dc_X(1:N);
y = dc_X(N+1:2*N);
theta = dc_X(2*N+1:3*N);
v = dc_X(3*N+1:4*N);
w = dc_X(4*N+1:5*N);

%%%%%%%%%%%%%%%%%
% Plot results
%%%%%%%%%%%%%%%%%%
% 2D Trajectory
figure(1)
plot(x, y)
xlabel("X")
ylabel("Y")
title("Robot Trajectory")

% States vs Time
figure(2)
subplot(3,1,1)
plot(t, x)
yline(xf(1), 'r')
xlabel("Time (sec)")
ylabel("X")
title("X vs Time")

subplot(3,1,2)
plot(t, y)
yline(xf(2), 'r')
xlabel("Time (sec)")
ylabel("Y")
title("Y vs Time")

subplot(3,1,3)
plot(t, theta)
yline(xf(3), 'r')
xlabel("Time (sec)")
ylabel("Heading (rad)")
title("Heading vs Time")

% Control vs Time
figure(3)
subplot(2,1,1)
plot(t, v)
xlabel("Time (sec)")
ylabel("Forward Velocity (m/s)")
title("Forward Velocity vs Time")

subplot(2,1,2)
plot(t, w)
xlabel("Time (sec)")
ylabel("Angular Velocity (rad/s)")
title("Angular Velocity vs Time")