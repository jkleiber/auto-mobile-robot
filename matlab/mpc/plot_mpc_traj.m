close all

states = ["x", "y", "theta"];

% Plot the states in subplots
figure
for i = 1:length(states)
    subplot(length(states), 1, i)
    plot(t, x_traj(i,2:end))
    hold on
    yline(ref_traj(i), 'r-')
    title(states(i))
end
sgtitle("X")
legend(states)

% Plot the control trajectory in subplots
figure
subplot(2,1,1)
plot(t, u_traj(1,2:end))
title("v")

subplot(2,1,2)
plot(t, u_traj(2,2:end))
title("w")

sgtitle("U")


% Plot the robot's x,y path
figure
plot(x_traj(1,:), x_traj(2,:))
title("Path")