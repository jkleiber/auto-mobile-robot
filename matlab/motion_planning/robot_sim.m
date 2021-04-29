close all

%% Simulate hermite-simpson control open loop
x0 = [0; 0; 0];
t0 = 0;
traj_tf = num_waypoints * tf;
ctrl_t = linspace(t0, traj_tf, length(v_traj));
states = [x_traj y_traj theta_traj];

[tout, sim_x] = ode45(@(t,y) robot_simulation(t,y,states,v_traj,w_traj, ctrl_t), [t0 traj_tf], x0);

obstacles = 1:2:num_poles*2;
% 2D Trajectory
figure(1)
plot(sim_x(:,1), sim_x(:,2))
hold on
plot(states(:,1), states(:,2))
scatter(obstacles, zeros(num_poles,1),'r', 'filled')
hold off
legend('simulation', 'plan')
xlabel("x")
ylabel("y")
title("Simulated Trajectory")

% Control
u = zeros(length(tout),2);
for i = 1:length(tout)
    t = tout(i);
    x = sim_x(i,:);
    
    [~, u(i,:)] = robot_simulation(t, x, states, v_traj, w_traj, ctrl_t);
end

figure(2)
plot(tout, u)
title("u")
legend('v', 'w')

function [dxdt, u] = robot_simulation(t, x, states, v, w, ctrl_t)
    % Interpolate direct collocation input
    v_in = interp1(ctrl_t, v, t);
    w_in = interp1(ctrl_t, w, t);
    state = interp1(ctrl_t, states, t)';

    % Control gains
    zeta = 0.4;
    b = 18;
    K = 2*zeta*sqrt(w_in^2 + b*v_in^2);
    k1 = K;
    k2 = b;
    k3 = K;

    % Find error in robot frame
    delta = state - x;
    theta = x(3);
    R = [cos(theta) sin(theta) 0;
        -sin(theta) cos(theta) 0;
             0          0      1];
    E = R*delta;
    ex = E(1);
    ey = E(2);
    et = E(3);
    
    % Nonlinear control (Ramsete Controller)
    U = [-k1 * ex;
         -k2 * v_in * sinc(et) * ey - k3 * et];
    
    % Command to drivetrain after nonlinear transform
    u = [v_in*cos(et) - U(1);
         w_in - U(2)];
     
    u(1) = constrain(u(1), 0, 1);
    u(2) = constrain(u(2), -1, 1);
     
    % Find derivatives of nonlinear system
    dxdt = differential_drive(x, u);
end

function c = constrain(c, cmin, cmax)
    c = min(cmax, max(c, cmin));
end