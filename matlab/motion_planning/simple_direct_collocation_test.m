close all

% Starting and Final States
x0 = [0; 0; 0];
xf = [1; -1; 0];
vf = 1;

% Timing
dt = 0.1;
t0 = 0;
tf = 2;
N = floor((tf - t0) / dt);
t = linspace(t0, tf, N);
u0 = 1;%sin(t)';

% Find input variables
% [X, Y, Theta, V, W]
% del = xf - x0;  % vector from start to finish
% start_angle = atan2(del(1),del(2));
% end_angle = xf(3);
% angle_sweep = linspace(start_angle, end_angle, N);

% Right turn
if xf < x0
    
% Left turn
elseif xf > x0

end   
 
X = [linspace(x0(1), xf(1), N)'; linspace(x0(2), xf(2), N)'; linspace(x0(3), xf(3), N)'; ... 
     u0.*ones(N,1); u0.*ones(N,1)];

% Optimization Matrices
Qf = 10000*eye(3);
Q = 100*eye(3);
R = 70*eye(2);

% Create lower and upper bound matrices
lb = [-Inf*ones(N,1); -Inf*ones(N,1); -Inf*ones(N,1); 0*ones(N,1); -6*ones(N,1)];
ub = [Inf*ones(N,1); Inf*ones(N,1); Inf*ones(N,1); 6*ones(N,1); 6*ones(N,1)];

% Bound the start state to exactly the start x
lb(1) = x0(1);
lb(N+1) = x0(2);
lb(2*N+1) = x0(3);
ub(1) = x0(1);
ub(N+1) = x0(2);
ub(2*N+1) = x0(3);

% Bound the final state by a small value epsilon
eps = 1e-2;
lb(N) = xf(1)-eps;
lb(2*N) = xf(2)-eps;
lb(3*N) = xf(3)-eps;
ub(N) = xf(1)+eps;
ub(2*N) = xf(2)+eps;
ub(3*N) = xf(3)+eps;

% Lock the final control to the desired velocity
% ub(4*N) = vf;
% lb(4*N) = vf;

A = [];        
b = [];
Aeq = [];
beq = [];
options = optimoptions(@fmincon,'OptimalityTolerance',1e-6, ... 
                        'MaxFunctionEvaluations', 1e8, ... 
                        'MaxIterations', 1e3, ...
                        'Display', 'iter'); % set tolerance
                    
%                         'Algorithm', 'sqp', ...
% , dt, N, Qf, Q, R, xf
dc_X = fmincon(@(x) obj_function(x, dt, N, Qf, Q, R, xf),X,A,b,Aeq,beq,lb,ub,@(x) robot_constraints(x, dt, N, xf),options);

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

function [c, ceq] = robot_constraints(X,dt,N,xf)
    c = [];

    % Parse X
    x = X(1:N);
    y = X(N+1:N*2);
    theta = X(2*N + 1:3*N);
    v = X(3*N+1:4*N);
    w = X(4*N+1:5*N);
    
    % Collect states and control
    state = [x y theta];
    ctrl = [v w];
    
    % Dynamics
    f = @differential_drive;
    
%     ceq = [0; 0; 0];
    ceq = [];
    for i = 1:(N-1)
        % Get state and control
        xk = state(i,:)';
        xk1 = state(i+1,:)';
        uk = ctrl(i,:)';
        uk1 = ctrl(i+1,:)';
        
        % Determine collocation point
        
        xc = 0.5*(xk + xk1) + (dt/8)*(f(xk, uk) + f(xk1, uk1));
        uc = 0.5*(uk + uk1);
        
%         dx = xk - xk1;
%         hx = (dt/6)*(f(xk, uk) + f(xk1, uk1) + 4*f(xc, uc));
        xc_dot = f(xc, uc);
        x_dot = -(3/(2*dt))*(xk - xk1) - 0.25*(f(xk, uk) + f(xk1, uk1));
        
        ceq = [ceq; x_dot - xc_dot];
    end
    
%     ceq = [ceq; zeros(3,1)];
end

 function J = obj_function(X, dt, N, Qf, Q, R, xf)
    % Cost function
    J = 0;
    
    % Parse X
    x = X(1:N);
    y = X(N+1:N*2);
    theta = X(2*N + 1:3*N);
    v = X(3*N+1:4*N);
    w = X(4*N+1:5*N);
    
    % Collect states and control
    state = [x y theta];
    ctrl = [v w];
    
    for i = 1:(N-1)
        xk = state(i,:)';
        xk1 = state(i+1,:)';
        uk = ctrl(i,:)';
        uk1 = ctrl(i+1,:)';
        
        Ji = ( (xk-xf)'*Q*(xk-xf) + (xk1-xf)'*Q*(xk1-xf) + uk'*R*uk + uk1'*R*uk1 )*dt/2;
        J = J + Ji;
    end
    
    % Terminal cost
    x_f = state(end);
    
    Jf = 0.5*(x_f - xf)'*Qf*(x_f - xf);
    J = J + Jf;
end


%% Useful resources
% Helpful intro guide: https://mec560sbu.github.io/2016/09/30/direct_collocation/
% Survey of Trajectory Optimization techniques: https://epubs.siam.org/doi/pdf/10.1137/16M1062569

% See Zotero for more papers I saved


