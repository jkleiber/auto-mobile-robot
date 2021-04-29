
function [time, x, y, theta, v, w] = direct_collocation(x0, xf, t0, tf, dt, Qf, Q, R, u_lower, u_upper)

% Timing discretization
N = floor((tf - t0) / dt);
time = linspace(t0, tf, N)';

% Find input variables
% [X, Y, Theta, V, W]
X = [linspace(x0(1), xf(1), N)'; linspace(x0(2), xf(2), N)'; linspace(x0(3), xf(3), N)'; ... 
     zeros(N,1); zeros(N,1)];

% Create lower and upper bound matrices
lb = [-Inf*ones(N,1); -Inf*ones(N,1); -Inf*ones(N,1); u_lower(1)*ones(N,1); u_lower(2)*ones(N,1)];
ub = [Inf*ones(N,1); Inf*ones(N,1); Inf*ones(N,1); u_upper(1)*ones(N,1); u_upper(2)*ones(N,1)];

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
[dc_X, fval, exitFlag, output] = fmincon(@(x) obj_function(x, dt, N, Qf, Q, R, xf),X,A,b,Aeq,beq,lb,ub,@(x) robot_constraints(x, dt, N, xf),options);

if exitFlag ~= 1
    error(output)
end

% Gather results
x = dc_X(1:N);
y = dc_X(N+1:2*N);
theta = dc_X(2*N+1:3*N);
v = dc_X(3*N+1:4*N);
w = dc_X(4*N+1:5*N);

end


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
        
        % Wrap theta correctly
%         xk(3) = wrapAngle(xk(3));
%         xk1(3) = wrapAngle(xk1(3));
        
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
        uk1 = ctrl(i,:)';
        
        Ji = ( (xk-xf)'*Q*(xk-xf) + (xk1-xf)'*Q*(xk1-xf) + uk'*R*uk + uk1'*R*uk1 )*(dt/2);
        J = J + Ji;
    end
    
    % Terminal cost
    x_f = state(end);
    
    Jf = 0.5*(x_f - xf)'*Qf*(x_f - xf);
    J = J + Jf;
 end

function angle = wrapAngle(angle)
    % Wrap to +/- 2pi
    angle = mod(angle, 2*pi);
    
    % Wrap to +/- pi
    if angle > pi
        angle = 2*pi - angle;
    elseif angle < -pi
        angle = -pi - angle;
    end
end

%% Useful resources
% Helpful intro guide: https://mec560sbu.github.io/2016/09/30/direct_collocation/
% Survey of Trajectory Optimization techniques: https://epubs.siam.org/doi/pdf/10.1137/16M1062569

% See Zotero for more papers I saved


