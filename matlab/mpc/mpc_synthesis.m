function u = mpc_synthesis(x0, x_ref, sys, constraints, dt)
    % Set the lookahead
    lookahead = 15;

    % Set the constraints
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [constraints(1,1)*ones(lookahead, 1); constraints(2,1)*ones(lookahead, 1)];
    ub = [constraints(1,2)*ones(lookahead, 1); constraints(2,2)*ones(lookahead, 1)];
    
    % Initial control 
    u0 = zeros(2*lookahead,1);
    
    % Get the control from the convex optimization problem
    U = fmincon(@(x) obj_func(x, x0, x_ref, sys, lookahead, dt), u0, A, b, Aeq, beq, lb, ub); 

    % Get the first element from the optimal control vector U
    u = [U(1); U(lookahead+1)];
end

function J = obj_func(u, x0, x_ref, sys, lookahead, dt)
    % Objective function

    % Compute the trajectory given the control vector u
    x_traj = ones(3,lookahead+1).*x0;
    ref_traj = ones(3,lookahead).*x_ref;
    for i = 1:lookahead
        % System update
        xi = x_traj(:,i);
        ui = [u(i); u(lookahead + i)];
        x_traj(:,i+1) = xi + sys(xi, ui, dt);
    end
    
    % Cost function
    weights = [100, 100, 10];
    J = 0;
    for i = 1:length(weights)
        J = J + weights(i) * sum((ref_traj(i,:) - x_traj(i,2:end)).^2);
    end
end