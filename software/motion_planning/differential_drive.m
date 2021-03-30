function dx = differential_drive(x,u)
    
    % Constraints
    % Determine through system ID
    v_upper = 6;
    v_lower = -6;
    w_upper = 6;
    w_lower = -6;

    % Velocity
    v = constrain(u(1), v_lower, v_upper);
    w = constrain(u(2), w_lower, w_upper);
    theta = x(3);
    
    % State update
    f_x = v * cos(theta);
    f_y = v * sin(theta);
    f_theta = w;

    dx = [f_x; f_y; f_theta];
end

function c = constrain(c, cmin, cmax)
    c = min(cmax, max(c, cmin));
end