function [ u ] = pd_controller(~, s, s_des, params)
    %PD_CONTROLLER  PD controller for the height
    %
    %   s: 2x1 vector containing the current state [z; v_z]
    %   s_des: 2x1 vector containing desired state [z; v_z]
    %   params: robot parameters
    
    % u = 0;
    
    % FILL IN YOUR CODE HERE
    m = params.mass;
    g = params.gravity;
    kp = 750;
    kv = 100;
    zdotdot_des = 0;
    z = s(1);
    z_des = s_des(1);
    zdot = s(2);
    zdot_des = s_des(2);
    e = z_des - z;
    edot = zdot_des - zdot;

    u = m * (zdotdot_des + kp * e + kv * edot + g);

    if u > params.u_max 
        u = params.u_max;
    elseif u < params.u_min
        u = params.u_min;
    end
end