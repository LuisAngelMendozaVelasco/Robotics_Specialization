function [ u1, u2 ] = controller(~, state, des_state, params)
    %CONTROLLER  Controller for the planar quadrotor
    %
    %   state: The current state of the robot with the following fields:
    %   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
    %   state.omega = [phi_dot]
    %
    %   des_state: The desired states are:
    %   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
    %   [y_ddot; z_ddot]
    %
    %   params: robot parameters
    
    %   Using these current and desired states, you have to compute the desired
    %   controls
    
    % u1 = 0;
    % u2 = 0;
    
    % FILL IN YOUR CODE HERE
    m = params.mass;
    g = params.gravity;
    zdotdot_T = des_state.acc(2);
    I_xx = params.Ixx;
    phidotdot_T = 0;
    ydotdot_T = des_state.acc(1);

    zdot_T = des_state.vel(2);
    zdot = state.vel(2);
    edot_z = zdot_T - zdot;

    z_T = des_state.pos(2);
    z = state.pos(2);
    e_z = z_T - z;

    phidot_T = 0;
    phidot = state.omega;
    edot_phi = phidot_T - phidot;

    ydot_T = des_state.vel(1);
    ydot = state.vel(1);
    edot_y = ydot_T - ydot;

    y_T = des_state.pos(1);
    y = state.pos(1);
    e_y = y_T - y;
    
    k_pz = 80;
    k_vz = 12;
    
    k_py = 20;
    k_vy = 5;
    
    k_pphi = 1000;
    k_vphi = 30;

    phi_c = -1 / g * (ydotdot_T + k_vy * edot_y + k_py * e_y);
    phi = state.rot;
    e_phi = phi_c - phi;
    
    u1 = m * (g + zdotdot_T + k_vz * edot_z + k_pz * e_z);   
    u2 = I_xx * (phidotdot_T + k_vphi * edot_phi + k_pphi * e_phi);
    
    if u1 > params.maxF
        u1 = params.maxF;
    elseif u1 < params.minF
        u1 = params.minF;
    end
end