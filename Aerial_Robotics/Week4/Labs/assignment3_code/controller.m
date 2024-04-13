function [F, M] = controller(t, state, des_state, params)
    %CONTROLLER  Controller for the quadrotor
    %
    %   state: The current state of the robot with the following fields:
    %   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
    %   state.rot = [phi; theta; psi], state.omega = [p; q; r]
    %
    %   des_state: The desired states are:
    %   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
    %   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
    %   des_state.yawdot
    %
    %   params: robot parameters
    
    %   Using these current and desired states, you have to compute the desired
    %   controls
    
    
    % =================== Your code goes here ===================
    
    % % Thrust
    % F = 0;
    % 
    % % Moment
    % M = zeros(3,1);
    
    % =================== Your code ends here ===================
    
    m = params.mass;
    g = params.gravity;

    zdotdot_des = des_state.acc(3);

    zdot_des = des_state.vel(3);
    zdot = state.vel(3);
    e_dz = zdot_des - zdot;

    z_des = des_state.pos(3);
    z = state.pos(3);
    e_pz = z_des - z;

    k_dz = 750;
    k_pz = 450;

    % Thrust
    F = m * (g + zdotdot_des + k_dz * e_dz + k_pz * e_pz);

    % ===================

    psi_des = des_state.yaw;
    ydotdot_des = des_state.acc(2);
    xdotdot_des = des_state.acc(1);

    ydot_des = des_state.vel(2);
    ydot = state.vel(2);
    e_dy = ydot_des - ydot;

    y_des = des_state.pos(2);
    y = state.pos(2);
    e_py = y_des - y;

    xdot_des = des_state.vel(1);
    xdot = state.vel(1);
    e_dx = xdot_des - xdot;

    x_des = des_state.pos(1);
    x = state.pos(1);
    e_px = x_des - x;

    k_px = 100;
    k_dx = 5;
    
    k_py = 100;
    k_dy = 5;

    rdotdot_1des = xdotdot_des + k_dx * e_dx + k_px * e_px;
    rdotdot_2des = ydotdot_des + k_dy * e_dy + k_py * e_py;

    phi_des = 1 / g * (rdotdot_1des * sin(psi_des) - rdotdot_2des * cos(psi_des));
    theta_des = 1 / g * (rdotdot_1des * cos(psi_des) + rdotdot_2des * sin(psi_des));

    % ===================

    phi = state.rot(1);
    e_pphi = phi_des - phi;

    p_des = 0;
    p = state.omega(1);
    e_dp = p_des - p;

    theta = state.rot(2);
    e_ptheta = theta_des - theta;

    q_des = 0;
    q = state.omega(2);
    e_dq = q_des - q;

    psi = state.rot(3);
    e_ppsi = psi_des - psi;

    r_des = des_state.yawdot;
    r = state.omega(3);
    e_dr = r_des - r; 

    k_ppsi = 29; 
    k_dpsi = 0.1;
    
    k_pphi = 30;
    k_dphi = 0.1;
    
    k_ptheta = 14;
    k_dtheta = 0.02;

    % Moment
    M = zeros(3,1);
    M(1, 1) = k_pphi * e_pphi + k_dphi * e_dp;
    M(2, 1) = k_ptheta * e_ptheta + k_dtheta * e_dq;
    M(3, 1) = k_ppsi * e_ppsi + k_dpsi * e_dr;
end