function endeff = computeForwardKinematics(rads)
    % x = 0;
    % y = 0;
    x = cos(rads);
    y = sin(rads);
    
    endeff = [x, y];
end