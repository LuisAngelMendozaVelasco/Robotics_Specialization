% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
    % function [segI, loc] = detectBall(I)
    %
    % INPUT
    % I       120x160x3 numerial array 
    %
    % OUTPUT
    % segI    120x160 numeric array
    % loc     1x2 or 2x1 numeric array 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Hard code your learned model parameters here
    %
    % mu = 
    % sig = 
    % thre = 
    par = load("parameters.mat");
    mu = par.mu;
    mu = reshape(mu, 3, 1);
    sig = par.S;
    thre = 1e-6;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find ball-color pixels using your model
    invS = inv(sig);
    I = double(I);
    
    for i = 1:size(I, 1)
        for j = 1:size(I, 2)
            x = I(i, j, :);
            x = reshape(x, 3, 1);
            P(i, j) = 1 / (2*pi)^(3/2) / sqrt(det(sig)) * exp(-1/2*(x-mu)' * invS * (x-mu));
        end
    end
    
    B = P>thre;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Do more processing to segment out the right cluster of pixels.
    % You may use the following functions.
    %   bwconncomp
    %   regionprops
    % Please see example_bw.m if you need an example code.
    CC = bwconncomp(B);
    numPixels = cellfun(@numel, CC.PixelIdxList);
    [biggest, idx] = max(numPixels);
    bw_biggest(CC.PixelIdxList{idx}) = true; 
    
    % show the centroid
    % http://www.mathworks.com/help/images/ref/regionprops.html
    S = regionprops(CC, 'Centroid');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute the location of the ball center
    
    % segI = 
    % loc = 
    segI = B; 
    loc = S(idx).Centroid;
    plot(loc(1), loc(2), 'r+');
    
    % Note: In this assigment, the center of the segmented ball area will be considered for grading. 
    % (You don't need to consider the whole ball shape if the ball is occluded.)
end