function H = HjacDepth(x, map, sensor_pos, K)
% HjacDepth: output the jacobian of the depth measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       map          of environment, n x [x1 y1 x2 y2] walls
%       sensor_pos   sensor position in the body frame [1x2]
%       K            number of measurements (rays) the sensor gets between 27 to -27 
%
%   OUTPUTS
%       H            Kx3 jacobian matrix
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Sasenarine, Etienne
    
% initalize perturbation value, Jacobian matrix and angles
epsilon = 1e-5;
H = zeros(K, 3);
angles = linspace(27 * pi/180, -27 * pi/180, K)';

% loop through x,y,theta
for j = 1:3
    % perturb positive
    x_pos = x;
    x_pos(j) = x_pos(j) + epsilon;

    % perturb negative
    x_neg = x;
    x_neg(j) = x_neg(j) - epsilon;

    % compute depth measurements for positive and negative perturbations
    z_pos = depthPredict(x_pos, map, sensor_pos, angles);
    z_neg = depthPredict(x_neg, map, sensor_pos, angles);
    
    % calculate the Jacobian for the current perturbation
    H(:, j) = (z_pos - z_neg) / (2 * epsilon);
end

end