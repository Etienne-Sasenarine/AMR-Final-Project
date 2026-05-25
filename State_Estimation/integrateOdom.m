function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   Sasenarine, Etienne

% initial pose
x = initPose(1);
y = initPose(2);
theta = initPose(3);

N = length(d);
finalPose = zeros(3, N);

for i = 1:N
    if phi(i) == 0
        % zero phi - straight line motion
        x = x + d(i) * cos(theta);
        y = y + d(i) * sin(theta);
    else
        % nonzero phi
        x = x + (d(i)/phi(i)) * (sin(theta + phi(i)) - sin(theta));
        y = y - (d(i)/phi(i)) * (cos(theta + phi(i)) - cos(theta));
    end
    % update heading and store pose
    theta = theta + phi(i);
    finalPose(:, i) = [x; y; theta];
end
end