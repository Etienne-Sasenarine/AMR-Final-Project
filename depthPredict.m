function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 1
%   Sasenarine, Etienne


x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
sensorWorld = [x; y] + R * sensorOrigin(:);

K = length(angles);
maxRange = 100;
depth = zeros(K,1);

for k = 1:K
    angle = theta + angles(k);
    rayEnd = [sensorWorld(1) + maxRange * cos(angle);
              sensorWorld(2) + maxRange * sin(angle)];

    for i = 1:size(map,1)
        [isect, xi, yi] = intersectPoint(sensorWorld(1), sensorWorld(2), ...
            rayEnd(1), rayEnd(2), map(i,1), map(i,2), map(i,3), map(i,4)); 
        if isect
            distance = norm([xi; yi] - sensorWorld);
            projected_depth = distance * cos(angles(k));
            if projected_depth < depth(k) || depth(k) == 0
                depth(k) = projected_depth;
            end
        end
    end
end

depth = depth(:);

end