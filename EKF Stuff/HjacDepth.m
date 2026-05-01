function H =  HjacDepth(x, map, sensor_pos, K)
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


n = size(x,1);%number of states
del = .1;
angles = linspace(deg2rad(27),deg2rad(-27),K);
H = zeros(K,3);
%go state by state to only linearize about that state var first 
for i = 1:n
    state = zeros(n,1);
    state(i) = 1;
    x1 = x + del*state;
    x2 = x - del*state;
    H1 = depthPredict(x1,map,sensor_pos,angles);
    H2 = depthPredict(x2,map,sensor_pos,angles);
    H(:,i) = (H1 - H2)/ (2*del);
end


end

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
%   Homework 2
depth = zeros(size(angles));
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);
R_ib = [cos(theta) -sin(theta); sin(theta) cos(theta)];  
sensorOriginWorld =  R_ib * sensorOrigin.' + [x;y];

lineLen = 50;

for i = 1:length(angles)

    sensorLineX = sensorOriginWorld(1)+lineLen*cos(theta+angles(i));
    sensorLineY = sensorOriginWorld(2)+lineLen*sin(theta+angles(i));
    minDist = 1000;
    for k = 1:size(map,1)
        [isec, xInt, yInt] = intersectPoint(map(k,1),map(k,2),map(k,3), ...
            map(k,4), sensorOriginWorld(1),sensorOriginWorld(2), ...
            sensorLineX, sensorLineY);
        if isec
           r = sqrt((xInt-sensorOriginWorld(1))^2 + (yInt-sensorOriginWorld(2))^2);
           depthRaw = r*cos(angles(i));
           if depthRaw <= minDist
              minDist = depthRaw;
           end
        end
    end
    depth(i) = minDist;
end
end