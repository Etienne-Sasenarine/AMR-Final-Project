function [mu_next_gps, sigma_next_gps, mu_next_depth, sigma_next_depth] = ...
    testEKF(mu, ut, sigma, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays)
% testEKF: Performs one step of the extended Kalman Filter, outputs the belief given previous belief
%
%   INPUTS
%       mu           previous vector of pose state (mu(t-1)
%       ut           current command [d; phi]
%       sigma        previous covariance matrix
%       R            state model noise covariance matrix
%       z_gps        current gps measurement vector
%       Q_GPS        GPS measurement noise covariance matrix
%       z_depth      depth measurement vector
%       Q_depth      Realsense depth measurement noise covariance matrix
%       map          map of the environment
%       sensor_pos   sensor position in the robot frame [x y]
%       n_rs_rays    number of evenly distributed realsense depth rays
%       (27...-27) degrees
%
%   OUTPUTS
%       mu_next_gps      current estimate of vector of pose state (gps)
%       sigma_next_gps   current covariance matrix (gps)
%       mu_next_depth    current estimate of vector of pose state (depth)
%       sigma_next_depth current covariance matrix (depth)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   First, Last Name

    % you can create pointers to functions in this way:   
    % dynamicsJac   = @(x,u) GjacDiffDrive(x, u); %pointer to jacobian of the dynamics
    % you can create a pointer and send extra parameters in this way:
    % measureJac   = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays); 
    % then, inside your filter, you can call it like this:
    % z = measureJac(mu_prev);
    

    % function handles
    g = @(x,u) integrateOdom(x, u(1), u(2));
    dynamicsJac = @(x,u) GjacDiffDrive(x, u);

    h_gps = @(x) hGPS(x);
    H_gps = @(x) HjacGPS(x);

    angles = linspace(deg2rad(27), deg2rad(-27), n_rs_rays).';
    h_depth = @(x) depthPredict(x, map, sensor_pos, angles);
    H_depth = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays);

    % EKF with GPS measurement
    
    % prediction
    mu_bar_gps = g(mu, ut);
    Gt_gps = dynamicsJac(mu, ut);
    sigma_bar_gps = Gt_gps * sigma * Gt_gps' + R;

    % update
    z_hat_gps = h_gps(mu_bar_gps);
    Ht_gps = H_gps(mu_bar_gps);

    K_gps = sigma_bar_gps * Ht_gps' / (Ht_gps * sigma_bar_gps * Ht_gps' + Q_GPS);

    mu_next_gps = mu_bar_gps + K_gps * (z_gps - z_hat_gps);
    sigma_next_gps = (eye(size(sigma_bar_gps)) - K_gps * Ht_gps) * sigma_bar_gps;

    % wrap heading
    mu_next_gps(3) = atan2(sin(mu_next_gps(3)), cos(mu_next_gps(3)));

    % EKF with depth measurement

    % prediction
    mu_bar_depth = g(mu, ut);
    Gt_depth = dynamicsJac(mu, ut);
    sigma_bar_depth = Gt_depth * sigma * Gt_depth' + R;

    % update
    z_hat_depth = h_depth(mu_bar_depth);
    Ht_depth = H_depth(mu_bar_depth);

    K_depth = sigma_bar_depth * Ht_depth' / (Ht_depth * sigma_bar_depth * Ht_depth' + Q_depth);

    mu_next_depth = mu_bar_depth + K_depth * (z_depth - z_hat_depth);
    sigma_next_depth = (eye(size(sigma_bar_depth)) - K_depth * Ht_depth) * sigma_bar_depth;

    % wrap heading
    mu_next_depth(3) = atan2(sin(mu_next_depth(3)), cos(mu_next_depth(3)));

end


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

xPos = initPose(1);
yPos = initPose(2);
theta = initPose(3);
N = length(d);

finalPose = zeros(3,N);

for i=1:N
    thetaMid = theta + 0.5*phi(i);
    
    xPosNext = xPos + d(i)*cos(thetaMid);
    yPosNext = yPos + d(i)*sin(thetaMid);
    thetaNext = theta + phi(i);

    finalPose(1,i) = xPosNext;
    finalPose(2,i) = yPosNext;
    finalPose(3,i) = thetaNext;
    
    xPos = xPosNext;
    yPos = yPosNext;
    theta = thetaNext;
end
end

function G = GjacDiffDrive(x, u)
% GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       u            2-by-1 vector [d, phi]'
%
%   OUTPUTS
%       G            Jacobian matrix partial(g)/partial(x)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Last, First Name
    

state = x;
x = state(1,1);
y = state(2,1);
th = state(3,1);

d = u(1,1);
phi = u(2,1);

%jacobian matrix with turning 
if phi ~= 0
G = [1 0 (d/phi)*(cos(th+phi)-cos(th));
    0 1 (d/phi)*(sin(th+phi)-sin(th));
    0 0 1];
else 
%jacobian matrix without turning 
G = [1 0 -d*sin(th); 0 1 d*cos(th); 0 0 1];
end
end

function Loc = hGPS(x)
% hGPS: predict the GPS measurements for a robot pose.
%
%   Loc = hGPS(pose) returns
%   the expected GPS measurements
%
%   INPUTS
%       x            3-by-1 vector of pose
%
%   OUTPUTS
%       Loc          n-by-m vector of the output of the GPS
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Last, First Name
Loc = x;
end


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

function H = HjacGPS(x)
% HjacGPS: output the jacobian of the GPS measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%
%   OUTPUTS
%       H            jacobian matrix of GPS measurement
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Last, First Name

H = eye(length(x));

end

function[isect,x,y]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4)
% INTERSECTPOINT: find the x/y coordinates of the intersection of two line
% segments (if intersection exists)
% 
%   [ISECT,X,Y] = INTERSECTPOINT(X1,Y1,X2,Y2,X3,Y3,X4,Y4) calculates the
%   intersection point of two line segments
% 
%   INPUTS
%       x1,y1   x/y coordinates of point 1 on line segment 1
%       x2,y2   x/y coordinates of point 2 on line segment 1
%       x3,y3   x/y coordinates of point 1 on line segment 2
%       x4,y4   x/y coordinates of point 2 on line segment 2
% 
%   OUTPUTS
%       isect   bool variable (true if segments intersect, false otherwise)
%       x,y     x/y coordinates of intersection point (if isect = true)
%
%   TEMP
%       ua      distance of intersection point along line 1 (0 <= ua <= 1) 
%               where: x = x1 + ua (x2 - x1) 
% 
%   Refer to http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d for
%   line segment intersection equations.
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots

x = [];
y = [];
ua = [];

denom = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);

if denom == 0
    % if denom = 0, lines are parallel
    isect = false;
    return;
else
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3))/denom;
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/denom;
    
    % if (0 <= ua <= 1) and (0 <= ub <= 1), intersection point lies on line
    % segments
    if ua >= 0 && ub >= 0 && ua <= 1 && ub <= 1
        isect = true;
        x = x1 + ua*(x2-x1);
        y = y1 + ua*(y2-y1);
    else
        % else, the intersection point lies where the infinite lines
        % intersect, but is not on the line segments
        isect = false;
        return;
    end
end
end