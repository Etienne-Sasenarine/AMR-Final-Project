function [dataStore] = visitWaypoints(Robot, waypoints, closeEnough, epsilon,gotopt)
if nargin < 2
    %waypoints = [-3,0; 0 -3; 3 0; 0 3];
    waypoints = [-1 0; 1 0];
    closeEnough = .1;
    epsilon = .2;
    gotopt = 1;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);

while(gotopt <= size(waypoints,1))
    %get current pos
    [noRobotCount, dataStore] = readStoreSensorData(Robot, 0,dataStore);
    posx = dataStore.truthPose(end,2);
    posy = dataStore.truthPose(end,3);

    targetX = waypoints(gotopt,1);
    targetY = waypoints(gotopt,2);
    
    diffX = targetX - posx;
    diffY = targetY - posy;
    
    dist = sqrt(diffX^2 + diffY^2);
    
    if dist <= closeEnough
        gotopt = gotopt +1;
    else
        %linearize vx and vy
        [cmdV, cmdW] = feedbackLin(diffX,diffY,dataStore.truthPose(end,4),epsilon);

        [cmdV,cmdW] = limitCmds(cmdV,cmdW,.49,.13);
        
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
  
end

SetFwdVelAngVelCreate(Robot, 0, 0);

end

