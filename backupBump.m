function[dataStore] = backupBump(Robot)

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


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.

%Drive straight with a velocity of .4 m/s, when bump sensor is triggered 
%back up .25m and turn 30 degrees clockwise. 
maxT = 15;
tic
while(toc<maxT)
    [noRobotCount, dataStore] = readStoreSensorData(Robot,0,dataStore);
    frontBump = dataStore.bump(end,7);
    rightBump = dataStore.bump(end,2);
    leftBump = dataStore.bump(end,3);

    if frontBump ~= 0
        [cmdV, cmdW] = limitCmds(-.25,pi/6,.49,.13);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(1)
    elseif rightBump ~= 0
        [cmdV, cmdW] = limitCmds(-.25,pi/6,.49,.13);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(1)
    elseif leftBump ~= 0
        [cmdV, cmdW] = limitCmds(-.25,pi/6,.49,.13);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(1)
    else 
        [cmdV, cmdW] = limitCmds(.4,0,.49,.13);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
end