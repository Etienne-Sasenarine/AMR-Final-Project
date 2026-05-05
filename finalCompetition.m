function [dataStore] = finalCompetition(Robot, maxTime, offset_x, offset_y)
% main function for AMR final competition

if nargin < 2
    maxTime = 420;
end
if nargin < 3
    offset_x = 0.13;
end
if nargin < 4
    offset_y = 0.0;
end

try 
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

SetFwdVelAngVelCreate(Robot, 0,0);

% load map data
compMap = load("compMap.mat");
%compMap = load("PracticeMap2026.mat");
beaconLoc = compMap.beaconLoc;
ECwaypoints = compMap.ECwaypoints;
map = compMap.map;
optWalls = compMap.optWalls;
waypoints = compMap.waypoints;

% create dataStore
global dataStore
dataStore = struct('robotPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'waypoints', [], ...
                   'optWallStates', zeros(size(optWalls, 1), 1));

% define depth sensor
dummy_depth = RealSenseDist(Robot);
num_depth_sensors = length(dummy_depth) - 1;
sensorOrigin = [offset_x, offset_y];
angles = linspace(27*(pi/180), -27*(pi/180), num_depth_sensors)';

% define noise and particle filter functions
R = diag([0.005, 0.005, 0.01]);
Q = 0.05 * eye(num_depth_sensors);
predict_func = @integrateOdom;
update_func = @(pose) depthPredict(pose, map, sensorOrigin, angles);

% run initial localization
disp('Starting initial localization...');
[initial_pose, dataStore] = initLocalize(Robot, dataStore, waypoints, beaconLoc, map, R, Q, predict_func, update_func);
dataStore.robotPose = [0, initial_pose']; 
disp(['Localized! Starting Pose: ', num2str(initial_pose')]);

% find starting waypoint
start_xy = initial_pose(1:2)';
dists = vecnorm(waypoints' - start_xy');
[~, startIdx] = min(dists);
start_pose = waypoints(startIdx, :);

% define map using optWalls
all_walls = [map; optWalls]; 
boundaries = [-3.048, 3.048, -2.286, 2.286];

% define waypoints
unvisitedWaypoints = waypoints;
unvisitedWaypoints(startIdx, :) = [];
target_waypoints = [start_pose; unvisitedWaypoints];
num_targets = size(target_waypoints, 1);
unvisitedECWaypoints = ECwaypoints;

% PRM parameters
numSamples = 2000; 
robot_radius = 0.2; 

% build PRM and solve shortest hamiltonian path
disp('Building PRM and solving TSP...');
roadmap = buildPRM(all_walls, boundaries, numSamples, robot_radius, target_waypoints);
target_indices = 1:num_targets;
[cost_matrix, parent_matrix] = buildCostMatrix(roadmap, target_indices);
optimal_order = shortestHamiltonianPath(cost_matrix);

low_level_node_path = extractPath(optimal_order, target_indices, parent_matrix);
physical_path_coords = roadmap.nodes(low_level_node_path, :);

% define control parameters
maxV = 0.1;
wheel2center = 0.13;     
epsilon = 0.2;           
prmIdx = 1;

% initialize particle filter
M = 50; 
particles = zeros(3, M);
particles(1, :) = initial_pose(1) + 0.1 * randn(1, M);
particles(2, :) = initial_pose(2) + 0.1 * randn(1, M);
particles(3, :) = initial_pose(3) + 0.05 * randn(1, M); 

noRobotCount = 0;

tic;
while toc < maxTime && ~isempty(unvisitedWaypoints)
    current_time = toc;
    
    % read sensors
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    
    % extract control input
    if size(dataStore.odometry, 1) >= 1
        u = [dataStore.odometry(end, 2); dataStore.odometry(end, 3)];
    else
        u = [0; 0];
    end
    
    % extract measurements and filter NaNs
    if size(dataStore.rsdepth, 1) >= 1
        z_raw = dataStore.rsdepth(end, 3:end)'; 
        num_rays_total = length(z_raw);
        angles_raw = linspace(27 * pi/180, -27 * pi/180, num_rays_total)';
        
        valid_idx = ~isnan(z_raw) & ~isinf(z_raw) & (z_raw > 0.1) & (z_raw < 5.0);
        
        z = z_raw(valid_idx);
        angles_valid = angles_raw(valid_idx);
        num_valid = length(z);
        
        if num_valid > 0
            Q_pf = eye(num_valid) * 0.1; 
            update_fn = @(pose) depthPredict(pose, all_walls, sensorOrigin, angles_valid);
        else
            z = []; Q_pf = []; update_fn = @(pose) [];
        end
    else
        z = []; Q_pf = []; update_fn = @(pose) [];
    end
    
    % run particle filter
    [particles, ~] = PF(particles, u, z, R, Q_pf, predict_func, update_fn);
    
    x = mean(particles(1,:));
    y = mean(particles(2,:));
    theta = atan2(mean(sin(particles(3,:))), mean(cos(particles(3,:))));
    dataStore.robotPose = [dataStore.robotPose; [current_time, x, y, theta]];
    
    % navigate using visitWaypoints
    [cmdV, cmdW, prmIdx, unvisitedWaypointsNew] = visitWaypoints(x, y, theta, physical_path_coords, prmIdx, unvisitedWaypoints, epsilon);
        
    % check regular waypoints
    if size(unvisitedWaypointsNew, 1) < size(unvisitedWaypoints, 1)
        SetFwdVelAngVelCreate(Robot, 0, 0);
        try beepCreate(Robot); catch; beep; end

        hit_wp = setdiff(unvisitedWaypoints, unvisitedWaypointsNew, 'rows');
        dataStore.waypoints = [dataStore.waypoints; hit_wp];

        pause(0.5); 
    end
    unvisitedWaypoints = unvisitedWaypointsNew;
    
    % check ec waypoints
    for i = 1:size(unvisitedECWaypoints, 1)
        if norm([x, y] - unvisitedECWaypoints(i, :)) <= 0.2 
            SetFwdVelAngVelCreate(Robot, 0, 0);
            try beepCreate(Robot); catch; beep; end

            dataStore.waypoints = [dataStore.waypoints; unvisitedECWaypoints(i, :)];
            unvisitedECWaypoints(i, :) = [];
            pause(0.5); 
            break; 
        end
    end

    % check if bump sensor triggered
    bumped = false;
    if size(dataStore.bump, 1) >= 1
        if any(dataStore.bump(end, 2:end) > 0)
            bumped = true;
        end
    end
    
    % drive robot
    if bumped
        SetFwdVelAngVelCreate(Robot, 0, 0);

        % check if optional wall bumped
        for w = 1:size(optWalls, 1)
            if dataStore.optWallStates(w) == 0 
                midX = (optWalls(w,1) + optWalls(w,3))/2;
                midY = (optWalls(w,2) + optWalls(w,4))/2;
                
                distToWall = norm([x, y] - [midX, midY]);
                
                if distToWall < 0.35 
                    dataStore.optWallStates(w) = 1;
                end
            end
        end

        pause(0.1);
     
        SetFwdVelAngVelCreate(Robot, -0.1, 0);
        pause(0.5); 
        
        SetFwdVelAngVelCreate(Robot, 0, 0.5); 
        pause(0.5);
        
        SetFwdVelAngVelCreate(Robot, 0, 0);
        
    else
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
    
    % break if path is exhausted
    if prmIdx > size(physical_path_coords, 1) && ~isempty(unvisitedWaypoints)
        disp('Reached end of path, but waypoints remain unvisited.');
        break;
    end
    
    pause(0.05); 
end

SetFwdVelAngVelCreate(Robot, 0, 0);
disp('Regular Waypoints Run Complete');

if toc < maxTime && ~isempty(unvisitedECWaypoints)
    disp('Planning route for remaining EC waypoints...');
    
    % use the final position from PF as the new start pose
    start_pose_ec = [x, y];
    target_waypoints_ec = [start_pose_ec; unvisitedECWaypoints];
    num_targets_ec = size(target_waypoints_ec, 1);
    
    disp('Building PRM for EC Run...');
    roadmap_ec = buildPRM(all_walls, boundaries, numSamples, robot_radius, target_waypoints_ec);
    target_indices_ec = 1:num_targets_ec;
    [cost_matrix_ec, parent_matrix_ec] = buildCostMatrix(roadmap_ec, target_indices_ec);
    optimal_order_ec = shortestHamiltonianPath(cost_matrix_ec);
    
    if ~isempty(optimal_order_ec)
        low_level_node_path_ec = extractPath(optimal_order_ec, target_indices_ec, parent_matrix_ec);
        physical_path_coords_ec = roadmap_ec.nodes(low_level_node_path_ec, :);
        
        prmIdx = 1;
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
        
        disp('Starting EC Navigation...');
        
        while toc < maxTime && ~isempty(unvisitedECWaypoints)
            current_time = toc;
            
            % read sensors
            [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
            
            % extract control input
            if size(dataStore.odometry, 1) >= 1
                u = [dataStore.odometry(end, 2); dataStore.odometry(end, 3)];
            else
                u = [0; 0];
            end
            
            % extract measurements and filter NaNs
            if size(dataStore.rsdepth, 1) >= 1
                z_raw = dataStore.rsdepth(end, 3:end)'; 
                num_rays_total = length(z_raw);
                angles_raw = linspace(27 * pi/180, -27 * pi/180, num_rays_total)';
                
                valid_idx = ~isnan(z_raw) & ~isinf(z_raw) & (z_raw > 0.1) & (z_raw < 5.0);
                
                z = z_raw(valid_idx);
                angles_valid = angles_raw(valid_idx);
                num_valid = length(z);
                
                if num_valid > 0
                    Q_pf = eye(num_valid) * 0.1; 
                    update_fn = @(pose) depthPredict(pose, all_walls, sensorOrigin, angles_valid);
                else
                    z = []; Q_pf = []; update_fn = @(pose) [];
                end
            else
                z = []; Q_pf = []; update_fn = @(pose) [];
            end
            
            % run particle filter
            [particles, ~] = PF(particles, u, z, R, Q_pf, predict_func, update_fn);
            
            x = mean(particles(1,:));
            y = mean(particles(2,:));
            theta = atan2(mean(sin(particles(3,:))), mean(cos(particles(3,:))));
            dataStore.robotPose = [dataStore.robotPose; [current_time, x, y, theta]];
            
            % navigate using visitWaypoints
            [cmdV, cmdW, prmIdx, unvisitedECWaypointsNew] = visitWaypoints(x, y, theta, physical_path_coords_ec, prmIdx, unvisitedECWaypoints, epsilon);
                
            % check EC waypoints 
            if size(unvisitedECWaypointsNew, 1) < size(unvisitedECWaypoints, 1)
                SetFwdVelAngVelCreate(Robot, 0, 0);
                try beepCreate(Robot); catch; beep; end
                hit_wp = setdiff(unvisitedECWaypoints, unvisitedECWaypointsNew, 'rows');
                dataStore.waypoints = [dataStore.waypoints; hit_wp];
                disp(['*** EC SCORE at (', num2str(hit_wp(1)), ', ', num2str(hit_wp(2)), ')! ***']);
                pause(0.5); 
            end
            unvisitedECWaypoints = unvisitedECWaypointsNew;
            
            % check if bump sensor triggered
            bumped = false;
            if size(dataStore.bump, 1) >= 1
                if any(dataStore.bump(end, 2:end) > 0)
                    bumped = true;
                end
            end
            
            % drive robot
            if bumped
                SetFwdVelAngVelCreate(Robot, 0, 0);
                % check if optional wall bumped
                for w = 1:size(optWalls, 1)
                    if dataStore.optWallStates(w) == 0 
                        midX = (optWalls(w,1) + optWalls(w,3))/2;
                        midY = (optWalls(w,2) + optWalls(w,4))/2;
                        
                        distToWall = norm([x, y] - [midX, midY]);
                        
                        if distToWall < 0.35 
                            dataStore.optWallStates(w) = 1;
                        end
                    end
                end
                pause(0.1);
                SetFwdVelAngVelCreate(Robot, -0.1, 0);
                pause(0.5); 
                SetFwdVelAngVelCreate(Robot, 0, 0.5); 
                pause(0.5);
                SetFwdVelAngVelCreate(Robot, 0, 0);
            else
                [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);
                SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
            end
            
            % break if path is exhausted
            if prmIdx > size(physical_path_coords_ec, 1) && ~isempty(unvisitedECWaypoints)
                disp('Reached end of EC path.');
                break;
            end
            
            pause(0.05); 
        end
    else
        disp('Failed to plan a path to remaining EC waypoints.');
    end
    SetFwdVelAngVelCreate(Robot, 0, 0);
    disp('Extra Credit Run Complete.');
else
    disp('Either time is up, or all EC Waypoints were collected on the first run!');
end

% create final figure
figure('Name', 'AMR Final Competition Run', 'NumberTitle', 'off');
hold on; grid on; axis equal;
xlim([boundaries(1), boundaries(2)]);
ylim([boundaries(3), boundaries(4)]);
title('Final Competition Run');
xlabel('X (m)');
ylabel('Y (m)');

% plot known walls
for i = 1:size(map, 1)
    plot([map(i,1), map(i,3)], [map(i,2), map(i,4)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
end

% plot optional walls
optWallStates = dataStore.optWallStates;
for i = 1:size(optWalls, 1)
    if optWallStates(i) == 0 
        plot([optWalls(i,1), optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], ...
            'r-', 'LineWidth', 2, 'HandleVisibility', 'off');
    elseif optWallStates(i) == 1
        plot([optWalls(i,1), optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], ...
            'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end

% plot robot trajectory
if ~isempty(dataStore.robotPose)
    poseX = dataStore.robotPose(:, 2);
    poseY = dataStore.robotPose(:, 3);
    plot(poseX, poseY, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Trajectory');

    plot(poseX(1), poseY(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point');
    plot(poseX(end), poseY(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End Point');
end

% plot visited waypoints
if ~isempty(dataStore.waypoints)
    plot(dataStore.waypoints(:,1), dataStore.waypoints(:,2), 'p', ...
        'MarkerSize', 12, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', ...
        'DisplayName', 'Visited Waypoints');
end

plot(NaN, NaN, 'k-', 'LineWidth', 2, 'DisplayName', 'Known / Verified Walls');
plot(NaN, NaN, 'r-', 'LineWidth', 2, 'DisplayName', 'Undetermined Walls');

legend('Location', 'bestoutside');
hold off;

end