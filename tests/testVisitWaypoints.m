function testVisitWaypoints(Robot, startIdx)
% TESTFULLPIPELINE: End-to-end test of the Waypoint Motion Planning strategy
% 
%   INPUTS
%       Robot       iRobot Create object (from simulator initialization)
%       startIdx    (Optional) Index of the starting waypoint. Defaults to 1.

    % Handle optional startIdx argument
    if nargin < 2
        startIdx = 1; 
    end

    % ---------------------------------------------------------
    % 1. LOAD MAP & INITIALIZE
    % ---------------------------------------------------------
    disp(['--- PHASE 1: Initialization (Starting at Waypoint ', num2str(startIdx), ') ---']);
    % Load the practice map (Ensure this file is in your directory)
    mapData = load('PracticeMap2026.mat');
    
    all_walls = [mapData.map; mapData.optWalls];
    standard_waypoints = mapData.waypoints;
    boundaries = [-3.048, 3.048, -2.286, 2.286]; 
    
    % Validate the requested start index
    num_standard_waypoints = size(standard_waypoints, 1);
    if startIdx < 1 || startIdx > num_standard_waypoints
        error(['startIdx must be between 1 and ', num2str(num_standard_waypoints)]);
    end
    
    % Dynamically set the start pose based on the index
    start_pose = standard_waypoints(startIdx, :);
    
    % Create the unvisited targets list by copying all waypoints, 
    % then deleting the row corresponding to our start location
    unvisitedWaypoints = standard_waypoints;
    unvisitedWaypoints(startIdx, :) = []; 
    
    % Combine start pose and unvisited waypoints for the PRM/TSP planner
    target_waypoints = [start_pose; unvisitedWaypoints];
    num_targets = size(target_waypoints, 1);
    
    % ---------------------------------------------------------
    % 2. PLANNING PHASE
    % ---------------------------------------------------------
    disp('--- PHASE 2: Motion Planning ---');
    
    % A. Build PRM
    numSamples = 500; 
    robot_radius = 0.2; 
    disp('Building PRM...');
    roadmap = buildPRM(all_walls, boundaries, numSamples, robot_radius, target_waypoints);
    
    % B. Build Cost Matrix
    disp('Calculating Dijkstra Cost Matrix...');
    target_indices = 1:num_targets;
    [cost_matrix, parent_matrix] = buildCostMatrix(roadmap, target_indices);
    
    % C. Solve TSP / Shortest Hamiltonian Path
    disp('Solving Shortest Hamiltonian Path...');
    optimal_order = shortestHamiltonianPath(cost_matrix);
    
    if isempty(optimal_order)
        error('Planning Failed: No valid path to visit all waypoints.');
    end
    
    % D. Extract Low-Level PRM Path
    disp('Extracting physical PRM route...');
    low_level_node_path = extractPath(optimal_order, target_indices, parent_matrix);
    physical_path_coords = roadmap.nodes(low_level_node_path, :);
    
    disp('Planning Complete! Ready to drive.');
    disp(' ');
    
    % ---------------------------------------------------------
    % 3. EXECUTION PHASE (CONTROL LOOP)
    % ---------------------------------------------------------
    disp('--- PHASE 3: Execution ---');
    
    % Control Parameters
    maxTime = 300;           
    maxV = 0.2;              % STRICT final competition speed limit [cite: 153]
    wheel2center = 0.13;     
    epsilon = 0.15;          % Turn radius / lookahead distance
    prmIdx = 1;
    
    % Initialize Sensor Data Store
    global dataStore
    dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], 'bump', [], 'beacon', []);
    noRobotCount = 0;
    SetFwdVelAngVelCreate(Robot, 0, 0);
    
    tic;
    while toc < maxTime && ~isempty(unvisitedWaypoints)
        
        % Read sensors
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
        
        if isempty(dataStore.truthPose)
            continue;
        end
        
        % --------------------------------------------------------------
        % CRITICAL WARNING FOR FINAL COMPETITION:
        % Overhead localization is banned. You MUST replace truthPose 
        % with your EKF output (x, y, theta) in the final script. [cite: 137]
        % --------------------------------------------------------------
        x = dataStore.truthPose(end, 2);
        y = dataStore.truthPose(end, 3);
        theta = dataStore.truthPose(end, 4);
        
        % Call your visitWaypoints function
        [cmdV, cmdW, prmIdx, unvisitedWaypointsNew] = visitWaypoints(...
            x, y, theta, physical_path_coords, prmIdx, unvisitedWaypoints, epsilon);
            
        % Check if a waypoint was removed (meaning we scored!)
        if size(unvisitedWaypointsNew, 1) < size(unvisitedWaypoints, 1)
            SetFwdVelAngVelCreate(Robot, 0, 0);
            
            % Trigger the competition-required beep [cite: 154]
            try
                beepCreate(Robot); 
            catch
                beep; % Fallback to MATLAB system beep if beepCreate isn't loaded
            end
            
            disp(['SCORE! Remaining waypoints: ', num2str(size(unvisitedWaypointsNew, 1))]);
            pause(0.5); % Brief pause to stabilize after scoring
        end
        
        unvisitedWaypoints = unvisitedWaypointsNew;
        
        % Enforce speed limits and send commands
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        
        % Break if we reached the end of the PRM path
        if prmIdx > size(physical_path_coords, 1) && ~isempty(unvisitedWaypoints)
            disp('Reached end of PRM path, but waypoints remain unvisited.');
            break;
        end
        
        pause(0.1);
    end
    % Stop the robot cleanly
    SetFwdVelAngVelCreate(Robot, 0, 0);
    
    if isempty(unvisitedWaypoints)
        disp('MISSION ACCOMPLISHED: All waypoints successfully visited!');
    else
        disp('MISSION ENDED: Time limit reached or path exhausted.');
    end
end