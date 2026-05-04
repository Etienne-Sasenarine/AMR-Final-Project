function testVisitWaypoints(Robot, startIdx)
% TESTFULLPIPELINE: End-to-end test of the Waypoint Motion Planning strategy
% 
%   INPUTS
%       Robot       iRobot Create object (from simulator initialization)
%       startIdx    (Optional) Index of the starting waypoint. Defaults to 4.

    % Handle optional startIdx argument
    if nargin < 2
        startIdx = 3; 
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
    
    % Create the unvisited targets list
    unvisitedWaypoints = standard_waypoints;
    unvisitedWaypoints(startIdx, :) = []; 
    
    % Combine start pose and unvisited waypoints for the PRM/TSP planner
    target_waypoints = [start_pose; unvisitedWaypoints];
    num_targets = size(target_waypoints, 1);
    
    % Load the EC waypoints list
    if isfield(mapData, 'ECwaypoints')
        unvisitedECWaypoints = mapData.ECwaypoints;
    else
        unvisitedECWaypoints = []; 
    end
    
    % ---------------------------------------------------------
    % 2. PLANNING PHASE
    % ---------------------------------------------------------
    disp('--- PHASE 2: Motion Planning ---');
    
    % A. Build PRM
    numSamples = 2000; 
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
    
    % ---------------------------------------------------------
    % 3. EXECUTION PHASE (CONTROL LOOP)
    % ---------------------------------------------------------
    disp('--- PHASE 3: Execution ---');
    
    % Control Parameters
    maxTime = 300;           
    maxV = 0.2;              % STRICT final competition speed limit
    wheel2center = 0.13;     
    epsilon = 0.2;           % Turn radius / lookahead distance
    prmIdx = 1;
    
    % --- NEW: EKF INITIALIZATION ---
    R = diag([0.005, 0.005, 0.01]); % Process noise (motion)
    % Note: Q is now initialized dynamically inside the loop based on sensor rays
    
    % Assuming initial heading is 0, initialize mu and Sigma
    mu = [start_pose(1); start_pose(2); 0]; 
    Sigma = diag([0.1, 0.1, 0.1]); 
    
    % Initialize Sensor Data Store (Added ekfMu and ekfSigma)
    global dataStore
    dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], 'bump', [], ...
                       'beacon', [], 'ekfMu', [], 'ekfSigma', []);
    noRobotCount = 0;
    SetFwdVelAngVelCreate(Robot, 0, 0);
    
    tic;
    while toc < maxTime && ~isempty(unvisitedWaypoints)
        
        current_time = toc;
        
        % Read sensors
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
        
        if isempty(dataStore.truthPose)
            continue;
        end
        
        % --- NEW: EKF BACKGROUND TRACKING ---
        
        % 1. Extract Control Input (u) from Odometry
        if size(dataStore.odometry, 1) >= 1
            d = dataStore.odometry(end, 2); 
            phi = dataStore.odometry(end, 3);
            u = [d; phi];
        else
            u = [0; 0];
        end
        
        % 2. Extract Measurement (z) from RealSense Depth
        % In dataStore.rsdepth, col 1 is count, col 2 is time, col 3-end are the ray depths
        if size(dataStore.rsdepth, 1) >= 1
            z = dataStore.rsdepth(end, 3:end)'; 
            
            % DYNAMICALLY SIZE Q based on number of depth rays returned
            num_rays = length(z);
            Q = eye(num_rays) * 0.1; 
        else
            z = []; % Pass empty if no measurement this tick
            Q = [];
        end
        
        % 3. Set up Function Pointers for the EKF
        sensor_pos = [0.13, 0]; 
        
        g_func = @(x, u) integrateOdom(x, u(1), u(2)); 
        Gjac_func = @(x, u) GjacDiffDrive(x, u);
        
        if ~isempty(z)
            actual_n = length(z);
            angles = linspace(27 * pi/180, -27 * pi/180, actual_n)';
            
            % Pass 'all_walls' (from Phase 1) as the map
            h_func = @(x) depthPredict(x, all_walls, sensor_pos, angles);
            Hjac_func = @(x) HjacDepth(x, all_walls, sensor_pos, actual_n);
        else
            % Fallback dummy functions for when the EKF is only running the predict step
            h_func = @(x) [];
            Hjac_func = @(x) [];
        end
        
        % 4. Run the EKF Engine
        [mu, Sigma] = EKF(mu, Sigma, u, z, g_func, Gjac_func, h_func, Hjac_func, R, Q);
        
        % Log EKF data
        dataStore.ekfMu = [dataStore.ekfMu; [current_time, mu']];
        
        % --------------------------------------------------------------
        % NAVIGATION LOGIC (STRICTLY USING TRUTH POSE)
        % --------------------------------------------------------------
        x = dataStore.truthPose(end, 2);
        y = dataStore.truthPose(end, 3);
        theta = dataStore.truthPose(end, 4);
        
        % 1. Call your visitWaypoints function for regular navigation
        [cmdV, cmdW, prmIdx, unvisitedWaypointsNew] = visitWaypoints(...
            x, y, theta, physical_path_coords, prmIdx, unvisitedWaypoints, epsilon);
            
        % 2. Check if a regular waypoint was reached
        if size(unvisitedWaypointsNew, 1) < size(unvisitedWaypoints, 1)
            SetFwdVelAngVelCreate(Robot, 0, 0);
            try beepCreate(Robot); catch; beep; end
            disp(['SCORE! Remaining regular waypoints: ', num2str(size(unvisitedWaypointsNew, 1))]);
            pause(0.5); 
        end
        unvisitedWaypoints = unvisitedWaypointsNew;
        
        % 3. Opportunistic Drive-By Scoring for EC Waypoints
        for i = 1:size(unvisitedECWaypoints, 1)
            dist_to_ec = norm([x, y] - unvisitedECWaypoints(i, :));
            
            if dist_to_ec <= 0.2 
                SetFwdVelAngVelCreate(Robot, 0, 0);
                try beepCreate(Robot); catch; beep; end
                disp(['*** EXTRA CREDIT SCORE at (', num2str(unvisitedECWaypoints(i,1)), ', ', num2str(unvisitedECWaypoints(i,2)), ')! ***']);
                unvisitedECWaypoints(i, :) = [];
                pause(0.5); 
                break; 
            end
        end
        
        % 4. Enforce speed limits and send commands
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
        disp('MISSION ACCOMPLISHED: All regular waypoints successfully visited!');
    else
        disp('MISSION ENDED: Time limit reached or path exhausted.');
    end
    
    % ---------------------------------------------------------
    % 4. PLOTTING PHASE (Truth vs. EKF)
    % ---------------------------------------------------------
    disp('--- PHASE 4: Generating Diagnostics ---');
    
    if ~isempty(dataStore.ekfMu) && ~isempty(dataStore.truthPose)
        figure('Name', 'EKF vs Truth Pose Trajectory'); hold on; grid on;
        
        % Plot the true trajectory (Columns 2 and 3 are x and y)
        plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3), 'b-', 'LineWidth', 2);
        
        % Plot the EKF estimated trajectory
        plot(dataStore.ekfMu(:,2), dataStore.ekfMu(:,3), 'r--', 'LineWidth', 1.5);
        
        % Plot Start and End points for clarity
        plot(dataStore.truthPose(1,2), dataStore.truthPose(1,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot(dataStore.truthPose(end,2), dataStore.truthPose(end,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        title('Robot Trajectory: Overhead Camera vs EKF Estimate');
        xlabel('Global X Position (m)');
        ylabel('Global Y Position (m)');
        legend('True Pose (Overhead)', 'EKF Estimate', 'Start Point', 'End Point', 'Location', 'best');
        axis equal;
    else
        disp('Warning: Not enough data logged to plot EKF comparison.');
    end
end