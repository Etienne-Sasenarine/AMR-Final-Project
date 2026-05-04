function testVisitWaypoints(Robot, startIdx)
% TESTFULLPIPELINE: End-to-end test of the Waypoint Motion Planning strategy
% 
%   INPUTS
%       Robot       iRobot Create object (from simulator initialization)
%       startIdx    (Optional) Index of the starting waypoint. Defaults to 3.

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
    
    % Combine known walls and optional walls if they exist
    if isfield(mapData, 'optWalls')
        all_walls = [mapData.map; mapData.optWalls];
    else
        all_walls = mapData.map;
    end
    
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
    % 3. EXECUTION PHASE (CONTROL LOOP WITH PF)
    % ---------------------------------------------------------
    disp('--- PHASE 3: Execution ---');
    
    % Control Parameters
    maxTime = 300;           
    maxV = 0.2;              % STRICT final competition speed limit
    wheel2center = 0.13;     
    epsilon = 0.2;           % Turn radius / lookahead distance
    prmIdx = 1;
    
    % --- PF INITIALIZATION ---
    M = 50; % Number of particles for continuous tracking
    particles = zeros(3, M);
    
    % Seed particles around the start pose (Assuming initial heading is 0)
    particles(1, :) = start_pose(1) + 0.1 * randn(1, M);
    particles(2, :) = start_pose(2) + 0.1 * randn(1, M);
    particles(3, :) = 0 + 0.05 * randn(1, M); 
    
    R_pf = diag([0.005, 0.005, 0.01]); % Process noise (motion uncertainty)
    predict_fn = @integrateOdom;       % Odometry prediction step function
    sensor_pos = [0.13, 0];            % Physical offset of the RealSense from center
    
    % Initialize Sensor Data Store 
    global dataStore
    dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], 'bump', [], ...
                       'beacon', [], 'pfMu', []);
    noRobotCount = 0;
    SetFwdVelAngVelCreate(Robot, 0, 0);
    
    tic;
    while toc < maxTime && ~isempty(unvisitedWaypoints)
        
        current_time = toc;
        
        % Read sensors
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
        
        % --------------------------------------------------------------
        % 1. Extract Control Input (u) from Odometry
        % --------------------------------------------------------------
        if size(dataStore.odometry, 1) >= 1
            d = dataStore.odometry(end, 2); 
            phi = dataStore.odometry(end, 3);
            u = [d; phi];
        else
            u = [0; 0];
        end
        
        % --------------------------------------------------------------
        % 2. Extract Measurement (z) with Hardware NaN Filtering
        % --------------------------------------------------------------
        if size(dataStore.rsdepth, 1) >= 1
            z_raw = dataStore.rsdepth(end, 3:end)'; 
            num_rays_total = length(z_raw);
            angles_raw = linspace(27 * pi/180, -27 * pi/180, num_rays_total)';
            
            % DYNAMIC HARDWARE FILTER: Keep only valid floats within sensor range
            valid_idx = ~isnan(z_raw) & ~isinf(z_raw) & (z_raw > 0.1) & (z_raw < 5.0);
            
            z = z_raw(valid_idx);
            angles = angles_raw(valid_idx);
            num_valid = length(z);
            
            if num_valid > 0
                Q_pf = eye(num_valid) * 0.1; 
                update_fn = @(pose) depthPredict(pose, all_walls, sensor_pos, angles);
            else
                % All rays failed (NaNs). Skip measurement update this loop.
                z = [];
                Q_pf = [];
                update_fn = @(pose) [];
            end
        else
            z = [];
            Q_pf = [];
            update_fn = @(pose) [];
        end
        
        % --------------------------------------------------------------
        % 3. Run the Particle Filter Step
        % --------------------------------------------------------------
        [particles, ~] = PF(particles, u, z, R_pf, Q_pf, predict_fn, update_fn);
        
        % Extract current belief (mean) from the particle cloud
        x = mean(particles(1,:));
        y = mean(particles(2,:));
        % Use circular mean for angles to avoid -pi/pi cancellation issues
        theta = atan2(mean(sin(particles(3,:))), mean(cos(particles(3,:))));
        
        mu = [x; y; theta];
        dataStore.pfMu = [dataStore.pfMu; [current_time, mu']];
        
        % --------------------------------------------------------------
        % NAVIGATION LOGIC (USING PF STATE)
        % --------------------------------------------------------------
        [cmdV, cmdW, prmIdx, unvisitedWaypointsNew] = visitWaypoints(...
            x, y, theta, physical_path_coords, prmIdx, unvisitedWaypoints, epsilon);
            
        if size(unvisitedWaypointsNew, 1) < size(unvisitedWaypoints, 1)
            SetFwdVelAngVelCreate(Robot, 0, 0);
            try beepCreate(Robot); catch; beep; end
            disp(['SCORE! Remaining regular waypoints: ', num2str(size(unvisitedWaypointsNew, 1))]);
            pause(0.5); 
        end
        unvisitedWaypoints = unvisitedWaypointsNew;
        
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
        
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        
        if prmIdx > size(physical_path_coords, 1) && ~isempty(unvisitedWaypoints)
            disp('Reached end of PRM path, but waypoints remain unvisited.');
            break;
        end
        
        pause(0.05); % Run fast to ensure tight control loop frequency
    end
    
    % Stop the robot cleanly
    SetFwdVelAngVelCreate(Robot, 0, 0);
    
    if isempty(unvisitedWaypoints)
        disp('MISSION ACCOMPLISHED: All regular waypoints successfully visited!');
    else
        disp('MISSION ENDED: Time limit reached or path exhausted.');
    end
    
    % ---------------------------------------------------------
    % 4. PLOTTING PHASE (Truth vs. PF)
    % ---------------------------------------------------------
    disp('--- PHASE 4: Generating Diagnostics ---');
    
    if ~isempty(dataStore.pfMu)
        figure('Name', 'PF vs Truth Pose Trajectory'); hold on; grid on;
        
        % Plot the true trajectory ONLY if it exists (i.e. if in sim or using Vicon)
        if ~isempty(dataStore.truthPose)
            plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'True Pose (Overhead)');
            plot(dataStore.truthPose(1,2), dataStore.truthPose(1,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Truth Start');
            plot(dataStore.truthPose(end,2), dataStore.truthPose(end,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Truth End');
        end
        
        % Plot the PF estimated trajectory
        plot(dataStore.pfMu(:,2), dataStore.pfMu(:,3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'PF Estimate');
        plot(dataStore.pfMu(1,2), dataStore.pfMu(1,3), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'PF Start');
        plot(dataStore.pfMu(end,2), dataStore.pfMu(end,3), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'PF End');
        
        title('Robot Trajectory Tracking');
        xlabel('Global X Position (m)');
        ylabel('Global Y Position (m)');
        legend('Location', 'best');
        axis equal;
    else
        disp('Warning: Not enough data logged to plot PF comparison.');
    end
end