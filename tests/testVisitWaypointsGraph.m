function testVisitWaypointsGraph(Robot, startIdx)
% TESTFULLPIPELINE: End-to-end test of the Waypoint Motion Planning strategy
% 
%   INPUTS
%       Robot       iRobot Create object (from simulator initialization)
%       startIdx    (Optional) Index of the starting waypoint. Defaults to 3.

    if nargin < 2
        startIdx = 3; 
    end
    
    % ---------------------------------------------------------
    % 1. LOAD MAP & INITIALIZE
    % ---------------------------------------------------------
    disp(['--- PHASE 1: Initialization (Starting at Waypoint ', num2str(startIdx), ') ---']);
    mapData = load('PracticeMap2026.mat');
    
    if isfield(mapData, 'optWalls')
        all_walls = [mapData.map; mapData.optWalls];
    else
        all_walls = mapData.map;
    end
    
    standard_waypoints = mapData.waypoints;
    boundaries = [-3.048, 3.048, -2.286, 2.286]; 
    
    num_standard_waypoints = size(standard_waypoints, 1);
    if startIdx < 1 || startIdx > num_standard_waypoints
        error(['startIdx must be between 1 and ', num2str(num_standard_waypoints)]);
    end
    
    start_pose = standard_waypoints(startIdx, :);
    unvisitedWaypoints = standard_waypoints;
    unvisitedWaypoints(startIdx, :) = []; 
    
    target_waypoints = [start_pose; unvisitedWaypoints];
    num_targets = size(target_waypoints, 1);
    
    if isfield(mapData, 'ECwaypoints')
        unvisitedECWaypoints = mapData.ECwaypoints;
    else
        unvisitedECWaypoints = []; 
    end
    
    % ---------------------------------------------------------
    % 2. PLANNING PHASE
    % ---------------------------------------------------------
    disp('--- PHASE 2: Motion Planning ---');
    numSamples = 2000; 
    robot_radius = 0.2; 
    disp('Building PRM...');
    roadmap = buildPRM(all_walls, boundaries, numSamples, robot_radius, target_waypoints);
    
    disp('Calculating Dijkstra Cost Matrix...');
    target_indices = 1:num_targets;
    [cost_matrix, parent_matrix] = buildCostMatrix(roadmap, target_indices);
    
    disp('Solving Shortest Hamiltonian Path...');
    optimal_order = shortestHamiltonianPath(cost_matrix);
    
    if isempty(optimal_order)
        error('Planning Failed: No valid path to visit all waypoints.');
    end
    
    disp('Extracting physical PRM route...');
    low_level_node_path = extractPath(optimal_order, target_indices, parent_matrix);
    physical_path_coords = roadmap.nodes(low_level_node_path, :);
    disp('Planning Complete! Ready to drive.');
    
    % ---------------------------------------------------------
    % 3. EXECUTION PHASE (CONTROL LOOP WITH PF)
    % ---------------------------------------------------------
    disp('--- PHASE 3: Execution ---');
    
    maxTime = 300;           
    maxV = 0.1;              
    wheel2center = 0.13;     
    epsilon = 0.2;           
    prmIdx = 1;
    
    % --- PF INITIALIZATION ---
    M = 50; 
    particles = zeros(3, M);
    particles(1, :) = start_pose(1) + 0.1 * randn(1, M);
    particles(2, :) = start_pose(2) + 0.1 * randn(1, M);
    particles(3, :) = 0 + 0.05 * randn(1, M); 
    
    R_pf = diag([0.005, 0.005, 0.01]); 
    predict_fn = @integrateOdom;       
    sensor_pos = [0.13, 0];            
    
    global dataStore
    dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], 'bump', [], ...
                       'beacon', [], 'pfMu', []);
    noRobotCount = 0;
    SetFwdVelAngVelCreate(Robot, 0, 0);

    % =========================================================
    % --- LIVE PLOT SETUP (DO THIS BEFORE THE LOOP) ---
    % =========================================================
    fig_live = figure('Name', 'Live PF Tracking vs Truth');
    clf(fig_live); % Clear it in case it was already open
    hold on; grid on; axis equal;
    xlim([boundaries(1), boundaries(2)]);
    ylim([boundaries(3), boundaries(4)]);
    title('Live Tracking: True Pose (Blue) vs PF Estimate (Red) vs Particles (Pink)');
    xlabel('X (m)'); ylabel('Y (m)');
    
    % 1. Plot static map features (Walls and PRM path)
    for w = 1:size(all_walls, 1)
        plot([all_walls(w,1), all_walls(w,3)], [all_walls(w,2), all_walls(w,4)], 'k-', 'LineWidth', 1.5);
    end
    plot(standard_waypoints(:,1), standard_waypoints(:,2), 'bp', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(physical_path_coords(:,1), physical_path_coords(:,2), 'g:', 'LineWidth', 1);

    % 2. Initialize dynamic graphics handles (Starts with dummy/initial data)
    h_particles = plot(particles(1,:), particles(2,:), '.', 'Color', [1.0 0.6 0.6], 'MarkerSize', 4);
    
    h_truth_trail = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
    h_truth_pt = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    h_pf_trail = plot(start_pose(1), start_pose(2), 'r--', 'LineWidth', 1.5);
    h_pf_pt = plot(start_pose(1), start_pose(2), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    % =========================================================
    
    tic;
    while toc < maxTime && ~isempty(unvisitedWaypoints)
        
        current_time = toc;
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
        
        % 1. Extract Control Input (u) from Odometry
        if size(dataStore.odometry, 1) >= 1
            d = dataStore.odometry(end, 2); 
            phi = dataStore.odometry(end, 3);
            u = [d; phi];
        else
            u = [0; 0];
        end
        
        % 2. Extract Measurement (z) with Hardware NaN Filtering
        if size(dataStore.rsdepth, 1) >= 1
            z_raw = dataStore.rsdepth(end, 3:end)'; 
            num_rays_total = length(z_raw);
            angles_raw = linspace(27 * pi/180, -27 * pi/180, num_rays_total)';
            
            valid_idx = ~isnan(z_raw) & ~isinf(z_raw) & (z_raw > 0.1) & (z_raw < 5.0);
            
            z = z_raw(valid_idx);
            angles = angles_raw(valid_idx);
            num_valid = length(z);
            
            if num_valid > 0
                Q_pf = eye(num_valid) * 0.1; 
                update_fn = @(pose) depthPredict(pose, all_walls, sensor_pos, angles);
            else
                z = [];
                Q_pf = [];
                update_fn = @(pose) [];
            end
        else
            z = [];
            Q_pf = [];
            update_fn = @(pose) [];
        end
        
        % 3. Run the Particle Filter Step
        [particles, ~] = PF(particles, u, z, R_pf, Q_pf, predict_fn, update_fn);
        
        x = mean(particles(1,:));
        y = mean(particles(2,:));
        theta = atan2(mean(sin(particles(3,:))), mean(cos(particles(3,:))));
        
        mu = [x; y; theta];
        dataStore.pfMu = [dataStore.pfMu; [current_time, mu']];

        % =========================================================
        % --- UPDATE LIVE PLOT GRAPHICS HANDLES ---
        % =========================================================
        % Update particle cloud
        set(h_particles, 'XData', particles(1,:), 'YData', particles(2,:));
        
        % Update truth trajectory (if overhead camera data exists)
        if ~isempty(dataStore.truthPose)
            set(h_truth_trail, 'XData', dataStore.truthPose(:,2), 'YData', dataStore.truthPose(:,3));
            set(h_truth_pt, 'XData', dataStore.truthPose(end,2), 'YData', dataStore.truthPose(end,3));
        end
        
        % Update PF trajectory
        set(h_pf_trail, 'XData', dataStore.pfMu(:,2), 'YData', dataStore.pfMu(:,3));
        set(h_pf_pt, 'XData', mu(1), 'YData', mu(2));
        
        % drawnow limitrate intelligently drops frames to keep the code running fast
        drawnow limitrate;
        % =========================================================
        
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
        
        pause(0.05); 
    end
    
    SetFwdVelAngVelCreate(Robot, 0, 0);
    
    if isempty(unvisitedWaypoints)
        disp('MISSION ACCOMPLISHED: All regular waypoints successfully visited!');
    else
        disp('MISSION ENDED: Time limit reached or path exhausted.');
    end
    
    % ---------------------------------------------------------
    % 4. PLOTTING PHASE (Final Static Plot)
    % ---------------------------------------------------------
    disp('--- PHASE 4: Generating Final Diagnostics ---');
    
    if ~isempty(dataStore.pfMu)
        figure('Name', 'Final PF vs Truth Pose Trajectory'); hold on; grid on;
        
        if ~isempty(dataStore.truthPose)
            plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'True Pose (Overhead)');
            plot(dataStore.truthPose(1,2), dataStore.truthPose(1,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Truth Start');
            plot(dataStore.truthPose(end,2), dataStore.truthPose(end,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Truth End');
        end
        
        plot(dataStore.pfMu(:,2), dataStore.pfMu(:,3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'PF Estimate');
        plot(dataStore.pfMu(1,2), dataStore.pfMu(1,3), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'PF Start');
        plot(dataStore.pfMu(end,2), dataStore.pfMu(end,3), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'PF End');
        
        title('Final Robot Trajectory Tracking');
        xlabel('Global X Position (m)');
        ylabel('Global Y Position (m)');
        legend('Location', 'best');
        axis equal;
    end
end