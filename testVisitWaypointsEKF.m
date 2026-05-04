function testVisitWaypointsEKF(Robot, startIdx, initial_pose, initial_sigma)
% testVisitWaypointsEKF: Identical to testVisitWaypoints, but localizes
% with an EKF (odom + RealSense depth + AprilTag beacons) instead of
% reading from dataStore.truthPose.
%
%   INPUTS
%       Robot         iRobot Create object (sim or real)
%       startIdx      Index of the starting waypoint
%       initial_pose  3-by-1 [x; y; theta] from initLocalize

    if nargin < 2,  startIdx = 3;                                               end
    if nargin < 3,  initial_pose = [0; 0; 0];                                   end
    if nargin < 4,  initial_sigma = diag([0.05^2, 0.05^2, (5*pi/180)^2]);      end

    % ---------------------------------------------------------
    % 1. LOAD MAP & INITIALIZE
    % ---------------------------------------------------------
    disp(['--- PHASE 1: Initialization (Starting at Waypoint ', num2str(startIdx), ') ---']);
    mapData = load('PracticeMap2026.mat');

    all_walls           = [mapData.map; mapData.optWalls];
    map                 = mapData.map;          % mandatory walls only (EKF)
    beaconLoc           = mapData.beaconLoc;
    standard_waypoints  = mapData.waypoints;
    boundaries          = [-3.048, 3.048, -2.286, 2.286];

    num_standard_waypoints = size(standard_waypoints, 1);
    if startIdx < 1 || startIdx > num_standard_waypoints
        error(['startIdx must be between 1 and ', num2str(num_standard_waypoints)]);
    end

    start_pose                    = standard_waypoints(startIdx, :);
    unvisitedWaypoints            = standard_waypoints;
    unvisitedWaypoints(startIdx,:) = [];
    target_waypoints              = [start_pose; unvisitedWaypoints];
    num_targets                   = size(target_waypoints, 1);

    if isfield(mapData, 'ECwaypoints')
        unvisitedECWaypoints = mapData.ECwaypoints;
    else
        unvisitedECWaypoints = [];
    end

    % ---------------------------------------------------------
    % 2. PLANNING PHASE
    % ---------------------------------------------------------
    disp('--- PHASE 2: Motion Planning ---');

    numSamples   = 500;
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
    low_level_node_path  = extractPath(optimal_order, target_indices, parent_matrix);
    physical_path_coords = roadmap.nodes(low_level_node_path, :);

    disp('Planning Complete! Ready to drive.');
    disp(' ');

    % ---------------------------------------------------------
    % 2.5. LIVE TRAJECTORY PLOT
    % ---------------------------------------------------------
    fig_traj = figure('Name', 'EKF Localization vs TruthPose'); clf;
    hold on; grid on; axis equal;
    title('PRM plan, truthPose, and EKF estimate');
    xlabel('X (m)'); ylabel('Y (m)');

    % Walls
    for w = 1:size(map, 1)
        plot(map(w, [1 3]), map(w, [2 4]), 'k-', 'LineWidth', 2);
    end
    if isfield(mapData, 'optWalls')
        for w = 1:size(mapData.optWalls, 1)
            plot(mapData.optWalls(w, [1 3]), mapData.optWalls(w, [2 4]), ...
                 'Color', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1.2);
        end
    end

    % Waypoints / EC waypoints / beacons
    h_wp  = plot(standard_waypoints(:,1), standard_waypoints(:,2), ...
                 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
    if ~isempty(unvisitedECWaypoints)
        plot(unvisitedECWaypoints(:,1), unvisitedECWaypoints(:,2), ...
             'p', 'MarkerSize', 10, 'Color', [0 0.6 0], 'MarkerFaceColor', [0 0.8 0]);
    end
    if ~isempty(beaconLoc)
        plot(beaconLoc(:,2), beaconLoc(:,3), 'rs', ...
             'MarkerSize', 9, 'MarkerFaceColor', 'r');
    end

    % Planned PRM path
    h_prm = plot(physical_path_coords(:,1), physical_path_coords(:,2), ...
                 'c-', 'LineWidth', 2);
    plot(physical_path_coords(:,1), physical_path_coords(:,2), 'c.', 'MarkerSize', 10);

    % Trajectory line handles (start empty; we'll grow them in the loop)
    h_truth = plot(NaN, NaN, 'g-',  'LineWidth', 1.6);
    h_ekf   = plot(NaN, NaN, 'r--', 'LineWidth', 1.6);
    h_robot = plot(initial_pose(1), initial_pose(2), ...
                   'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);

    legend([h_wp, h_prm, h_truth, h_ekf, h_robot], ...
           {'Waypoints', 'PRM plan', 'truthPose', 'EKF \mu', 'Robot'}, ...
           'Location', 'bestoutside');
    axis([boundaries(1)-0.2, boundaries(2)+0.2, ...
          boundaries(3)-0.2, boundaries(4)+0.2]);
    drawnow;

    plot_decim = 3;       % update plot every N control ticks
    tick = 0;

    % ---------------------------------------------------------
    % 3. EXECUTION PHASE (CONTROL LOOP) -- EKF replaces truthPose
    % ---------------------------------------------------------
    disp('--- PHASE 3: Execution ---');

    % Control parameters (identical to testVisitWaypoints)
    maxTime      = 300;
    maxV         = 0.2;
    wheel2center = 0.13;
    epsilon      = 0.15;
    prmIdx       = 1;

    % RealSense geometry (must match what initLocalize used)
    dummy_depth       = RealSenseDist(Robot);
    num_depth_sensors = length(dummy_depth);
    sensorOrigin      = [0.13; 0];
    angles            = linspace(-27*(pi/180), 27*(pi/180), num_depth_sensors);

    % EKF state
    mu    = initial_pose(:);
    sigma = initial_sigma;

    % EKF noise (tuned conservatively)
    R_ekf  = diag([0.02^2, 0.02^2, (2*pi/180)^2]);
    Q_dpth = 0.10^2 * eye(num_depth_sensors);
    Q_bcn  = diag([0.05^2, 0.05^2]);

    % Gating + Jacobian sanity bounds
    gate_threshold = 0.5;
    bcn_gate       = 0.4;
    min_valid      = 0.1;
    max_valid      = 4.0;
    H_cap          = 50;

    % Sensor data store (identical to testVisitWaypoints)
    global dataStore
    dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], ...
                       'bump', [], 'beacon', [], 'ekfMu', [], 'ekfSigma', []);
    noRobotCount = 0;
    SetFwdVelAngVelCreate(Robot, 0, 0);

    last_odom_idx   = 0;
    last_depth_idx  = 0;
    last_beacon_idx = 0;

    tic;
    while toc < maxTime && ~isempty(unvisitedWaypoints)

        % Read sensors
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

        % Wait for the first odom sample (mirror original "wait for truthPose")
        if isempty(dataStore.odometry)
            continue;
        end

        % --- EKF PREDICT on new odometry ---
        cur_odom_idx = size(dataStore.odometry, 1);
        if cur_odom_idx > last_odom_idx
            new_rows  = dataStore.odometry(last_odom_idx+1:cur_odom_idx, :);
            u         = [sum(new_rows(:, 2)); sum(new_rows(:, 3))];
            mu_prev   = mu;
            mu        = integrateOdom(mu_prev, u(1), u(2));
            mu(3)     = atan2(sin(mu(3)), cos(mu(3)));
            Gt        = GjacDiffDrive(mu_prev, u);
            sigma     = Gt * sigma * Gt' + R_ekf;
            last_odom_idx = cur_odom_idx;
        end

        % --- EKF UPDATE with depth ---
        if ~isempty(dataStore.rsdepth)
            cur_depth_idx = size(dataStore.rsdepth, 1);
            if cur_depth_idx > last_depth_idx
                z_full     = dataStore.rsdepth(end, 2:end)';
                z_hat_full = depthPredict(mu, map, sensorOrigin, angles);
                H_full     = numericalDepthJac(mu, map, sensorOrigin, angles);
                H_full(abs(H_full) > H_cap) = 0;

                valid = ~isnan(z_full) & ~isnan(z_hat_full) & ...
                        z_full     >= min_valid & z_full     <= max_valid & ...
                        z_hat_full >= min_valid & z_hat_full <= max_valid & ...
                        abs(z_full - z_hat_full) < gate_threshold;

                if any(valid)
                    z     = z_full(valid);
                    z_hat = z_hat_full(valid);
                    H     = H_full(valid, :);
                    Q     = Q_dpth(valid, valid);

                    Kgain = sigma * H' / (H * sigma * H' + Q);
                    mu    = mu + Kgain * (z - z_hat);
                    sigma = (eye(3) - Kgain * H) * sigma;
                    sigma = 0.5 * (sigma + sigma');
                    mu(3) = atan2(sin(mu(3)), cos(mu(3)));
                end
                last_depth_idx = cur_depth_idx;
            end
        end

        % --- EKF UPDATE with AprilTag beacons ---
        if ~isempty(dataStore.beacon)
            cur_beacon_idx = size(dataStore.beacon, 1);
            if cur_beacon_idx > last_beacon_idx
                new_beacon_rows = dataStore.beacon(last_beacon_idx+1:cur_beacon_idx, :);
                for bi = 1:size(new_beacon_rows, 1)
                    [tagID, z_bcn] = parseBeaconRow(new_beacon_rows(bi, :));
                    if isempty(z_bcn), continue; end

                    bRow = beaconLoc(beaconLoc(:,1) == tagID, :);
                    if isempty(bRow), continue; end
                    bxy  = bRow(1, 2:3)';

                    z_hat_bcn = beaconPredict(mu, bxy, sensorOrigin);
                    H_bcn     = beaconJac(mu, bxy, sensorOrigin);

                    innov = z_bcn - z_hat_bcn;
                    if any(abs(innov) > bcn_gate), continue; end

                    Kgain = sigma * H_bcn' / (H_bcn * sigma * H_bcn' + Q_bcn);
                    mu    = mu + Kgain * innov;
                    sigma = (eye(3) - Kgain * H_bcn) * sigma;
                    sigma = 0.5 * (sigma + sigma');
                    mu(3) = atan2(sin(mu(3)), cos(mu(3)));
                end
                last_beacon_idx = cur_beacon_idx;
            end
        end

        % Log EKF estimate
        dataStore.ekfMu    = [dataStore.ekfMu;    toc, mu'];
        dataStore.ekfSigma = [dataStore.ekfSigma; toc, sigma(:)'];

        % --------------------------------------------------------------
        % EKF output replaces truthPose for control
        % --------------------------------------------------------------
        x = mu(1);
        y = mu(2);
        theta = mu(3);

        % 1. Call visitWaypoints for regular navigation
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

        % --- Update live plot (truthPose + EKF mu) ---
        tick = tick + 1;
        if mod(tick, plot_decim) == 0 && ishandle(fig_traj)
            if ~isempty(dataStore.truthPose)
                set(h_truth, 'XData', dataStore.truthPose(:,2), ...
                             'YData', dataStore.truthPose(:,3));
            end
            if ~isempty(dataStore.ekfMu)
                set(h_ekf, 'XData', dataStore.ekfMu(:,2), ...
                           'YData', dataStore.ekfMu(:,3));
            end
            set(h_robot, 'XData', x, 'YData', y);
            drawnow limitrate;
        end

        pause(0.1);
    end

    SetFwdVelAngVelCreate(Robot, 0, 0);

    if isempty(unvisitedWaypoints)
        disp('MISSION ACCOMPLISHED: All regular waypoints successfully visited!');
        if isempty(unvisitedECWaypoints)
            disp('PLUS: All EC waypoints were collected!');
        else
            disp(['Missed ', num2str(size(unvisitedECWaypoints, 1)), ' EC waypoints.']);
        end
    else
        disp('MISSION ENDED: Time limit reached or path exhausted.');
    end
end


%% ========================================================================
%  Helpers
%% ========================================================================
function H = numericalDepthJac(x, map, sensorOrigin, angles)
    K     = length(angles);
    H     = zeros(K, 3);
    delXY = 0.01;
    delTh = deg2rad(1);
    dels  = [delXY; delXY; delTh];

    for i = 1:3
        e       = zeros(3, 1);
        e(i)    = dels(i);
        z_plus  = depthPredict(x + e, map, sensorOrigin, angles);
        z_minus = depthPredict(x - e, map, sensorOrigin, angles);
        bad = isnan(z_plus) | isnan(z_minus);
        z_plus(bad)  = 0;
        z_minus(bad) = 0;
        H(:, i)   = (z_plus - z_minus) / (2 * dels(i));
        H(bad, i) = 0;
    end
end

function [tagID, z_rel] = parseBeaconRow(row)
% Assumes dataStore.beacon row format: [time, _, tagID, x_rel, y_rel].
% Adjust column indices here if your RealSenseTag returns a different shape.
    if numel(row) < 5
        tagID = []; z_rel = []; return;
    end
    tagID = row(3);
    z_rel = [row(4); row(5)];
end

function z = beaconPredict(mu, bxy, sensorOrigin)
    x  = mu(1); y = mu(2); th = mu(3);
    sx = sensorOrigin(1); sy = sensorOrigin(2);
    sxw = x + sx*cos(th) - sy*sin(th);
    syw = y + sx*sin(th) + sy*cos(th);
    dxw = bxy(1) - sxw;
    dyw = bxy(2) - syw;
    z = [ cos(th)*dxw + sin(th)*dyw;
         -sin(th)*dxw + cos(th)*dyw];
end

function H = beaconJac(mu, bxy, sensorOrigin)
    th = mu(3);
    sx = sensorOrigin(1); sy = sensorOrigin(2);
    z  = beaconPredict(mu, bxy, sensorOrigin);
    xr = z(1); yr = z(2);
    H = [ -cos(th), -sin(th),   yr + sy ;
           sin(th), -cos(th), -(xr + sx)];
end
