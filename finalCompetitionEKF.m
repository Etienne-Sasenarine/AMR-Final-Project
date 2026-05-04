function [dataStore] = finalCompetitionEKF(Robot, use_truth_angle, show_plots)
% main function for AMR final competition (EKF localization variant)
%   use_truth_angle  (optional, default false): if true, overrides the
%       refined heading with the overhead localization truth pose (sim only)
%   show_plots       (optional, default false): if true, opens live trajectory
%       figure during navigation -- disable for real competition runs
    if nargin < 2, use_truth_angle = false; end
    if nargin < 3, show_plots = false;      end

%% Initial Localizaiton
addpath("EKF Stuff\");
addpath("tests\");
%1. Load the map data
    mapData = load("PracticeMap2026.mat");
    waypoints = mapData.waypoints;
    beacons = mapData.beaconLoc;
    map = mapData.map;

    % 2. Get the TRUE number of depth sensors dynamically
    % Take a quick reading to see exactly how many points the RealSense returns
    dummy_depth = RealSenseDist(Robot);
    num_depth_sensors = length(dummy_depth);

    disp(['RealSense configured with ', num2str(num_depth_sensors), ' depth points.']);

    % 3. Define Realsense parameters for depthPredict based on the true length
    sensorOrigin = [0.13; 0];

    % Spread the angles evenly across the 54-degree Field of View (-27 to +27 deg)
    angles = linspace(-27*(pi/180), 27*(pi/180), num_depth_sensors);

    % 4. Define Noise Matrices (used by the initial particle filter only)
    R = diag([0.001, 0.001, 0.01]);
    Q = 0.05 * eye(num_depth_sensors); % Now correctly matches the size of z

    % 5. Define Function Handles
    predict_func = @integrateOdom;
    update_func = @(pose) depthPredict(pose, map, sensorOrigin, angles);

    % 6. Execute Localization
    disp('Starting initial localization sequence...');
    initial_pose = initLocalize(Robot, waypoints, beacons, map, R, Q, predict_func, update_func);

    disp('PF localization complete. Initial Pose:');
    disp(initial_pose);
    fprintf('[DIAG] After PF: theta = %.1f deg\n', initial_pose(3) * 180/pi);

    %% Snap position to the exact starting waypoint
    % The PF identifies which waypoint we're at; the waypoint's known map
    % coordinates are far more accurate than any particle-mean estimate.
    % Trust XY = waypoint coords exactly. Heading stays as-is (the PF only
    % clusters to ~18 deg, so we don't trust it -- see refineHeadingEKF).
    waypoint_idx = 1;
    dist = inf;
    for i = 1:size(waypoints,1)
        distCurr = norm(initial_pose(1:2) - waypoints(i,:)');
        if  distCurr < dist
            dist = distCurr;
            waypoint_idx = i;
        end
    end
    initial_pose(1) = waypoints(waypoint_idx, 1);   % exact x
    initial_pose(2) = waypoints(waypoint_idx, 2);   % exact y
    disp(['Snapped XY to waypoint ', num2str(waypoint_idx), ': ', ...
          num2str(initial_pose(1)), ', ', num2str(initial_pose(2))]);
    fprintf('[DIAG] After XY snap: theta still = %.1f deg\n', initial_pose(3) * 180/pi);

    %% Refine heading via EKF rotation-in-place
    % Position is now ground-truth-accurate; theta is the only unknown.
    % Spin slowly and let depth + beacons pull theta to truth before driving.
    disp('Refining heading with EKF (rotating in place)...');
    refined_pose = refineHeadingEKF(Robot, initial_pose, map, beacons, ...
                                    sensorOrigin, angles);
    fprintf('[DIAG] After refineHeadingEKF: theta = %.1f deg\n', refined_pose(3) * 180/pi);

    % Optional: override theta from overhead truth (simulator only)
    if use_truth_angle
        truth_ds = struct('truthPose', [], 'odometry', [], 'rsdepth', [], ...
                          'bump', [], 'beacon', []);
        noRobotCountTmp = 0;
        for k_snap = 1:5
            [noRobotCountTmp, truth_ds] = readStoreSensorData(Robot, noRobotCountTmp, truth_ds);
            pause(0.1);
        end
        if ~isempty(truth_ds.truthPose)
            truth_theta = truth_ds.truthPose(end, 4);
            fprintf('[DIAG] use_truth_angle=true: overriding theta %.1f deg -> %.1f deg (truth)\n', ...
                    refined_pose(3)*180/pi, truth_theta*180/pi);
            refined_pose(3) = truth_theta;
        else
            disp('[DIAG] use_truth_angle=true but no truthPose available; keeping refined theta.');
        end
    end

    %% Beacon-center heading alignment
    % If any known beacon is within sensor range of the starting waypoint,
    % rotate the robot until that beacon is centered in the camera FOV
    % (y_rel == 0 means the beacon is directly ahead).  At that moment
    % theta = atan2(by - y, bx - x) is ground-truth, so we override
    % refined_pose(3) with a geometrically exact value.
    %
    % Spin direction is chosen as the shortest angular path from the current
    % EKF heading toward the nearest in-range beacon, so this step typically
    % takes only a small fraction of a full rotation.

    % Find beacons within 3 m of the snapped starting position
    in_range = [];
    for bi = 1:size(beacons, 1)
        if norm([beacons(bi,2) - refined_pose(1), beacons(bi,3) - refined_pose(2)]) < 3.0
            in_range(end+1) = bi; %#ok<AGROW>
        end
    end

    if ~isempty(in_range)
        disp('Beacon in range — centering in FOV for exact heading lock...');

        % Pick spin direction: shortest path from current theta to nearest beacon
        min_ang_err = inf;
        spin_dir    = 1;   % +1 = CCW, -1 = CW
        for bi = in_range
            bx_b = beacons(bi, 2);  by_b = beacons(bi, 3);
            th_to_bcn = atan2(by_b - refined_pose(2), bx_b - refined_pose(1));
            ang_err   = atan2(sin(th_to_bcn - refined_pose(3)), ...
                              cos(th_to_bcn - refined_pose(3)));
            if abs(ang_err) < abs(min_ang_err)
                min_ang_err = ang_err;
                spin_dir    = sign(ang_err);
            end
        end
        if spin_dir == 0, spin_dir = 1; end

        turnW_bcn    = spin_dir * 0.2;   % slow rotation toward beacon
        center_tol   = 0.08;             % y_rel < 8 cm ≈ 3–5 deg centered
        max_bcn_rot  = pi;               % give up after half rotation
        bcn_angle    = 0;
        bcn_aligned  = false;

        ds_bcn       = struct('truthPose',[],'odometry',[],'rsdepth',[],'bump',[],'beacon',[]);
        noRobotCountB = 0;

        while ~bcn_aligned && bcn_angle < max_bcn_rot
            [cV, cW] = limitCmds(0, turnW_bcn, 0.2, 0.13);
            SetFwdVelAngVelCreate(Robot, cV, cW);
            [noRobotCountB, ds_bcn] = readStoreSensorData(Robot, noRobotCountB, ds_bcn);

            if ~isempty(ds_bcn.odometry)
                bcn_angle = bcn_angle + abs(ds_bcn.odometry(end, 3));
            end

            if ~isempty(ds_bcn.beacon)
                row = ds_bcn.beacon(end, :);
                if numel(row) >= 5
                    tagID = row(3);
                    y_rel = row(5);   % lateral offset: 0 = beacon dead-ahead
                    bRow  = beacons(beacons(:,1) == tagID, :);
                    if ~isempty(bRow) && abs(y_rel) < center_tol
                        SetFwdVelAngVelCreate(Robot, 0, 0);
                        bx_b = bRow(1, 2);  by_b = bRow(1, 3);
                        % Ground-truth theta: robot is pointing straight at beacon
                        theta_locked = atan2(by_b - refined_pose(2), ...
                                             bx_b - refined_pose(1));
                        refined_pose(3) = theta_locked;
                        fprintf('[ALIGN] Tag %d centered (y_rel=%.3f m) -> theta = %.1f deg\n', ...
                                tagID, y_rel, theta_locked * 180/pi);
                        bcn_aligned = true;
                    end
                end
            end
            pause(0.02);
        end

        SetFwdVelAngVelCreate(Robot, 0, 0);
        if ~bcn_aligned
            disp('[ALIGN] Beacon not centered within half-rotation. Using existing theta.');
        end
    else
        disp('[ALIGN] No beacon within 3 m — skipping alignment step.');
    end

    disp('Final refined_pose going into navigation:');
    disp(refined_pose);
    fprintf('[DIAG] Navigation start: theta = %.1f deg\n', refined_pose(3) * 180/pi);

    disp(waypoints)
    disp(waypoint_idx);

    nav_sigma = diag([0.05^2, 0.05^2, (10*pi/180)^2]);   % tight theta: just aligned to beacon
    testVisitWaypointsEKF(Robot, waypoint_idx, refined_pose, nav_sigma, show_plots);

end
