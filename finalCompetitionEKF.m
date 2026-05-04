function [dataStore] = finalCompetitionEKF(Robot, use_truth_angle)
% main function for AMR final competition (EKF localization variant)
%   use_truth_angle  (optional, default false): if true, overrides the
%       refined heading with the overhead localization truth pose (sim only)
    if nargin < 2, use_truth_angle = false; end

%% Initial Localizaiton
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

    disp('Final refined_pose going into navigation:');
    disp(refined_pose);

    disp(waypoints)
    disp(waypoint_idx);
    
    nav_sigma = diag([0.05^2, 0.05^2, (45*pi/180)^2]);
    testVisitWaypointsEKF(Robot, waypoint_idx, refined_pose, nav_sigma);

end
