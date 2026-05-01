function [dataStore] = finalCompetition(Robot)%(Robot, maxTime, offset_x, offset_y)
% main function for AMR final competition

%% Initial Localizaiton
% Initialize Sensor Data Store
    global dataStore
    dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], 'bump', [], 'beacon', [], 'EKFPose', []);
    noRobotCount = 0;

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

    % 4. Define Noise Matrices
    R = diag([0.001, 0.001, 0.01]);
    Q = 0.05 * eye(num_depth_sensors); % Now correctly matches the size of z

    % 5. Define Function Handles
    predict_func = @integrateOdom;
    update_func = @(pose) depthPredict(pose, map, sensorOrigin, angles);

    % 6. Execute Localization
    disp('Starting initial localization sequence...');
    initial_pose = initLocalize(Robot, waypoints, beacons, map, R, Q, predict_func, update_func);
    
    disp('Localization complete. Final Pose:');
    disp(initial_pose);

    %% set starting point for PRM 
    disp("here")
    waypoint_idx = 1;
    dist = 100;
    for i = 1:size(waypoints,1)
        distCurr = norm(initial_pose - waypoints(i));
        if  distCurr < dist
            dist = distCurr; 
            waypoint_idx = i; 
        end
    end
    disp(waypoints)
    disp(waypoint_idx);
    %perhaps change 
    testVisitWaypoints(Robot,waypoint_idx);

