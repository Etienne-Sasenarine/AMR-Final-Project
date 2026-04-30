function initial_pose = testInitLocalize(Robot)
% runInitLocalize: Wrapper function to execute the initial localization
% sequence using a specific practice map and predefined noise matrices.
%
%   INPUTS
%       Robot           Robot object
%
%   OUTPUTS
%       initial_pose    3-by-1 state vector [x; y; theta] of the start pose

% 1. Load the map data
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
    R = diag([0.0001, 0.0001, 0.001]);
    Q = 2 * eye(num_depth_sensors); % Now correctly matches the size of z

    % 5. Define Function Handles
    predict_func = @integrateOdom;
    update_func = @(pose) depthPredict(pose, map, sensorOrigin, angles);

    % 6. Execute Localization
    disp('Starting initial localization sequence...');
    initial_pose = initLocalize(Robot, waypoints, beacons, R, Q, predict_func, update_func);
    
    disp('Localization complete. Final Pose:');
    disp(initial_pose);