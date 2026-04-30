function initial_pose = initLocalize(Robot, waypoints, beacons, walls, R, Q, predict, update)
% initLocalize: Spins robot in place and uses particle filter to find
% initial starting location
%
%   INPUTS
%       Robot           Robot object
%       waypoints       k-by-2 matrix of possible starting locations
%       beacons         b-by-2 matrix of beacon locations [tag, x, y]
%       map             N-by-4 matrix of map walls [x1, y1, x2, y2]
%       R               process noise (3-by-3)
%       Q               measurement noise (k-by-k)
%       predict         prediction step function
%       update          update step function
%
%   OUTPUTS
%       initial_pose      3-by-1 state vector [x; y; theta] of the start pose

try 
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

SetFwdVelAngVelCreate(Robot, 0,0);

% define parameters
k = size(waypoints, 1);
dataStore = struct('odometry', [], 'rsdepth', [], 'beacon', []);
noRobotCount = 0;
turnW = 0.2;
wheel2center = 0.13;
max_sensor_range = 3.0;

% find visibility map of tags for each waypoint
disp('Computing Visibility Map with Line-of-Sight...');
visibility_map = cell(k, 1); 

for i = 1:k
    wp_x = waypoints(i, 1);
    wp_y = waypoints(i, 2);
    visible_tags = [];
    
    for j = 1:size(beacons, 1)
        bx = beacons(j, 2);
        by = beacons(j, 3);
        tag_id = beacons(j, 1);
        
        dist = sqrt((bx - wp_x)^2 + (by - wp_y)^2);
        if dist < max_sensor_range
            % Check if walls block the view
            if hasLineOfSight([wp_x, wp_y], [bx, by], walls)
                visible_tags = [visible_tags, tag_id];
            end
        end
    end
    visibility_map{i} = visible_tags;
end
disp('Visibility map computed!');

% full 360 degree rotation to find tags
disp('Starting 360-degree sweep to find tags...');
seen_tags = [];
sweep_angle_turned = 0;

while sweep_angle_turned < 2.1 * pi
    [cmdV, cmdW] = limitCmds(0, turnW, 0.2, wheel2center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    odom = dataStore.odometry(end, :);
    sweep_angle_turned = sweep_angle_turned + abs(odom(3));
    
    if ~isempty(dataStore.beacon)
        recent_beacons = dataStore.beacon(end, :);
        if ~isempty(recent_beacons)
            seen_id = recent_beacons(1, 3); 
            if ~ismember(seen_id, seen_tags)
                seen_tags = [seen_tags, seen_id];
                disp(['Spotted Tag: ', num2str(seen_id)]);
            end
        end
    end
    pause(0.05);
end
SetFwdVelAngVelCreate(Robot, 0, 0);

% match tags to waypoints
scores = zeros(k, 1);
for i = 1:k
    expected_tags = visibility_map{i};
    scores(i) = length(intersect(seen_tags, expected_tags));
end

max_score = max(scores);
best_wp_indices = find(scores == max_score);

% initalize everywhere if no tags seen
if max_score == 0 || isempty(seen_tags)
    disp('No tags matched or seen! Falling back to all waypoints.');
    best_wp_indices = 1:k; 
else
    disp(['Matching complete. Initializing at waypoint(s): ', num2str(best_wp_indices')]);
end

% initalize particles
particles_per_wp = 500;
M = particles_per_wp * length(best_wp_indices);
particles = zeros(3, M);
idx = 1;

for i = 1:length(best_wp_indices)
    wp_idx = best_wp_indices(i);
    particles(1, idx:idx+particles_per_wp-1) = waypoints(wp_idx, 1) + 0.4 * randn(1, particles_per_wp);
    particles(2, idx:idx+particles_per_wp-1) = waypoints(wp_idx, 2) + 0.4 * randn(1, particles_per_wp); 
    particles(3, idx:idx+particles_per_wp-1) = 2 * pi * rand(1, particles_per_wp);
    idx = idx + particles_per_wp;
end

% --- VISUALIZATION SETUP ---
figure(1); clf; hold on; grid on; axis equal;
title('Particle Filter Tracking'); xlabel('X (m)'); ylabel('Y (m)');
plot(waypoints(:,1), waypoints(:,2), 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
if ~isempty(beacons)
    plot(beacons(:,2), beacons(:,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end
h_particles = plot(particles(1,:), particles(2,:), 'r.', 'MarkerSize', 5);

min_x = min(waypoints(:,1)) - 2; max_x = max(waypoints(:,1)) + 2;
min_y = min(waypoints(:,2)) - 2; max_y = max(waypoints(:,2)) + 2;
axis([min_x max_x min_y max_y]); axis manual;

% run particle filter
disp('Dialing in exact pose with Particle Filter...');
total_angle_turned = 0;
heading_threshold = 0.05;
required_particles = 0.75 * M;
frame_counter = 0;
resample_interval = 15;

is_global_search = length(best_wp_indices) > 1;
if is_global_search
    disp('Global Search Mode: Relaxing measurement noise to prevent false convergence.');
    active_Q = Q * 40;
else
    active_Q = Q;
end

while true
    [cmdV, cmdW] = limitCmds(0, turnW, 0.2, wheel2center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    odom = dataStore.odometry(end, :);
    u = [odom(2); odom(3)];
    total_angle_turned = total_angle_turned + abs(odom(3));
    
    depth = dataStore.rsdepth(end, :);
    z = depth(2:end)';
    
    frame_counter = frame_counter + 1;
    if mod(frame_counter, resample_interval) == 0
        [particles, ~] = PF(particles, u, z, R, active_Q, predict, update);
    else
        for i = 1:M
             particles(:, i) = predict(particles(:, i), u(1), u(2)) + randn(3,1) .* sqrt(diag(R));
        end
    end
    
    set(h_particles, 'XData', particles(1,:), 'YData', particles(2,:));
    drawnow limitrate;
    
    has_converged = false;
    for i = 1:length(best_wp_indices)
        wp_idx = best_wp_indices(i);
        dist_to_wp = sqrt((particles(1,:) - waypoints(wp_idx,1)).^2 + ...
                          (particles(2,:) - waypoints(wp_idx,2)).^2);
        
        cluster_mask = (dist_to_wp < 0.5);
        if sum(cluster_mask) > required_particles
            cluster_particles = particles(:, cluster_mask);
            R_theta = sqrt(mean(cos(cluster_particles(3,:)))^2 + mean(sin(cluster_particles(3,:)))^2);
            
            if (1 - R_theta) < heading_threshold
                has_converged = true;
                starting_idx = wp_idx;
                mean_theta = atan2(mean(sin(cluster_particles(3, :))), mean(cos(cluster_particles(3, :))));
                break;
            end
        end
    end
    
    if has_converged && (total_angle_turned > pi) 
        init_x = waypoints(starting_idx, 1); 
        init_y = waypoints(starting_idx, 2);
        SetFwdVelAngVelCreate(Robot, 0, 0);
        disp(['Converged! Starting at waypoint ', num2str(starting_idx)]);
        break;
    end
    pause(0.05); 
end

initial_pose = [init_x; init_y; mean_theta];
end

% helper functions
function isVisible = hasLineOfSight(p1, p2, map)
    % Checks if the line segment from p1 to p2 intersects any wall in the map.
    % map is an N-by-4 matrix: [x1, y1, x2, y2]
    
    isVisible = true;
    for i = 1:size(map, 1)
        wall_p1 = [map(i, 1), map(i, 2)];
        wall_p2 = [map(i, 3), map(i, 4)];
        
        if segmentsIntersect(p1, p2, wall_p1, wall_p2)
            isVisible = false;
            return;
        end
    end
end

function intersect = segmentsIntersect(A, B, C, D)
    % True if line segment A-B intersects line segment C-D
    % Uses cross product orientation method
    intersect = (ccw(A, C, D) ~= ccw(B, C, D)) && (ccw(A, B, C) ~= ccw(A, B, D));
end

function result = ccw(A, B, C)
    % Counter-clockwise orientation check
    result = (C(2) - A(2)) * (B(1) - A(1)) > (B(2) - A(2)) * (C(1) - A(1));
end
