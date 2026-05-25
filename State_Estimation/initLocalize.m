function [initial_pose, dataStore] = initLocalize(Robot, dataStore, waypoints, beacons, walls, R, Q, predict, update)
% initLocalize: Spins robot in place and uses particle filter to find
% initial starting location
%
%   INPUTS
%       Robot           Robot object
%       dataStore       dataStore struct
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
%       dataStore         dataStore struct

try 
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

SetFwdVelAngVelCreate(Robot, 0,0);

% define parameters
k = size(waypoints, 1);
noRobotCount = 0;
turnW = 0.4;
wheel2center = 0.13;
max_sensor_range = 3.0;

% find visibility map of tags for each waypoint
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

% full 360 degree rotation to find tags
seen_tags = [];
sweep_angle_turned = 0;
while sweep_angle_turned < 2.05 * pi 
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
            end
        end
    end
    pause(0.02);
end
SetFwdVelAngVelCreate(Robot, 0, 0);

% match tags to waypoints
if isempty(seen_tags)
    best_wp_indices = [];
    for i = 1:k
        if isempty(visibility_map{i})
            best_wp_indices = [best_wp_indices, i];
        end
    end
    
    % if all waypoints see tags saftey fallback
    if isempty(best_wp_indices)
        best_wp_indices = 1:k;
    end
    
else
    scores = zeros(k, 1);
    for i = 1:k
        expected_tags = visibility_map{i};
        scores(i) = length(intersect(seen_tags, expected_tags));
    end
    max_score = max(scores);
    best_wp_indices = find(scores == max_score);
end

% if only waypoint is matched
if isscalar(best_wp_indices)
    particles_per_wp = 200;
else
    particles_per_wp = 500;
end

% initalize particles
M = particles_per_wp * length(best_wp_indices);
particles = zeros(3, M);
idx = 1;
for i = 1:length(best_wp_indices)
    wp_idx = best_wp_indices(i);

    if isscalar(best_wp_indices)
        spread = 0.05; 
    else
        spread = 0.3;
    end
    
    particles(1, idx:idx+particles_per_wp-1) = waypoints(wp_idx, 1) + spread * randn(1, particles_per_wp);
    particles(2, idx:idx+particles_per_wp-1) = waypoints(wp_idx, 2) + spread * randn(1, particles_per_wp); 
    particles(3, idx:idx+particles_per_wp-1) = 2 * pi * rand(1, particles_per_wp);
    idx = idx + particles_per_wp;
end

% run particle filter
total_angle_turned = 0;
heading_threshold = 0.1;
required_particles = 0.65 * M;
max_wander_dist = 1;
is_global_search = length(best_wp_indices) > 1;

if is_global_search
    active_Q = Q * 15;
else
    active_Q = Q * 5;
end

while true
    [cmdV, cmdW] = limitCmds(0, turnW, 0.2, wheel2center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    odom = dataStore.odometry(end, :);
    u = [odom(2); odom(3)];
    total_angle_turned = total_angle_turned + abs(odom(3));
    
    if mod(size(dataStore.odometry, 1), 2) == 0
        depth = dataStore.rsdepth(end, :);
        z = depth(3:end)';
        [particles, ~] = PF(particles, u, z, R, active_Q, predict, update);
    end
    
    % find the minimum distance from each particle to any active waypoint
    min_dist_to_wps = inf(1, M);
    for idx_wp = 1:length(best_wp_indices)
        wp_x = waypoints(best_wp_indices(idx_wp), 1);
        wp_y = waypoints(best_wp_indices(idx_wp), 2);
        dist = sqrt((particles(1,:) - wp_x).^2 + (particles(2,:) - wp_y).^2);
        min_dist_to_wps = min(min_dist_to_wps, dist);
    end
    
    % check if particles went past max distance
    stray_mask = min_dist_to_wps > max_wander_dist;
    num_strays = sum(stray_mask);
    
    % respawn dead particles directly on top of the valid waypoints
    if num_strays > 0
        random_wps = best_wp_indices(randi(length(best_wp_indices), 1, num_strays));
        particles(1, stray_mask) = waypoints(random_wps, 1)' + 0.3 * randn(1, num_strays);
        particles(2, stray_mask) = waypoints(random_wps, 2)' + 0.3 * randn(1, num_strays);
        particles(3, stray_mask) = 2 * pi * rand(1, num_strays);
    end
    
    has_converged = false;
    for i = 1:length(best_wp_indices)
        wp_idx = best_wp_indices(i);
        dist_to_wp = sqrt((particles(1,:) - waypoints(wp_idx,1)).^2 + ...
                          (particles(2,:) - waypoints(wp_idx,2)).^2);
        
        cluster_mask = (dist_to_wp < 0.6);
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
    
    min_angle_required = pi/2;
    if is_global_search
        min_angle_required = pi; 
    end
    
    if has_converged && (total_angle_turned > min_angle_required) 
        init_x = waypoints(starting_idx, 1); 
        init_y = waypoints(starting_idx, 2);
        SetFwdVelAngVelCreate(Robot, 0, 0);
        break;
    end
    pause(0.02); 
end
initial_pose = [init_x; init_y; mean_theta];
end

% helper functions
function isVisible = hasLineOfSight(p1, p2, map)
    % checks if the line segment from p1 to p2 intersects any wall in the map.
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
    % true if line segment A-B intersects line segment C-D
    % uses cross product orientation method
    intersect = (ccw(A, C, D) ~= ccw(B, C, D)) && (ccw(A, B, C) ~= ccw(A, B, D));
end

function result = ccw(A, B, C)
    % counter-clockwise orientation check
    result = (C(2) - A(2)) * (B(1) - A(1)) > (B(2) - A(2)) * (C(1) - A(1));
end