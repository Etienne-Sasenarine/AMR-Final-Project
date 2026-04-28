function initial_pose = initLocalize(Robot, waypoints, beacons, R, Q, predict, update)
% initLocalize: Spins robot in place and uses particle filter to find
% initial starting location
%
%   INPUTS
%       Robot           Robot object
%       waypoints       k-by-2 matrix of possible starting locations
%       beacons         b-by-2 matrix of beacon locations [tag, x, y]
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

% initalize particles
k = size(waypoints, 1);
particles_per_waypoint = 500; % tune for how many particles per waypoint
M = particles_per_waypoint * k;
particles = zeros(3, M);
% tight clusters of particles around each waypoint with uniform random orientation
idx = 1;
for i = 1:k
    particles(1, idx:idx+particles_per_waypoint-1) = waypoints(i, 1) + 0.1 * randn(1, particles_per_waypoint);
    particles(2, idx:idx+particles_per_waypoint-1) = waypoints(i, 2) + 0.1 * randn(1, particles_per_waypoint); 
    particles(3, idx:idx+particles_per_waypoint-1) = 2 * pi * rand(1, particles_per_waypoint);
    idx = idx + particles_per_waypoint;
end

% initalize spin
scan_speed = 0.4;
finish_speed = 1;
wheel2center = 0.13;
total_angle_spun = 0;
target_angle = 2 * pi;
isConverged = false;
variance_threshold = 0.05;
dataStore = struct('odometry', [], ...
                   'rsdepth', [], ...
                   'beacon', []);
noRobotCount = 0;

init_x = 0; 
init_y = 0;

% run filter
while true
    if isConverged
        turnW = finish_speed;
    else
        turnW = scan_speed;
    end

    [cmdV, cmdW] = limitCmds(0, turnW, 0.2, wheel2center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

    % read sensor data
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    % extract control input
    odom = dataStore.odometry(end, :);
    d = odom(2);
    phi = odom(3);
    u = [d; phi];
    total_angle_spun = total_angle_spun + abs(phi);

    % extract sensor measurements
    depth = dataStore.rsdepth(end, :);
    current_time = depth(1);
    z = depth(2:end)';

    % check if any beacons found
    if ~isempty(dataStore.beacon) && ~isConverged
        recent_beacons = dataStore.beacon(dataStore.beacon(:,1) == current_time, :);
        if ~isempty(recent_beacons)
            seen_id = recent_beacons(1, 2); 
            beacon_map_idx = find(beacons(:, 1) == seen_id, 1);
            
            if ~isempty(beacon_map_idx)
                global_bx = beacons(beacon_map_idx, 2);
                global_by = beacons(beacon_map_idx, 3);
                
                % find closest waypoint
                min_dist = inf;
                starting_idx = -1;
                for i = 1:k
                    dist = sqrt((waypoints(i,1) - global_bx)^2 + (waypoints(i,2) - global_by)^2);
                    if dist < min_dist
                        min_dist = dist;
                        starting_idx = i;
                    end
                end
                
                init_x = waypoints(starting_idx, 1);
                init_y = waypoints(starting_idx, 2);

                disp(['Beacon ', num2str(seen_id), ' detected! Snapping to waypoint ', num2str(starting_idx)]);
                isConverged = true;
            end
        end
    end

    % run particle filter
    if ~isConverged
        [particles, ~] = PF(particles, u, z, R, Q, predict, update);
    
        % check if particle filter converged
        var_x = var(particles(1, :));
        var_y = var(particles(2, :));
        if (var_x < variance_threshold) && (var_y < variance_threshold)
            isConverged = true;
            mean_x = mean(particles(1, :));
            mean_y = mean(particles(2, :));
            
            % find closest waypoint
            min_dist = inf;
            starting_idx = -1;
            for i = 1:k
                dist = sqrt((waypoints(i,1) - mean_x)^2 + (waypoints(i,2) - mean_y)^2);
                if dist < min_dist
                    min_dist = dist;
                    starting_idx = i;
                end
            end
            
            % set inital location to exact waypoint
            init_x = waypoints(starting_idx, 1);
            init_y = waypoints(starting_idx, 2);
        end
    end

    % check if rotation complete
    if total_angle_spun >= target_angle
        if isConverged
            SetFwdVelAngVelCreate(Robot, 0, 0);
            mean_theta = atan2(mean(sin(particles(3, :))), mean(cos(particles(3, :))));
            break;
        else
            % spin another full rotation
            target_angle = target_angle + 2 * pi;
        end
    end
    pause(0.05); 
end

initial_pose = [init_x; init_y; mean_theta];

end