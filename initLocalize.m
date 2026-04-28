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
turnW = 0.4;
wheel2center = 0.13;
variance_threshold = 0.05;
dataStore = struct('odometry', [], ...
                   'rsdepth', [], ...
                   'beacons', []);
noRobotCount = 0;

% run filter
while true
    [cmdV, cmdW] = limitCmds(0, turnW, 0.2, wheel2center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

    % read sensor data
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    % extract control input
    odom = dataStore.odometry(end, :);
    d = odom(2);
    phi = odom(3);
    u = [d; phi];

    % extract sensor measurements
    depth = dataStore.rsdepth(end, :);
    z = depth(2:end)';

    % check if beacon detected
    if ~isempty(dataStore.beacon)
        recent_beacons = dataStore.beacon(end, :);
        if ~isempty(recent_beacons)
            seen_id = recent_beacons(1, 2); 
            beacon_map_idx = find(beacons(:, 1) == seen_id, 1);
            
            if ~isempty(beacon_map_idx)
                global_bx = beacons(beacon_map_idx, 2);
                global_by = beacons(beacon_map_idx, 3);
                
                % find closest waypoint to the beacon
                min_dist = inf;
                starting_idx = -1;
                for i = 1:k
                    dist = sqrt((waypoints(i,1) - global_bx)^2 + (waypoints(i,2) - global_by)^2);
                    if dist < min_dist
                        min_dist = dist;
                        starting_idx = i;
                    end
                end
                
                % look for particles within 0.5 meters of the correct waypoint
                dist_to_target = sqrt((particles(1,:) - waypoints(starting_idx,1)).^2 + ...
                                      (particles(2,:) - waypoints(starting_idx,2)).^2);
                
                correct_particles = particles(:, dist_to_target < 0.5); 
                
                if ~isempty(correct_particles)
                    % resample the correct particles to fill back up to M
                    resample_idx = randi(size(correct_particles, 2), 1, M);
                    particles = correct_particles(:, resample_idx);
                    
                    disp(['Beacon ', num2str(seen_id), ' detected! Collapsed particles to waypoint ', num2str(starting_idx)]);
                end
                
                dataStore.beacon = []; 
            end
        end
    end

    % run particle filter
    [particles, ~] = PF(particles, u, z, R, Q, predict, update);

    % check if particle filter converged
    var_x = var(particles(1, :));
    var_y = var(particles(2, :));
    if (var_x < variance_threshold) && (var_y < variance_threshold)
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

        SetFwdVelAngVelCreate(Robot, 0, 0);
        mean_theta = atan2(mean(sin(particles(3, :))), mean(cos(particles(3, :))));
        break;
    end

    pause(0.05); 
end

initial_pose = [init_x; init_y; mean_theta];

end