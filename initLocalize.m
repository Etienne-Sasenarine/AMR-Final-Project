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
particles_per_waypoint = 1000; % tune for how many particles per waypoint
M = particles_per_waypoint * k;
particles = zeros(3, M);
% tight clusters of particles around each waypoint with uniform random orientation
idx = 1;
for i = 1:k
    particles(1, idx:idx+particles_per_waypoint-1) = waypoints(i, 1) + 0.4 * randn(1, particles_per_waypoint);
    particles(2, idx:idx+particles_per_waypoint-1) = waypoints(i, 2) + 0.4 * randn(1, particles_per_waypoint); 
    particles(3, idx:idx+particles_per_waypoint-1) = 2 * pi * rand(1, particles_per_waypoint);
    idx = idx + particles_per_waypoint;
end

% initalize spin
turnW = 0.2;
wheel2center = 0.13;
heading_threshold = 0.05;
dataStore = struct('odometry', [], ...
                   'rsdepth', [], ...
                   'beacon', []);
noRobotCount = 0;
total_angle_turned = 0;
frame_counter = 0;
resample_interval = 20;

% =========================================================================
% --- VISUALIZATION SETUP ---
% =========================================================================
figure(1); clf; hold on; grid on; axis equal;
title('Particle Filter Initialization');
xlabel('X (m)'); ylabel('Y (m)');

% Plot Waypoints as blue pentagrams
plot(waypoints(:,1), waypoints(:,2), 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Waypoints');

% Plot Beacons as red squares
if ~isempty(beacons)
    plot(beacons(:,2), beacons(:,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Beacons');
end

% Plot initial Particles as slightly larger RED dots
h_particles = plot(particles(1,:), particles(2,:), 'r.', 'MarkerSize', 5, 'DisplayName', 'Particles');
legend('Location', 'best');

% --- NEW CODE: Freeze the axis limits ---
% Calculate the boundaries of your map based on the waypoints
min_x = min(waypoints(:,1)) - 2;
max_x = max(waypoints(:,1)) + 2;
min_y = min(waypoints(:,2)) - 2;
max_y = max(waypoints(:,2)) + 2;

% Force MATLAB to lock the zoom to this bounding box
axis([min_x max_x min_y max_y]);
axis manual;
% =========================================================================

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
    total_angle_turned = total_angle_turned + abs(phi);

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
                
                dist_to_beacon = sqrt((particles(1,:) - global_bx).^2 + (particles(2,:) - global_by).^2);

                correct_particles = particles(:, dist_to_beacon < 3.0); 
                
                if ~isempty(correct_particles)
                    % resample the correct particles to fill back up to M
                    resample_idx = randi(size(correct_particles, 2), 1, M);
                    particles = correct_particles(:, resample_idx);
                    
                    disp(['Beacon ', num2str(seen_id), ' detected! Filtered out particles far from beacon.']);
                end
                
                dataStore.beacon = [];
                                
            end
        end
    end

    % run particle filter in intervals
    frame_counter = frame_counter + 1;
    if mod(frame_counter, resample_interval) == 0
        [particles, ~] = PF(particles, u, z, R, Q, predict, update);
    else
        for i = 1:M
             particles(:, i) = predict(particles(:, i), u(1), u(2)) + randn(3,1) .* sqrt(diag(R));
        end
    end


    % =====================================================================
    % --- VISUALIZATION UPDATE ---
    % =====================================================================
    % Update the x and y data of the particle plot handle
    set(h_particles, 'XData', particles(1,:), 'YData', particles(2,:));
    drawnow limitrate; % Forces MATLAB to update the figure window
    % ====================================================================

    % check if particle filter converged
    required_particles = 0.90 * M; 
    has_converged = false;
    
    for i = 1:k
        % calculate distance from all particles to this waypoint
        dist_to_wp = sqrt((particles(1,:) - waypoints(i,1)).^2 + ...
                          (particles(2,:) - waypoints(i,2)).^2);
        
        % find particles near this waypoint
        cluster_mask = (dist_to_wp < 0.5);
        num_in_cluster = sum(cluster_mask);
        
        if num_in_cluster > required_particles
            cluster_particles = particles(:, cluster_mask);
            R_theta = sqrt(mean(cos(cluster_particles(3,:)))^2 + mean(sin(cluster_particles(3,:)))^2);
            var_theta = 1 - R_theta;
            
            if var_theta < heading_threshold
                has_converged = true;
                starting_idx = i;
                mean_theta = atan2(mean(sin(cluster_particles(3, :))), mean(cos(cluster_particles(3, :))));
                break;
            end
        end
    end
    
    if has_converged && (total_angle_turned > 2 * pi)
        % set initial location to waypoint
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