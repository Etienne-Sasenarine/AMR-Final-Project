function refined_pose = refineHeadingEKF(Robot, initial_pose, map, beaconLoc, sensorOrigin, angles)
% refineHeadingEKF: Two-phase heading lock-in for the case where position
% is GROUND-TRUTH-ACCURATE (snapped to a known waypoint) but heading from
% the particle filter may be off by tens of degrees.
%
%   PHASE 1 (Coarse search): With the robot stationary, try a grid of
%       ~24 candidate headings spanning 360 deg. For each, compute the
%       measurement residual (depth + beacons) given the known xy.
%       The minimum-residual candidate becomes the EKF seed. This kills
%       the "EKF locks onto a wrong heading basin" failure mode.
%
%   PHASE 2 (Fine 1D EKF): Run a Kalman filter over THETA ALONE (xy is
%       a known constant) while rotating slowly. Position cannot drift
%       because it isn't part of the state vector at all.
%
%   INPUTS
%       Robot         iRobot Create object
%       initial_pose  3-by-1 [x; y; theta] -- xy from snapped waypoint
%                     (treated as ground truth), theta possibly very wrong
%       map           N-by-4 mandatory walls
%       beaconLoc     k-by-3 beacons [tagID, x, y]
%       sensorOrigin  RealSense origin in robot frame [2-by-1]
%       angles        depth ray angles
%
%   OUTPUTS
%       refined_pose  3-by-1 [x_waypoint; y_waypoint; theta_locked]

    K = length(angles);
    fixed_xy = initial_pose(1:2);                % LOCKED constant

    % ====================================================================
    % CONFIG (try toggling these if heading is 90 deg off)
    % ====================================================================
    %   beacon_axes_swap : set true if RealSenseTag returns rows where
    %       (col4, col5) is (y_relative, x_relative) instead of (x_rel, y_rel).
    %       Swap fixes a 90 deg orientation offset in beacon predictions.
    %   beacon_weight    : how much to up-weight beacon residuals vs depth
    %       in the coarse heading search. Bumped to 50 from 5 because in
    %       rectangular maps depth alone has 90-deg symmetry (you can be
    %       facing four different walls and get nearly identical readings)
    %       so beacons MUST dominate when present.
    beacon_axes_swap = false;
    beacon_weight    = 50;

    % ====================================================================
    % PHASE 0: Settle and grab a fresh measurement snapshot
    % ====================================================================
    SetFwdVelAngVelCreate(Robot, 0, 0);
    ds = struct('odometry', [], 'rsdepth', [], 'beacon', []);
    noRobotCount = 0;
    for k = 1:5
        [noRobotCount, ds] = readStoreSensorData(Robot, noRobotCount, ds);
        pause(0.1);
    end

    % ====================================================================
    % PHASE 1: Coarse heading search
    % ====================================================================
    candidates = linspace(-pi, pi, 49);
    candidates(end) = [];                        % 48 unique values

    % Latest depth scan
    if ~isempty(ds.rsdepth)
        z_obs = ds.rsdepth(end, 2:end)';
    else
        z_obs = [];
    end

    % Recent beacons (deduplicated by tagID, most recent reading wins)
    bcn_obs = {};
    if ~isempty(ds.beacon)
        seen_ids = [];
        for bi = size(ds.beacon, 1):-1:1
            row = ds.beacon(bi, :);
            if numel(row) >= 5 && ~ismember(row(3), seen_ids)
                bcn_obs{end+1} = struct('tagID', row(3), ...
                                        'z_rel', [row(4); row(5)]);
                seen_ids(end+1) = row(3); %#ok<AGROW>
            end
        end
    end

    fprintf('Coarse heading search (%d candidates, %d beacons visible)...\n', ...
            length(candidates), length(bcn_obs));

    best_score = inf;
    best_theta = initial_pose(3);
    for c = candidates
        score   = 0;
        n_terms = 0;

        % Depth term
        if ~isempty(z_obs)
            z_hat = depthPredict([fixed_xy; c], map, sensorOrigin, angles);
            valid = ~isnan(z_obs) & ~isnan(z_hat) & ...
                    z_obs >= 0.1 & z_obs <= 4.0 & ...
                    z_hat >= 0.1 & z_hat <= 4.0;
            if any(valid)
                score   = score + sum((z_obs(valid) - z_hat(valid)).^2);
                n_terms = n_terms + sum(valid);
            end
        end

        % Beacon term (weighted higher -- beacons are anchored to known IDs)
        for bi = 1:length(bcn_obs)
            tagID = bcn_obs{bi}.tagID;
            z_rel = bcn_obs{bi}.z_rel;
            bRow  = beaconLoc(beaconLoc(:,1) == tagID, :);
            if isempty(bRow), continue; end
            bxy = bRow(1, 2:3)';
            z_hat_bcn = local_beaconPredict([fixed_xy; c], bxy, sensorOrigin);
            score   = score + beacon_weight * sum((z_rel - z_hat_bcn).^2);
            n_terms = n_terms + 2;
        end

        if n_terms > 0 && score / n_terms < best_score
            best_score = score / n_terms;
            best_theta = c;
        end
    end

    fprintf('Coarse best: theta = %.1f deg (mean sq residual %.4f)\n', ...
            best_theta * 180/pi, best_score);

    % ====================================================================
    % PHASE 2: Fine 1D EKF on theta (position is NOT part of the state)
    % ====================================================================
    theta    = best_theta;
    sigma_th = (30*pi/180)^2;                    % wider seed -- coarse can be 90 deg off

    R_th     = (3*pi/180)^2;                     % per-step process noise
    Q_dpth   = 0.0025 * eye(K);                  % trust depth heavily (matches nav EKF)
    Q_bcn    = diag([0.07^2, 0.07^2]);

    gate_threshold = 2.5;                        % wider gate so wrong-basin corrections pass
    bcn_gate       = 1.5;
    H_cap          = 50;
    min_valid      = 0.1;
    max_valid      = 4.0;

    turnW          = 0.25;
    wheel2center   = 0.13;
    maxRotation    = 3*pi/2;                     % at most a 3/4 turn
    minRotation    = pi/4;                       % at least 45 deg
    sigma_target   = (2*pi/180)^2;               % stop when std < 2 deg

    last_odom_idx   = size(ds.odometry, 1);
    last_depth_idx  = size(ds.rsdepth, 1);
    last_beacon_idx = size(ds.beacon, 1);
    totalRotated    = 0;

    while totalRotated < maxRotation && ...
          (sigma_th > sigma_target || totalRotated < minRotation)

        [cmdV, cmdW] = limitCmds(0, turnW, 0.2, wheel2center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

        [noRobotCount, ds] = readStoreSensorData(Robot, noRobotCount, ds);

        % --- predict (rotation only; xy unchanged because it's not state) ---
        cur_odom_idx = size(ds.odometry, 1);
        if cur_odom_idx > last_odom_idx
            new_rows  = ds.odometry(last_odom_idx+1:cur_odom_idx, :);
            phi_total = sum(new_rows(:, 3));
            theta     = atan2(sin(theta + phi_total), cos(theta + phi_total));
            sigma_th  = sigma_th + R_th;
            totalRotated  = totalRotated + abs(phi_total);
            last_odom_idx = cur_odom_idx;
        end

        mu_full = [fixed_xy; theta];

        % --- depth update (1D Kalman over theta only) ---
        if ~isempty(ds.rsdepth) && size(ds.rsdepth, 1) > last_depth_idx
            z_full     = ds.rsdepth(end, 2:end)';
            z_hat_full = depthPredict(mu_full, map, sensorOrigin, angles);
            H_full     = local_numDepthJac(mu_full, map, sensorOrigin, angles);
            H_full(abs(H_full) > H_cap) = 0;

            valid = ~isnan(z_full) & ~isnan(z_hat_full) & ...
                    z_full     >= min_valid & z_full     <= max_valid & ...
                    z_hat_full >= min_valid & z_hat_full <= max_valid & ...
                    abs(z_full - z_hat_full) < gate_threshold;

            if any(valid)
                z     = z_full(valid);
                z_hat = z_hat_full(valid);
                H_th  = H_full(valid, 3);                    % m x 1
                Q     = Q_dpth(valid, valid);

                S     = H_th * sigma_th * H_th' + Q;         % m x m
                Kgain = sigma_th * H_th' / S;                % 1 x m
                theta = theta + Kgain * (z - z_hat);         % scalar update
                sigma_th = (1 - Kgain * H_th) * sigma_th;    % scalar
                theta = atan2(sin(theta), cos(theta));
            end
            last_depth_idx = size(ds.rsdepth, 1);
        end

        % --- beacon update (1D Kalman over theta only) ---
        if ~isempty(ds.beacon) && size(ds.beacon, 1) > last_beacon_idx
            new_beacon_rows = ds.beacon(last_beacon_idx+1:size(ds.beacon,1), :);
            for bi = 1:size(new_beacon_rows, 1)
                row = new_beacon_rows(bi, :);
                if numel(row) < 5, continue; end
                tagID = row(3);
                z_bcn = [row(4); row(5)];

                bRow = beaconLoc(beaconLoc(:,1) == tagID, :);
                if isempty(bRow), continue; end
                bxy  = bRow(1, 2:3)';

                mu_full   = [fixed_xy; theta];
                z_hat_bcn = local_beaconPredict(mu_full, bxy, sensorOrigin);
                H_bcn     = local_beaconJac(mu_full, bxy, sensorOrigin);
                H_th_bcn  = H_bcn(:, 3);                     % 2 x 1

                innov = z_bcn - z_hat_bcn;
                if any(abs(innov) > bcn_gate), continue; end

                S     = H_th_bcn * sigma_th * H_th_bcn' + Q_bcn;
                Kgain = sigma_th * H_th_bcn' / S;            % 1 x 2
                theta = theta + Kgain * innov;
                sigma_th = (1 - Kgain * H_th_bcn) * sigma_th;
                theta = atan2(sin(theta), cos(theta));
            end
            last_beacon_idx = size(ds.beacon, 1);
        end

        pause(0.05);
    end

    SetFwdVelAngVelCreate(Robot, 0, 0);

    fprintf('Heading lock-in done. theta=%.1f deg, std=%.2f deg, rotated %.0f deg\n', ...
            theta * 180/pi, sqrt(sigma_th)*180/pi, totalRotated*180/pi);

    % Position is the snapped waypoint, EXACTLY -- never moved
    refined_pose = [fixed_xy; theta];
end


%% ----- local helpers (self-contained) -----
function H = local_numDepthJac(x, map, sensorOrigin, angles)
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
        z_plus(bad) = 0; z_minus(bad) = 0;
        H(:, i)   = (z_plus - z_minus) / (2 * dels(i));
        H(bad, i) = 0;
    end
end

function z = local_beaconPredict(mu, bxy, sensorOrigin)
    x = mu(1); y = mu(2); th = mu(3);
    sx = sensorOrigin(1); sy = sensorOrigin(2);
    sxw = x + sx*cos(th) - sy*sin(th);
    syw = y + sx*sin(th) + sy*cos(th);
    dxw = bxy(1) - sxw;
    dyw = bxy(2) - syw;
    z = [ cos(th)*dxw + sin(th)*dyw;
         -sin(th)*dxw + cos(th)*dyw];
end

function H = local_beaconJac(mu, bxy, sensorOrigin)
    th = mu(3);
    sx = sensorOrigin(1); sy = sensorOrigin(2);
    z  = local_beaconPredict(mu, bxy, sensorOrigin);
    xr = z(1); yr = z(2);
    H = [ -cos(th), -sin(th),   yr + sy ;
           sin(th), -cos(th), -(xr + sx)];
end
