function [cmdV, cmdW, prmIdx, unvisitedWaypoints] = visitWaypoints(x, y, theta, prmPath, prmIdx, unvisitedWaypoints, epsilon)
% visitWaypoints: Drives robot along a PRM path while checking for scoring waypoints
% 
%   INPUTS
%       x, y, theta         robot's current pose
%       prmPath             (x,y) coordinates of prm path
%       prmIdx              current index of prm node being tracked
%       unvisitedWaypoints  unvisited waypoints
%       epsilon             turn radius
% 
%   OUTPUTS
%       cmdV, cmdW          forward and angular velocity
%       prmIdx              updated index of prm node being tracked
%       unvisitedWaypoints  updated unvisited waypoints

% initalize fwd and ang velocity to 0
cmdV = 0;
cmdW = 0;

% check if within 0.1m of any waypoint
for i = 1:size(unvisitedWaypoints, 1)
    target_x = unvisitedWaypoints(i, 1);
    target_y = unvisitedWaypoints(i, 2);
    
    % calculate Euclidean distance to waypoint
    dist_to_target = sqrt((x - target_x)^2 + (y - target_y)^2);
    
    if dist_to_target <= 0.1
        disp(['WAYPOINT REACHED at (', num2str(target_x), ', ', num2str(target_y), ')!']);
        % remove waypoint and beep
        unvisitedWaypoints(i, :) = []; 
        % beepCreate(Robot) <- TODO add this
        return; 
    end
end

% return if reached end of prmPath
if prmIdx > size(prmPath, 1)
    return; 
end

% navigate to target node
wx = prmPath(prmIdx, 1);
wy = prmPath(prmIdx, 2);

ex = wx - x;
ey = wy - y;
distance_to_prm_node = sqrt(ex^2 + ey^2);

% go to next node if less than 0.1m away
if distance_to_prm_node < 0.1
    prmIdx = prmIdx + 1;
else
    % Cap the error vector to maxV before feedbackLin.
    % Without capping, a 1 m error with epsilon=0.15 produces cmdW ~ 6 rad/s,
    % which after wheel saturation still spins at ~1.5 rad/s.  Large omega
    % accumulates odometry heading error and fights EKF localization.
    maxFeedVel = 0.2;
    err_scale  = min(1, maxFeedVel / distance_to_prm_node);
    [cmdV, cmdW] = feedbackLin(ex * err_scale, ey * err_scale, theta, epsilon);
end

end