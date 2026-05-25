function [mu, Sigma] = EKF(mu_init, Sigma_init, u, z, g, Gjac, h, Hjac, R, Q)
% EKF: performs a prediction and update step of the Extended Kalman Filter
%
%   INPUTS
%       mu_init         previous pose estimation (3-by-1)            
%       Sigma_init      previous covariance matrix (3-by-3)
%       u               control input [d; phi] 
%       z               sensor measurements
%       g               non-linear dynamics function
%       Gjac            Jacobian of g function
%       h               non-linear measurement function
%       Hjac            Jacobian of h function
%       R               process noise
%       Q               measurement noise
%
%   OUTPUTS
%       mu              predicted current pose of robot (3-by-1)
%       Sigma           covariance matrix of predicted pose (3-by-3)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Sasenarine, Etienne

% prediction step
mu_bar = g(mu_init, u);
G = Gjac(mu_init, u);
Sigma_bar = G * Sigma_init * G' + R;

mu = mu_bar; 
Sigma = Sigma_bar;

% update step
if isempty(z) || all(isnan(z))
    return; 
end

exist_mask = ~isnan(z);
if any(exist_mask)
    expected_z = h(mu_bar);
 
    raw_diff = z(exist_mask) - expected_z(exist_mask);
    thresh = 0.15;
    acceptable_mask = abs(raw_diff) < thresh;
    
    if any(acceptable_mask)
        final_diff = raw_diff(acceptable_mask);
        H_full = Hjac(mu_bar);

        H_exist = H_full(exist_mask, :);
        H_final = H_exist(acceptable_mask, :);
        
        Q_exist = Q(exist_mask, exist_mask);
        Q_final = Q_exist(acceptable_mask, acceptable_mask);
        
        K = Sigma_bar * H_final' / (H_final * Sigma_bar * H_final' + Q_final);
        
        mu = mu_bar + K * final_diff(:);
        Sigma = (eye(size(Sigma_bar)) - K * H_final) * Sigma_bar;
    end
end

end