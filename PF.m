function [particles, weights] = PF(particles_init, u, z, R, Q, predict, update)
% PF: performs a prediction, update and resampling step of the Particle Filter
%
%   INPUTS
%       particles_init  previous set of particles (3-by-M)            
%       u               control input [d; phi] 
%       z               sensor measurements
%       R               process noise (3-by-3)
%       Q               measurement noise (k-by-k)
%       predict         prediction step function
%       update          update step function
%
%   OUTPUTS
%       particles       current set of particles (3-by-M)
%       weights         normalized weights (1-by-M)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 5
%   Sasenarine, Etienne

M = size(particles_init, 2);
k = length(z);

% prediction step
predicted_particles = zeros(3, M);
for i = 1:M
    mu_bar = predict(particles_init(:, i), u(1), u(2));
    predicted_particles(:, i) = mu_bar + randn(3,1) .* sqrt(diag(R));
end

if isempty(z) || all(isnan(z))
    particles = predicted_particles;
    weights = ones(1, M) / M;
    return;
end

% update step
weights = zeros(1, M);
max_rs_range = 4.0;

for i = 1:M
    expected_z = update(predicted_particles(:, i));
    
    if all(isnan(expected_z))
        weights(i) = -1000;
        continue;
    end
    
    logWeight = 0;
    for j = 1:k
        val_z = z(j);
        val_exp = expected_z(j);
        
        if isnan(val_z)
            if val_exp > max_rs_range || isnan(val_exp)
                logWeight = logWeight + 0; 
            else
                logWeight = logWeight - 50; 
            end
            
        else
            if isnan(val_exp) || val_exp > max_rs_range
                logWeight = logWeight - 50;
            else
                logWeight = logWeight + (-0.5 * (val_z - val_exp)^2 / Q(j,j) ...
                    - 0.5 * log(2*pi*Q(j,j)));
            end
        end
    end
    weights(i) = logWeight;
end

max_w = max(weights);

if max_w == -1000 || all(weights == -inf)
    weights = ones(1, M) / M;
else
    % convert from log weights
    weights = weights - max_w;
    weights = exp(weights);
    
    % normalize weights
    weights = weights + 1e-300;
    weights = weights / sum(weights);
end

% resampling step
particles = zeros(3, M);
r = rand() / M;
c = weights(1);
i = 1;

for m = 1:M
    U = r + (m - 1) / M;
    while U > c && i < M
        i = i + 1;
        c = c + weights(i);
    end
    particles(:, m) = predicted_particles(:, i);
end

weights = ones(1, M) / M;

end