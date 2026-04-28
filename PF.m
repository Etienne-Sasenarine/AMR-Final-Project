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

if isempty(z)
    particles = predicted_particles;
    weights = ones(1, M) / M;
    return;
end

% update step
weights = zeros(1, M);
for i = 1:M
    expected_z = update(predicted_particles(:, i));

    if all(isnan(expected_z))
        weights(i) = -inf;
        continue;
    end

    logWeight = 0;
    for j = 1:k
        if ~isnan(z(j)) && ~isnan(expected_z(j))
            logWeight = logWeight + (-0.5 * (z(j) - expected_z(j))^2 / Q(j,j) ...
                - 0.5 * log(2*pi*Q(j,j)));
        end
    end
    weights(i) = logWeight;
end

% convert from log weights
weights = weights - max(weights);
weights = exp(weights);

% normalize weights
weights = weights + 1e-300;
weights = weights / sum(weights);

% resampling step
particles = zeros(3, M);
cdf = cumsum(weights);
r = rand() / M;
for m = 1:M
    j = 1;
    target = r + (m - 1) / M;
    while j < M && cdf(j) < target
        j = j + 1;
    end
    particles(:, m) = predicted_particles(:, j);
end

end