function [mu_next, sigma_next] = EKF(mu, ut, sigma, R, z, Q, dynamics, dynamicsJac, measure, measureJac)
% EKF: Performs one step of the extended Kalman Filter
%
%   INPUTS
%       mu           previous vector of pose state
%       ut           current command [d; phi]
%       sigma        previous covariance matrix
%       R            state model noise covariance matrix
%       z            current measurement vector
%       Q            measurement noise covariance matrix
%       dynamics     function handle for dynamics model g(x,u)
%       dynamicsJac  function handle for dynamics jacobian G(x,u)
%       measure      function handle for measurement model h(x)
%       measureJac   function handle for measurement jacobian H(x)
%
%   OUTPUTS
%       mu_next      current estimate of vector of pose state
%       sigma_next   current covariance matrix
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4

    % prediction
    mu_bar = dynamics(mu, ut);
    Gt = dynamicsJac(mu, ut);
    sigma_bar = Gt * sigma * Gt' + R;

    % update
    z_hat = measure(mu_bar);
    Ht = measureJac(mu_bar);

    K = sigma_bar * Ht' / (Ht * sigma_bar * Ht' + Q);

    mu_next = mu_bar + K * (z - z_hat);
    sigma_next = (eye(size(sigma_bar)) - K * Ht) * sigma_bar;

    % wrap heading
    mu_next(3) = atan2(sin(mu_next(3)), cos(mu_next(3)));

end