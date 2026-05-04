function G = GjacDiffDrive(x, u)
% GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       u            2-by-1 vector [d, phi]'
%
%   OUTPUTS
%       G            Jacobian matrix partial(g)/partial(x)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Sasenarine, Etienne

theta = x(3);
d = u(1);
phi = u(2);

if phi == 0
    % straight-line motion
    G = [1, 0, -d * sin(theta);
        0, 1,  d * cos(theta);
        0, 0, 1];
else
    G = [1, 0, (d / phi) * (cos(theta + phi) - cos(theta));
        0, 1,  (d / phi) * (sin(theta + phi) - sin(theta));
        0, 0, 1];
end

end