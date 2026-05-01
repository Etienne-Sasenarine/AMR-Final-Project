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
%   Chalamalasetty, Shashank

% just to have it match notation in work
state = x;
x = state(1,1);
y = state(2,1);
th = state(3,1);

d = u(1,1);
phi = u(2,1);

%jacobian matrix with turning 
if phi ~= 0
G = [1 0 (d/phi)*(cos(th+phi)-cos(th));
    0 1 (d/phi)*(sin(th+phi)-sin(th));
    0 0 1];
else 
%jacobian matrix without turning 
G = [1 0 -d*sin(th); 0 1 d*cos(th); 0 0 1];
end

end