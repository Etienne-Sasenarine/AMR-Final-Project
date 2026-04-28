% test_CostMatrix.m
% Script to test the buildCostMatrix function using the .mat map

clear; clc; close all;

% 1. Load Map Data
% Load the provided .mat practice map
mapData = load('C:\Users\etien\cs4758\AMR-Final-Project\practiceMap2026_4credit.mat');

% Extract variables
map = mapData.map;
optWalls = mapData.optWalls;
standard_waypoints = mapData.waypoints;

% Set boundaries to the inner maze walls [x_min, x_max, y_min, y_max]
boundaries = [-3.048, 3.048, -2.286, 2.286]; 

% Define a dummy start pose near the center
start_pose = [0, 0];

% Combine start pose and waypoints into our targets list
target_waypoints = [start_pose; standard_waypoints];
num_targets = size(target_waypoints, 1);

% Combine static map walls and optional walls
all_walls = [map; optWalls];

% 2. Build the PRM
numSamples = 500; 
robot_radius = 0.2; 

disp('Building PRM...');
tic;
roadmap = buildPRM(all_walls, boundaries, numSamples, robot_radius, target_waypoints);
toc;
disp(['Generated ', num2str(size(roadmap.nodes, 1)), ' nodes and ', num2str(size(roadmap.edges, 1)), ' edges.']);
disp(' ');

% 3. Calculate Dijkstra Cost Matrix
disp('Calculating Cost Matrix...');
tic;
% Because we injected our targets first, they are nodes 1 through num_targets
target_indices = 1:num_targets;
cost_matrix = buildCostMatrix(roadmap, target_indices);
toc;

% 4. Display Results
disp(' ');
disp('--- Final Cost Matrix ---');
disp('Row/Col 1: Start Pose');
disp('Row/Col 2-5: Standard Waypoints 1 to 4');
disp('-------------------------');
disp(cost_matrix);

% Check for unreachable waypoints
if any(cost_matrix(:) == inf)
    disp(' ');
    disp('WARNING: Some waypoints are unreachable (Cost = Inf).');
    disp('If all optional walls are present, they might be blocking a path!');
else
    disp(' ');
    disp('SUCCESS: All waypoints are reachable from the start pose!');
end