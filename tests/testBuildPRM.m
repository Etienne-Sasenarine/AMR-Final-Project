% test_PRM.m
% Script to visually test the buildPRM function using the .mat map

clear; clc; close all;

% 1. Load Map Data
% Load the provided .mat practice map
mapData = load('C:\Users\etien\cs4758\AMR-Final-Project\practiceMap2026_4credit.mat');

% Extract variables dynamically
map = mapData.map;
optWalls = mapData.optWalls;
standard_waypoints = mapData.waypoints;

% Define Boundaries (Based on the text file header)
boundaries = [-3.048, 3.048, -2.286, 2.286];

% Define Start Pose (Dummy start pose for testing purposes)
% In the actual run, this will come from your initLocalize function
start_pose = [0, 0];

% Combine start pose and waypoints into our targets list
target_waypoints = [start_pose; standard_waypoints];

% Combine static map walls and optional walls for Phase 2 testing
all_walls = [map; optWalls];

% 2. Run PRM
numSamples = 1000;
robot_radius = 0.2; 

disp('Building PRM...');
tic;
roadmap = buildPRM(all_walls, boundaries, numSamples, robot_radius, target_waypoints);
toc;

disp(['Generated ', num2str(size(roadmap.nodes, 1)), ' nodes and ', num2str(size(roadmap.edges, 1)), ' edges.']);

% 3. Plotting
figure('Name', 'PRM Visualization', 'NumberTitle', 'off');
hold on; axis equal; grid on;
xlim([boundaries(1)-0.5, boundaries(2)+0.5]);
ylim([boundaries(3)-0.5, boundaries(4)+0.5]);
title('Probabilistic Roadmap (PRM)');

% A. Plot PRM Edges (Light gray)
for i = 1:size(roadmap.edges, 1)
    u = roadmap.edges(i, 1);
    v = roadmap.edges(i, 2);
    plot([roadmap.nodes(u, 1), roadmap.nodes(v, 1)], ...
         [roadmap.nodes(u, 2), roadmap.nodes(v, 2)], 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
end

% B. Plot PRM Nodes (Blue dots)
scatter(roadmap.nodes(:, 1), roadmap.nodes(:, 2), 10, 'b', 'filled');

% C. Plot All Walls (Black lines)
for i = 1:size(all_walls, 1)
    plot([all_walls(i, 1), all_walls(i, 3)], [all_walls(i, 2), all_walls(i, 4)], 'k', 'LineWidth', 2);
end

% D. Plot Target Waypoints (Red stars, larger)
scatter(target_waypoints(:, 1), target_waypoints(:, 2), 100, 'r', 'p', 'filled');

% Add a legend
legend('Edges', 'Nodes', 'Walls', 'Waypoints', 'Location', 'northeastoutside');
hold off;