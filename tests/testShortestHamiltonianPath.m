% test_ShortestHamiltonianPath.m
% Script to test the shortestHamiltonianPath function using the .mat map

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

% 4. Solve Shortest Hamiltonian Path
disp('Solving Shortest Hamiltonian Path...');
tic;
optimal_order = shortestHamiltonianPath(cost_matrix);
toc;

disp(' ');
if isempty(optimal_order)
    disp('ERROR: No valid path exists to visit all waypoints!');
else
    % Format the output string to clearly show the order
    route_str = 'Start (Node 1)';
    for i = 2:length(optimal_order)
        route_str = [route_str, ' -> Waypoint ', num2str(optimal_order(i)-1), ' (Node ', num2str(optimal_order(i)), ')'];
    end
    disp(['Optimal Order: ', route_str]);
end

% 5. Graphical Overlay of the Optimal Path
if ~isempty(optimal_order)
    figure('Name', 'Shortest Hamiltonian Path Visualization', 'NumberTitle', 'off');
    hold on; axis equal; grid on;
    
    % Set map bounds
    xlim([boundaries(1)-0.5, boundaries(2)+0.5]);
    ylim([boundaries(3)-0.5, boundaries(4)+0.5]);
    title('Shortest Hamiltonian Path (Waypoint Tour)');

    % A. Plot PRM Edges (very faint gray for context)
    for i = 1:size(roadmap.edges, 1)
        u = roadmap.edges(i, 1);
        v = roadmap.edges(i, 2);
        plot([roadmap.nodes(u, 1), roadmap.nodes(v, 1)], ...
             [roadmap.nodes(u, 2), roadmap.nodes(v, 2)], 'Color', [0.9 0.9 0.9], 'LineWidth', 0.5);
    end

    % B. Plot All Walls (Black lines)
    for i = 1:size(all_walls, 1)
        plot([all_walls(i, 1), all_walls(i, 3)], [all_walls(i, 2), all_walls(i, 4)], 'k', 'LineWidth', 2);
    end

    % C. Plot the High-Level TSP Tour (Thick magenta dashed line)
    tour_coords = target_waypoints(optimal_order, :);
    plot(tour_coords(:, 1), tour_coords(:, 2), 'm--', 'LineWidth', 2.5);

    % D. Plot Start Pose and Waypoints
    scatter(target_waypoints(1, 1), target_waypoints(1, 2), 200, 'g', 'p', 'filled'); % Start = Green Star
    scatter(target_waypoints(2:end, 1), target_waypoints(2:end, 2), 100, 'r', 'o', 'filled'); % Waypoints = Red Circles

    % E. Add Text Labels for the Visitation Order
    for i = 1:length(optimal_order)
        node_idx = optimal_order(i);
        x = target_waypoints(node_idx, 1);
        y = target_waypoints(node_idx, 2);
        
        if i == 1
            text(x + 0.15, y + 0.15, 'START', 'Color', [0 0.5 0], 'FontWeight', 'bold');
        else
            text(x + 0.15, y + 0.15, sprintf('Stop %d', i-1), 'Color', 'm', 'FontWeight', 'bold', 'FontSize', 11);
        end
    end

    % Add a legend
    % Dummy plots to ensure the legend grabs the right colors/styles
    plot(NaN, NaN, 'Color', [0.9 0.9 0.9], 'LineWidth', 1);
    plot(NaN, NaN, 'k', 'LineWidth', 2);
    plot(NaN, NaN, 'm--', 'LineWidth', 2.5);
    scatter(NaN, NaN, 200, 'g', 'p', 'filled');
    scatter(NaN, NaN, 100, 'r', 'o', 'filled');
    
    legend('PRM Edges', 'Walls', 'TSP Tour Order', 'Start Pose', 'Waypoints', 'Location', 'northeastoutside');
    hold off;
end