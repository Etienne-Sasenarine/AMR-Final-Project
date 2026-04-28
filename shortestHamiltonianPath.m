function bestPath = shortestHamiltonianPath(costMatrix)
% shortestHamiltonianPath: Finds the optimal order to visit all waypoints
% by solving the Shortest Hamiltonian Path Problem
%
%   INPUTS
%       costMatrix      T-by-T matrix of shortest path distances
%
%   OUTPUTS
%       bestPath        optimal route to visit all waypoints

T = size(costMatrix, 1);

% only start node
if T <= 1
    bestPath = 1;
    return;
end

% generate all possible orderings of waypoints
waypoints = 2:T;
all_perms = perms(waypoints);
num_perms = size(all_perms, 1);

minCost = inf;
bestPath = [];

% brute force solve Shortest Hamiltonian Path problem
for i = 1:num_perms
    current_perm = all_perms(i, :);
    
    current_cost = 0;
    current_node = 1;
    is_valid = true;
    
    for j = 1:length(current_perm)
        next_node = current_perm(j);
        edge_cost = costMatrix(current_node, next_node);

        if edge_cost == inf
            is_valid = false;
            break; 
        end
        
        current_cost = current_cost + edge_cost;
        current_node = next_node;
    end
    
    % Check if route is valid and has cheaper cost
    if is_valid && (current_cost < minCost)
        minCost = current_cost;
        bestPath = [1, current_perm];
    end
end
end