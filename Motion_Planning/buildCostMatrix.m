function [costMatrix, parentMatrix] = buildCostMatrix(roadmap, waypoints)
% buildCostMatrix: Runs Dijkstra's algorithm to compute the shortest
% distance between all pairs of waypoints in a PRM
%
%   INPUTS
%       roadmap         struct of nodes and edges
%       waypoints       list of waypoint indices
%
%   OUTPUTS
%       costMatrix     T-by-T matrix of shortest path distances
%       parentMatrix   T-by-N matrix of parent nodes from waypoint(i)

% define variables
N = size(roadmap.nodes, 1);
T = length(waypoints);
costMatrix = zeros(T, T);
parentMatrix = zeros(T, N);

% build adjacency list
adj = cell(N, 1);
for e = 1:size(roadmap.edges, 1)
    u = roadmap.edges(e, 1);
    v = roadmap.edges(e, 2);
    
    % use distance as edge cost
    cost = norm(roadmap.nodes(u, :) - roadmap.nodes(v, :));
    
    adj{u} = [adj{u}; v, cost];
    adj{v} = [adj{v}; u, cost]; 
end

% run dijkstra's algorithm on each waypoint
for i = 1:T
    src = waypoints(i);
    distances = inf(N, 1);
    distances(src) = 0;
    visited = false(N, 1);
    parents = zeros(N, 1);

    for j = 1:N
        % find closest node
        search_dist = distances;
        search_dist(visited) = inf;
        [min_dist, u] = min(search_dist);
        if min_dist == inf
            break; 
        end
        visited(u) = true;
        
        % check if all waypoints visited
        if all(visited(waypoints))
            break;
        end
        
        % update distances to neighbors
        neighbors = adj{u};
        for k = 1:size(neighbors, 1)
            v = neighbors(k, 1);
            w = neighbors(k, 2);
            
            if ~visited(v)
                alt = distances(u) + w;
                if alt < distances(v)
                    distances(v) = alt;
                    parents(v) = u;
                end
            end
        end
    end

    % store in cost matrix and parent matrix
    for j = 1:T
        distance = waypoints(j);
        costMatrix(i, j) = distances(distance);
    end
    parentMatrix(i, :) = parents;
end
end