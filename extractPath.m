function path = extractPath(bestPath, waypoints, parentMatrix)
% extractPath: returns sequence of nodes in PRM to visit
%
%   INPUTS
%       bestPath        waypoint path from shortestHamiltonianPath
%       waypoints       list of waypoint indices
%       parentMatrix    T-by-N matrix of parent nodes from waypoint(i)
%
%   OUTPUTS
%       path            sequence of nodes to visit

path = [];

% loop through each point in shortest hamiltonian path
for i = 1:(length(bestPath) - 1)
    src_idx = bestPath(i);   
    dst_idx = bestPath(i + 1);
    
    % find actual nodes in PRM
    src_node = waypoints(src_idx);
    dst_node = waypoints(dst_idx);
    
    % backtrack destination to source
    segment_path = [];
    curr = dst_node;
    
    while curr ~= src_node
        segment_path = [curr, segment_path];
        curr = parentMatrix(src_idx, curr);
        
        if curr == 0
            error('Backtracking failed: No valid path found. Check PRM connectivity.');
        end
    end
    
    if i == 1
        segment_path = [src_node, segment_path];
    end
    
    % append to full path
    path = [path, segment_path];
end
end