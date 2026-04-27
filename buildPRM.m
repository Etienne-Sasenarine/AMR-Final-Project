function roadmap = buildPRM(walls, boundaries, numSamples, radius, waypoints)
% buildPRM: builds probabilistic roadmap
%
% INPUTS:
%   walls               N-by-4 matrix of walls [x1, y1, x2, y2]
%   boundaries          map boundaries [x_min, x_max, y_min, y_max]
%   numSamples          number of random points to sample
%   radius              robot radius
%   waypoints           k-by-2 matrix of waypoints
%
% OUTPUTS:
%   roadmap.nodes       nodes in the roadmap 
%   roadmap.edges       edges in the roadmap

% define variables
nodes = [];
edges = [];
nodeComponents = [];
nextCompId = 1;

% generate samples
randomSamples = [boundaries(1) + rand(numSamples,1) * (boundaries(2) - boundaries(1)), ...
                 boundaries(3) + rand(numSamples,1) * (boundaries(4) - boundaries(3))];
allSamples = [waypoints; randomSamples];
totalSamples = size(allSamples, 1);

% run visibility PRM algorithm
for i = 1:totalSamples
    q = allSamples(i, :);

    % check if q is in free space
    in_obs = false;
    for w = 1:size(walls, 1)
        w1 = walls(w, 1:2);
        w2 = walls(w, 3:4);
        
        l2 = sum((w2 - w1).^2);
        if l2 == 0
            dist = norm(q - w1);
        else
            t_proj = max(0, min(1, dot(q - w1, w2 - w1) / l2));
            projection = w1 + t_proj * (w2 - w1);
            dist = norm(q - projection);
        end
        
        if dist <= radius
            in_obs = true; 
            break;
        end
    end
    
    if in_obs
        continue;
    end
    
    % first node in PRM
    if isempty(nodes)
        nodes = [nodes; q];
        nodeComponents = [nodeComponents; nextCompId];
        nextCompId = nextCompId + 1;
        continue;
    end
    
    % q visibility 
    visibleNodes = []; 
    visibleComps = [];
    uniqueComps = unique(nodeComponents);
    
    for c = 1:length(uniqueComps)
        c_id = uniqueComps(c);
        c_nodes_idx = find(nodeComponents == c_id);
        
        diffs = nodes(c_nodes_idx, :) - q;
        dists = sum(diffs.^2, 2);
        [~, min_idx] = min(dists);
        closest_node_idx = c_nodes_idx(min_idx);
        
        if ~checkEdgeCollision(q, nodes(closest_node_idx, :), walls, radius)
            visibleNodes = [visibleNodes; closest_node_idx];
            visibleComps = [visibleComps; c_id];
        end
    end
    
    % apply visibiltiy PRM keep/discard logic
    if isempty(visibleComps)
        % q cannot connect to any point q' in V
        nodes = [nodes; q];
        nodeComponents = [nodeComponents; nextCompId];
        nextCompId = nextCompId + 1;
        
    elseif length(visibleComps) >= 2 || i <= size(waypoints, 1)
        % q can connect to at least 2 disconnected components OR q is a waypoint
        new_node_idx = size(nodes, 1) + 1;
        nodes = [nodes; q];
        
        for v = 1:length(visibleNodes)
            edges = [edges; new_node_idx, visibleNodes(v)];
        end
        
        target_c_id = min(visibleComps);
        nodeComponents(new_node_idx, 1) = target_c_id;
        for v = 1:length(visibleComps)
            nodeComponents(nodeComponents == visibleComps(v)) = target_c_id;
        end
    end
end

% build roadmap data structure
roadmap.nodes = nodes;
roadmap.edges = edges;
end

% helper function to check if robot radius will cause collision with any wall
function isCollision = checkEdgeCollision(p1, p2, walls, radius)
    isCollision = false;
    stepSize = min(0.1, radius / 2); 
    edgeLength = norm(p2 - p1);
    numSteps = max(ceil(edgeLength / stepSize), 2);
    
    t = linspace(0, 1, numSteps)';
    edgePoints = [p1(1) + t*(p2(1)-p1(1)), p1(2) + t*(p2(2)-p1(2))];
    
    numWalls = size(walls, 1);
    for w = 1:numWalls
        w1 = walls(w, 1:2);
        w2 = walls(w, 3:4);
        
        l2 = sum((w2 - w1).^2);
        if l2 == 0
            dist = sqrt(sum((edgePoints - repmat(w1, numSteps, 1)).^2, 2));
        else
            t_proj = sum((edgePoints - repmat(w1, numSteps, 1)) .* repmat(w2 - w1, numSteps, 1), 2) / l2;
            t_proj = max(0, min(1, t_proj));
            projection = repmat(w1, numSteps, 1) + t_proj .* repmat(w2 - w1, numSteps, 1);
            dist = sqrt(sum((edgePoints - projection).^2, 2));
        end
       
        if any(dist <= radius)
            isCollision = true; return;
        end
    end
end