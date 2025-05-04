function collision_matrix = check_collisions(p, collision_radius)
% CHECK_COLLISIONS - Detects pairwise collisions between agents.
%
%   collision_matrix = check_collisions(p, collision_radius)
%
%   Inputs:
%       p                - (n x 2) matrix of agent positions, where each row is [x, y]
%       collision_radius - scalar threshold distance to consider a collision
%
%   Output:
%       collision_matrix - (n x n) logical matrix where entry (i, j) is true if
%                          agents i and j are within collision_radius of each other

    num_agents = size(p, 1);
    collision_matrix = false(num_agents); 

    for i = 1:num_agents
        for j = i+1:num_agents
            % compute distance between agent i and j
            dist = norm(p(i,:) - p(j,:));
            % mark as collision if distance is below threshold
            if dist < collision_radius
                collision_matrix(i,j) = true;
                collision_matrix(j,i) = true;
            end
        end
    end
end
