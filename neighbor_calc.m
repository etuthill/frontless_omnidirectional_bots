function [N] = neighbor_calc(p, num_agents)
    neighbor_distance = 6;
    N = zeros(num_agents);
    for i = 1:num_agents
        for j = 1:num_agents
            distance = norm(p(i,:) - p(j,:));
            if distance <= neighbor_distance
                N(i, j) = 1;
            end
        end
    end
    N(logical(eye(num_agents))) = 0;  
end