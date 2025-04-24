function [N] = neighbor_calc(p, num_agents)
    neighbor_distance = 3;
    N = zeros(num_agents);
    for i = 1:num_agents
        for j = 1:num_agents
            distance = sqrt((p(i, 1)-p(j ,1)).^2 + (p(i, 2) - p(j, 2)).^2);
            if distance <= neighbor_distance
                N(i, j) = 1;
            end
        end
    end
    one_matrix = ones(num_agents, 1);
    N = N - diag(one_matrix);
end
