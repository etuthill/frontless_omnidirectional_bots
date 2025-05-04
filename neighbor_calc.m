function [N] = neighbor_calc(p, num_agents)
% NEIGHBOR_CALC - Computes a binary adjacency matrix of neighboring agents.
%
%   N = NEIGHBOR_CALC(p, num_agents) returns an NxN matrix where N(i,j) is 1
%   if agent j is within a specified distance of agent i, and 0 otherwise.
%   The diagonal is set to 0 since agents are not neighbors with themselves.

    neighbor_distance = 10;
    N = zeros(num_agents);
    for i = 1:num_agents
        for j = 1:num_agents
            distance = norm(p(i,:) - p(j,:));
            if distance <= neighbor_distance
                N(i, j) = 1;  % mark as neighbor
            end
        end
    end
    N(logical(eye(num_agents))) = 0;  % zero out diagonal (self-neighboring)
end
