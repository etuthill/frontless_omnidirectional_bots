%check github

function [v] = agent_velocity(N, p, v, p_leader, v_leader, num_agents)

% parameter definition
k_c = 0.5;
k_a = 0.5;
k_s = 0.5;
k_p = 0.5;
k_v = 0.5;
r_0 = 0.1;
k_avoid = 0.1;

% preallocation
F_cohesion = zeros(num_agents, 2);
F_alignment = zeros(num_agents, 2);
F_separation = zeros(num_agents, 2);
F_leader = zeros(num_agents, 2);

for i = 1:num_agents
    for j = 1:length(N(i))
        if sum(N(i)) == 0
        % no neighbors: only follow leader
        v(i, :) = k_p * (p_leader - p(i,:)) + k_v * (v_leader - v(i,:));
        continue;
        end
        if sum(N(i)) > 0 && N(i, j) ~= 0
            % cohesion force on each agent (i) from neighbors (j)
            F_cohesion(i, :) = F_cohesion(i, :) +k_c*(1/sum(N(i)) * p(j,:) - p(i,:));
            
            % alignment force on each agent (i) from neighbors (j)
            F_alignment(i, :) = F_alignment(i, :) + k_a * (1/sum(N(i)) * v(j,:) - v(i,:));
            
            % separation force on each agent (i) from neighbors (j)
            r_vec = p(i,:) - p(j,:);
            r = norm(r_vec);
            F_separation(i,:) = k_s * (r_vec / r) * log(1 + r_0 / r);
            
            % force on each agent (i) from leader
            r_vec = p(i,:) - p_leader;
            r = norm(r_vec);  % distance between the particle and the leader
            
            % logarithmic repulsive force
            if r < r_0
                F_avoid = k_avoid * (r_vec / r) * log(1 + r_0 / r);  % Soft repulsion
            else
                F_avoid = [0, 0];  % no force if not too close
            end
            
            % update leader-following force
            F_leader(i, :) = F_leader(i, :) + k_p * (p_leader - p(i,:)) + k_v * (v_leader - v(i,:)) + F_avoid;
                        
            
        end
        % handle cases where distance from swarm is too great (N = 0)
    end
    % velocity on each agent (i)
    v(i, :) = F_cohesion(i, :) + F_alignment(i, :) + F_separation(i, :) + F_leader(i, :);
end
% Parameters:

% Cohesion:
% k_c = cohesion force constant (int)
% N = number of neighbors (mx)
% p = agent position (mx)

% Alignment 
% k_a = alignment force constant (int)
% N = number of neighbors (mx)
% v = agent velocity (mx)

% Separation: 
% k_s = separation force constant (int)
% p = agent position (mx)

% Leader:
% k_p = position following force constant (int)
% k_v = velocity following force constant (int)
% p_leader = leader position (vec)
% v_leader = leader velocity (vec)
% p = agent position (mx)
% v = agent velocity (mx)

% Forces:
% F_cohesion = cohesion force (pull) (mx)
% F_alignment = alignment force (pull) (mx)
% F_separation = separation force (push) (mx)
% F_leader = force relating agent to leader (pull) (mx)