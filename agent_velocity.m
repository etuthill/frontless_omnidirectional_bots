%check github

function [v] = agent_velocity(N, p, v, p_leader, v_leader, num_agents)

% parameter definition
k_c = 0.1;     % cohesion
k_a = 0.2;      % alignment
k_s = 0.4;      % separation

% Leader-following
k_p = 0.05;     % attraction to leader position - reduce!
k_v = 0.02;     % attraction to leader velocity - reduce!

% Avoidance
r_0 = 0.8;    
k_avoid = 0.8;   % makes repulsion significantly stronger

% preallocation
F_cohesion = zeros(num_agents, 2);
F_alignment = zeros(num_agents, 2);
F_separation = zeros(num_agents, 2);
F_leader = zeros(num_agents, 2);

for i = 1:num_agents
    if sum(N(i)) == 0
        % no neighbors: only follow leader
        r_vec = p(i,:) - p_leader;
        r = norm(r_vec);
        if r < r_0
            F_avoid = k_avoid * (r_vec / r) * log(1 + r_0 / r);
        else
            F_avoid = [0, 0];
        end
        v(i, :) = k_p * (p_leader - p(i,:)) + k_v * (v_leader - v(i,:)) + F_avoid;
        continue;
    end
    
    for j = 1:length(N(i))
        if N(i, j) ~= 0
            % cohesion force
            F_cohesion(i, :) = F_cohesion(i, :) + k_c * (1/sum(N(i)) * p(j,:) - p(i,:));

            % alignment force
            F_alignment(i, :) = F_alignment(i, :) + k_a * (1/sum(N(i)) * v(j,:) - v(i,:));

            % separation force
            r_vec = p(i,:) - p(j,:);
            r = norm(r_vec);
            F_separation(i,:) = F_separation(i,:) + k_s * (r_vec / r) * log(1 + r_0 / r);
        end
    end

    % leader-following
    r_vec = p(i,:) - p_leader;
    r = norm(r_vec);
    if r < r_0
        F_avoid = k_avoid * (r_vec / r) * log(1 + r_0 / r);
    else
        F_avoid = [0, 0];
    end

    F_leader(i, :) = k_p * (p_leader - p(i,:)) + k_v * (v_leader - v(i,:)) + F_avoid;

    % update velocity
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