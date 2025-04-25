function [v] = agent_velocity(N, p, v, p_leader, v_leader, num_agents)

% parameter definition
k_c = 0.09;     % cohesion
k_a = 0.05;      % alignment
k_s = 0.6;      % separation

% Leader-following
k_p = 0.1;     % attraction to leader position 
k_v = 0.05;     % attraction to leader velocity 

% Avoidance
r_0 = 0.5;    
k_avoid = 0.3;   

% Threshold for stopping agent movement
stop_threshold = 0.5;  % distance under which the agent stops

% preallocation
F_cohesion = zeros(num_agents, 2);
F_alignment = zeros(num_agents, 2);
F_separation = zeros(num_agents, 2);
F_leader = zeros(num_agents, 2);

for i = 1:num_agents
    % Calculate distance to leader
    r_vec = p(i,:) - p_leader;
    r = norm(r_vec);

    if 0.1 < r < stop_threshold
        % If the agent is within the threshold distance, stop the agent
        v(i, :) = [0, 0];
    else
        % Otherwise, calculate forces based on neighbors and leader
        F_cohesion(i, :) = [0, 0];
        F_alignment(i, :) = [0, 0];
        F_separation(i, :) = [0, 0];
        F_leader(i, :) = [0, 0];
        
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
        
        % Leader-following behavior
        if r >= stop_threshold
            if r < r_0
                F_avoid = k_avoid * (r_vec / r) * log(1 + r_0 / r);
            else
                F_avoid = [0, 0];
            end
            F_leader(i, :) = k_p * (p_leader - p(i,:)) + k_v * (v_leader - v(i,:)) + F_avoid;
        end

        % Update velocity based on combined forces
        v(i, :) = F_cohesion(i, :) + F_alignment(i, :) + F_separation(i, :) + F_leader(i, :);
    end
end
