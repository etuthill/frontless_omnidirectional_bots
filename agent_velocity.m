%{
AGENT_VELOCITY - Updates agent velocities based on swarm behavior and leader influence.

This function computes the new velocities for a swarm of agents using local interaction
rules (cohesion, alignment, separation) and attraction to a designated leader agent. 
Velocities are updated based on the sum of these interaction forces and are capped 
at a maximum speed.

INPUTS:
    N          - (num_agents x num_agents) adjacency matrix indicating neighboring agents
    p          - (num_agents x 2) matrix of agent positions
    v          - (num_agents x 2) matrix of agent velocities (will be updated)
    p_leader   - (1 x 2) position of the leader agent
    v_leader   - (1 x 2) velocity of the leader agent
    num_agents - scalar number of agents in the swarm

OUTPUT:
    v          - (num_agents x 2) updated velocity matrix for the agents

INTERNAL PARAMETERS:
    k_c   - cohesion force coefficient
    k_a   - alignment force coefficient
    k_s   - separation force coefficient
    k_p   - position attraction to leader
    k_v   - velocity matching with leader
    r_0   - interaction radius for separation
    dt    - time step for velocity update
    v_max - maximum allowed agent speed

BEHAVIOR:
    - Cohesion: Agents steer towards the average position of their neighbors.
    - Alignment: Agents align their velocity with their neighbors.
    - Separation: Agents repel from neighbors and the leader if too close.
    - Leader following: Agents are attracted to the leader's position and velocity,
      with a weakened effect if too close to avoid crowding.
    - All forces are combined and integrated using Euler's method.
    - Final velocities are clipped to a max magnitude to ensure stability.
%}


function [v] = agent_velocity(N, p, v, p_leader, v_leader, num_agents)

    % parameters
    k_c = 0.01;     % cohesion
    k_a = 0.01;     % alignment
    k_s = 1.0;      % separation
    k_p = 0.05;     % attraction to leader position
    k_v = 1.8;     % attraction to leader velocity
    r_0 = 0.9;      % seperation influence radius
    dt = 0.05;      % time step for velocity integration
    v_max = 1.0;    % max velocity

      % set up force arrays
    F_cohesion = zeros(num_agents, 2);
    F_alignment = zeros(num_agents, 2);
    F_separation = zeros(num_agents, 2);
    F_leader = zeros(num_agents, 2);

    for i = 1:num_agents
        % figure out force from following the leader
        r_vec = p(i,:) - p_leader;
        r = norm(r_vec);

        if r < r_0
            leader_follow_factor = (r / r_0)^2;
            F_leader(i, :) = leader_follow_factor * (k_p * (p_leader - p(i,:)) + k_v * (v_leader - v(i,:)));
        else
            F_leader(i, :) = k_p * (p_leader - p(i,:)) + k_v * (v_leader - v(i,:));
        end

        % reset forces
        F_cohesion(i, :) = [0, 0];
        F_alignment(i, :) = [0, 0];
        F_separation(i, :) = [0, 0];

        % check all the neighbors
        neighbors = find(N(i,:) ~= 0);
        num_neighbors = length(neighbors);

        for j = neighbors
            % pull towards neighbors (cohesion)
            F_cohesion(i, :) = F_cohesion(i, :) + k_c * ((1/num_neighbors) * (p(j,:) - p(i,:)));

            % match neighbor speeds (alignment)
            F_alignment(i, :) = F_alignment(i, :) + k_a * ((1/num_neighbors) * (v(j,:) - v(i,:)));

            % push away if too close (separation)
            r_vec_neighbor = p(i,:) - p(j,:);
            r_neighbor = norm(r_vec_neighbor);

            if r_neighbor < r_0 && r_neighbor > 0
                separation_strength = k_s * (1/r_neighbor - 1/r_0);
                F_separation(i, :) = F_separation(i, :) + separation_strength * (r_vec_neighbor / r_neighbor);
            end
        end

        % also push away from leader if too close
        r_vec_leader = p(i,:) - p_leader;
        r_leader = norm(r_vec_leader);
        if r_leader < r_0 && r_leader > 0
            separation_from_leader_strength = k_s * (1/r_leader - 1/r_0);
            F_separation(i,:) = F_separation(i,:) + separation_from_leader_strength * (r_vec_leader / r_leader);
        end

        % update velocity based on total forces
        total_force = F_cohesion(i,:) + F_alignment(i,:) + F_separation(i,:) + F_leader(i,:);
        v(i, :) = v(i,:) + total_force * dt;

        % limit speed so no one goes crazy fast
        speed = norm(v(i,:));
        if speed > v_max
            v(i,:) = (v(i,:) / speed) * v_max;
        end

    end
end