function [] = vector_simulation_drive()
% VECTOR_SIMULATION_DRIVE - Runs a simulation where agents follow a leader
% based on joystick input and stores their trajectories.
%
%   This function simulates a group of agents (2 in this case) following a
%   leader whose movement is controlled via a joystick. The positions and
%   velocities of both the leader and the agents are updated in each iteration.
%   The simulation runs for a specified amount of time, and the trajectories
%   are stored in 'pathdata.mat'.

    num_agents = 2; 
    p = [-5 -1; -5 1];  % initial agent positions
    v = zeros(num_agents, 2);  % start with zero velocity
    p_leader_current = [-4, 0];  % leader starts here
    v_leader_current = [0, 0];  % leader starts still

    joy = vrjoystick(1);  % connect to joystick
    drivetime = 8;  % how long to run (seconds)
    tic;
    collision_count = 0;  % keep track of crashes

    max_records = 130;  % maximum records for storing trajectory data
    leader_pos_store = zeros(max_records, 2);  % store leader position
    leader_pos_store(1,:) = p_leader_current;
    leader_vel_store = zeros(max_records, 2);  % store leader velocity
    leader_vel_store(1,:) = v_leader_current;
    store_index = 1;

    % set up the plot
    fig = figure('Name', 'interactive swarm simulation', 'NumberTitle', 'off');
    axis([-5, 5, -5, 5]);
    grid on;
    hold on;

    while toc < drivetime
        % read joystick input
        axes_vals = axis(joy);  
        dx = axes_vals(1);
        dy = -axes_vals(2); 

        % update leader movement based on joystick input
        v_leader_current = [dx, dy] * 0.06;  
        p_leader_current = p_leader_current + v_leader_current;

        % calculate neighbors and update agent velocities
        N = neighbor_calc(p, num_agents);  
        v = agent_velocity(N, p, v, p_leader_current, v_leader_current, num_agents);  
        p = position_update(p, v);

        % store leader's position and velocity
        leader_pos_store(store_index, :) = p_leader_current;
        leader_vel_store(store_index, :) = v_leader_current;

        % store each agent's position and velocity
        for a = 1:2
            agent_pos_store(store_index, a, :) = p(a, :);
            agent_vel_store(store_index, a, :) = v(a, :);
        end

        store_index = store_index + 1;

        % redraw everything
        cla;
        plot(p_leader_current(1), p_leader_current(2), 'r.', 'MarkerSize', 15);  % leader in red
        for j = 1:num_agents
            plot(p(j, 1), p(j, 2), 'mo');  % agents in magenta
        end
        drawnow;

        % check for collisions
        collision_radius = 0.05;
        collisions = check_collisions(p, collision_radius); 
        if any(collisions(:))
            collision_count = collision_count + 1;
        end

        pause(0.02);  % small delay so it runs smoothly
    end

    % save the collected data to a file
    save('pathdata.mat', 'leader_pos_store', 'leader_vel_store', 'agent_pos_store', 'agent_vel_store');
end
