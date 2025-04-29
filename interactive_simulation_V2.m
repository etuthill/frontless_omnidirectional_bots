function [] = interactive_simulation_V2()
    num_agents = 10; 
    p = rand(num_agents, 2) * 2 - 1;  % random start positions
    v = zeros(num_agents, 2);         % start with zero velocity
    p_leader_current = [1, 1];         % leader starts here
    v_leader_current = [0, 0];         % leader starts still

    joy = vrjoystick(1);               % connect to joystick
    drivetime = 50;                    % how long to run (seconds)
    tic;
    collision_count = 0;               % keep track of crashes

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

        % update leader movement
        v_leader_current = [dx, dy] * 0.06;  
        p_leader_current = p_leader_current + v_leader_current;

        % calculate neighbors and update agent velocities
        N = neighbor_calc(p, num_agents);  
        v = agent_velocity(N, p, v, p_leader_current, v_leader_current, num_agents);  
        p = position_update(p, v);  

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
end

