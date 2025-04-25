function [] = interactive_simulation_V2()
    num_agents = 5; 
    p = zeros(num_agents, 2);
    v = zeros(num_agents, 2);  
    p_leader_current = [1, 1];  
    v_leader_current = [0, 0]; 

    joy = vrjoystick(1); 
    drivetime = 30; 
    tic;
    collision_count = 0;  

    fig = figure('Name', 'Interactive Swarm Simulation', 'NumberTitle', 'off');
    axis([-5, 5, -5, 5]);
    grid on;
    hold on;

    while toc < drivetime
        axes_vals = axis(joy);  
        dx = axes_vals(1);
        dy = -axes_vals(2); 

        v_leader_current = [dx, dy] * 0.075;  
        p_leader_current = p_leader_current + v_leader_current;

        N = neighbor_calc(p, num_agents);  
        v = agent_velocity(N, p, v, p_leader_current, v_leader_current, num_agents);  
        p = position_update(p, v);  

        cla;
        plot(p_leader_current(1), p_leader_current(2), 'r.', 'MarkerSize', 15); 
        for j = 1:num_agents
            plot(p(j, 1), p(j, 2), 'mo');  
        end
        drawnow;

  
        collision_radius = 0.05;
        collisions = check_collisions(p, collision_radius); 
        if any(collisions(:))
            collision_count = collision_count + 1;
        end

        pause(0.02);  
    end
end
