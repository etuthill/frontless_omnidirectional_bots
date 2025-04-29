function [] = interactive_simulation()
num_agents = 2;
p = zeros(num_agents, 2);
v = zeros(num_agents, 2);
p_leader_current = [0, 0];
v_leader_current = [0, 0];

for i = 1:num_agents
    eval(sprintf('p_%d = [0; 0];', i));
    eval(sprintf('v_%d = [0; 0];', i));
end

p_1 = [-5; -1];
p_2 = [-5; 1];
p = [p_1'; p_2'];

[x_pos, y_pos] = control_leader(p_leader_current);
x_vel = leader_p_to_v(x_pos);
y_vel = leader_p_to_v(y_pos);
num_steps = length(x_pos);
collision_count = 0;

for i = 1:num_steps
    N = neighbor_calc(p, num_agents);
    v = agent_velocity(N, p, v, p_leader_current, v_leader_current, num_agents);
    p = position_update(p, v);
    
    for j = 1:num_agents
            eval(sprintf('p_%d(:, end+1) = p(%d, :)'';', j, j)); 
            eval(sprintf('v_%d(:, end+1) = v(%d, :)'';', j, j));
    end

    p_leader_current = [x_pos(i), y_pos(i)];
    v_leader_current = [x_vel(i), y_vel(i)];

    collision_radius = 0.05;
    collisions = check_collisions(p, collision_radius);
    if any(collisions(:))
        collision_count = collision_count + 1;
    end
end

disp(['Total Collision Count: ', num2str(collision_count)]);

figure;
plot(x_pos, y_pos, 'r.-'); 
hold on;
for i = 1:num_agents
    plot(p(i, 1), p(i, 2), 'mo'); 
    plot(p_1(1, :), p_1(2, :), 'g.-');
    plot(p_2(1, :), p_2(2, :), 'b.-');
end
xlabel('X Position (m)');
ylabel('Y Position (m)');
axis([-5, 5, -5, 5]);
title('Swarm Mechanics Simulation');


