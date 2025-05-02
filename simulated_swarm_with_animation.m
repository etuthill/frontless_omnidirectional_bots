clear all; close all; clc;
name = "Projection"
num_agents = 2;
num_steps = 75;

    k_c = 0.01;     % cohesion
    k_a = 0.01;     % alignment
    k_s = 1.0;      % separation
    k_p = 0.05;     % attraction to leader position
    k_v = 1.8;     % attraction to leader velocity
    r_0 = 0.9;      % seperation influence radius
    dt = 0.05;      % time step for velocity integration
    v_max = 1.0;    % max velocity


t = linspace(0, 10*pi, num_steps);
scale_factor = 0.3;
x_pos = scale_factor * (sin(t) + 2*sin(2*t));
y_pos = scale_factor * (cos(t) - 2*cos(2*t));
x_vel = gradient(x_pos, t);
y_vel = gradient(y_pos, t);

p = [-2, -1;    % Agent 1
     -2, 1];    % Agent 2
v = zeros(num_agents, 2);

% Store agent trajectories
agent_pos_history = cell(num_agents, 1);
for i = 1:num_agents
    agent_pos_history{i} = zeros(num_steps, 2);
    agent_pos_history{i}(1, :) = p(i, :);
end

[x_grid, y_grid] = meshgrid(-5:0.5:5, -5:0.5:5);

output_folder = "C:\Users\etuthill\Videos\QEA_2_Final";
videoFileName = fullfile(output_folder, name);
videoWriter = VideoWriter(videoFileName, 'MPEG-4');
videoWriter.FrameRate = 10;
open(videoWriter);

main_fig = figure('Position', [100 100 800 600]);

for step = 1:num_steps
    figure(main_fig); % Ensure we're drawing in the correct figure
    clf; % Clear figure each frame

    % Update leader
    p_leader_current = [x_pos(step), y_pos(step)];
    v_leader_current = [x_vel(step), y_vel(step)];

    % Swarm logic
    N = neighbor_calc(p, num_agents);
    v = agent_velocity(N, p, v, p_leader_current, v_leader_current, num_agents);
    
    % Limit velocities to v_max
    speeds = sqrt(sum(v.^2, 2));
    v = v .* min(v_max ./ speeds, 1);
    
    % Update positions
    p = position_update(p, v);

    % Update agent history
    for i = 1:num_agents
        agent_pos_history{i}(step, :) = p(i, :);
    end

    u_sep = zeros(size(x_grid)); v_sep = zeros(size(y_grid));
    u_coh = zeros(size(x_grid)); v_coh = zeros(size(y_grid));
    u_align = zeros(size(x_grid)); v_align = zeros(size(y_grid));
    u_lead = zeros(size(x_grid)); v_lead = zeros(size(y_grid));

for ix = 1:size(x_grid, 1)
    for iy = 1:size(y_grid, 2)
        pos = [x_grid(ix,iy), y_grid(ix,iy)];
        r_vec = p_leader_current - pos;
        r = norm(r_vec);
        
        if r > 0.01
            [~, nearest_agent] = min(vecnorm(p - pos, 2, 2));
            v_ref = v(nearest_agent,:);
            
            if r < r_0
                follow_factor = (r / r_0)^2;
                F_lead = follow_factor * k_p * r_vec;  % Remove velocity influence here
            else
                F_lead = k_p * r_vec;  % Remove velocity influence here
            end
            
            if r < r_0 / 2
                separation_strength = k_s * (1 / r - 2 / r_0);
                F_lead = F_lead - separation_strength * (r_vec / r);  % Separation still applies
            end

            u_lead(ix,iy) = F_lead(1);
            v_lead(ix,iy) = F_lead(2);
        else
        end
    end
end


    for ix = 1:size(x_grid, 1)
        for iy = 1:size(y_grid, 2)
            hypothetical_pos = [x_grid(ix, iy), y_grid(ix, iy)];
            F_sep = zeros(1, 2);
            F_coh = zeros(1, 2);
            F_align = zeros(1, 2);
            F_lead = zeros(1, 2);

            for i = 1:num_agents
                r_vec = hypothetical_pos - p(i, :);
                r = norm(r_vec);
                if r ~= 0
                    % Separation from agents
                    F_sep = F_sep + k_s * (r_vec / r) * log(1 + r_0 / r);
                end
                % Cohesion toward agents
                F_coh = F_coh + k_c * (p(i, :) - hypothetical_pos);

                % Alignment with agents (relative to mean velocity)
                F_align = F_align + k_a * (v(i, :) - mean(v, 1));
    

            end

            
            % Store force components
            u_sep(ix, iy) = F_sep(1); v_sep(ix, iy) = F_sep(2);
            u_coh(ix, iy) = F_coh(1); v_coh(ix, iy) = F_coh(2);
            u_align(ix, iy) = F_align(1); v_align(ix, iy) = F_align(2);
        end
    end

    % Plot leader trajectory
    plot(x_pos(1:step), y_pos(1:step), 'r.-', 'LineWidth', 1.5, 'DisplayName', 'Leader Path'); 
    hold on;

    % Plot agent paths and current positions
    colors = lines(num_agents);
    for i = 1:num_agents
        traj = agent_pos_history{i};
        plot(traj(1:step, 1), traj(1:step, 2), '.-', 'Color', colors(i,:), 'DisplayName', sprintf('Agent %d Path', i));
        plot(p(i, 1), p(i, 2), 'o', 'MarkerFaceColor', colors(i,:), 'MarkerSize', 8, 'DisplayName', sprintf('Agent %d', i));
    end

    % Plot leader's current position
    plot(p_leader_current(1), p_leader_current(2), 'k.', 'MarkerSize', 25, 'MarkerFaceColor', 'k', 'DisplayName', "Leader");

    % Plot force vector fields
    quiver(x_grid, y_grid, u_sep, v_sep, 'r', 'AutoScaleFactor', 1, 'DisplayName', "Separation");
    quiver(x_grid, y_grid, u_coh, v_coh, 'b', 'AutoScaleFactor', 1, 'DisplayName', "Cohesion");
    quiver(x_grid, y_grid, u_lead, v_lead, 'g', 'AutoScaleFactor', 1, 'DisplayName', "Leader");

    % Formatting
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    axis([-5, 5, -5, 5]);
    title(['Swarm Simulation Step ' num2str(step)]);
    legend('Location', 'northeastoutside');
    grid on;
    drawnow;
    
    % Capture frame for video
    frame = getframe(main_fig);
    writeVideo(videoWriter, frame);
    
    pause(0.01); % Small pause for animation
end

close(videoWriter);
disp(['Animation saved as: ' videoFileName]);

leader_fig = figure;
plot(x_pos, y_pos, 'r.-', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
axis([-5, 5, -5, 5]);
title(name);
grid on;

% Save leader path plot
leader_plot_name = fullfile(output_folder, name);
saveas(leader_fig, leader_plot_name, 'png');
disp(['Leader path plot saved as: ' leader_plot_name]);