% Number of steps
num_steps = 75;
name = "circle"


%%
% Initialize agent positions and velocities
num_agents = 2;
p = [-2, -1; -2, 1];  % Each row is an agent's initial position
v = zeros(num_agents, 2);

% Store agent trajectories
agent_pos_history = cell(num_agents, 1);
for i = 1:num_agents
    agent_pos_history{i} = zeros(num_steps, 2);
    agent_pos_history{i}(1, :) = p(i, :);
end

% Parameters for forces
    k_c = 0.01;     % cohesion
    k_a = 0.01;     % alignment
    k_s = 1.0;      % separation
    k_p = 0.05;     % attraction to leader position
    k_v = 1.8;     % attraction to leader velocity
    r_0 = 0.9;      % seperation influence radius
    dt = 0.05;      % time step for velocity integration
    v_max = 1.0;    % max velocity

% Grid for force field visualization
[x_grid, y_grid] = meshgrid(-5:0.5:5, -5:0.5:5);

% Prepare to store force vectors
u_sep = zeros(size(x_grid));  % Separation force (x-component)
v_sep = zeros(size(y_grid));  % Separation force (y-component)

u_coh = zeros(size(x_grid));  % Cohesion force (x-component)
v_coh = zeros(size(y_grid));  % Cohesion force (y-component)

u_align = zeros(size(x_grid));  % Alignment force (x-component)
v_align = zeros(size(y_grid));  % Alignment force (y-component)

u_leader = zeros(size(x_grid));  % Leader following force (x-component)
v_leader = zeros(size(y_grid));  % Leader following force (y-component)


%%

% Create a VideoWriter object
output_folder = 'C:\Users\etuthill\Videos\QEA_2_Final';
videoFileName = fullfile(output_folder, sprintf('%s.mp4', name));
% Create a VideoWriter object
videoWriter = VideoWriter(videoFileName, 'MPEG-4');  % 'MPEG-4' for MP4 format
videoWriter.FrameRate = 5;  % Adjust the frame rate (frames per second)
open(videoWriter);  % Open the video file for writing


% Main loop for each time step
for step = 1:num_steps
    clf; % Clear figure each frame

    % Update leader position and velocity
    p_leader_current = [x_pos(step), y_pos(step)];
    v_leader_current = [x_vel(step), y_vel(step)];

    % Swarm logic (calculate neighbors, update velocities, positions)
    N = neighbor_calc(p, num_agents);
    v = agent_velocity(N, p, v, p_leader_current, v_leader_current, num_agents);
    p = position_update(p, v);

    % Update agent history for visualization
    for i = 1:num_agents
        agent_pos_history{i}(step, :) = p(i, :);
    end

    % Calculate forces for visualization (separation, cohesion, and leader following)
    for ix = 1:size(x_grid, 1)
        for iy = 1:size(y_grid, 2)
            hypothetical_pos = [x_grid(ix, iy), y_grid(ix, iy)];
            F_sep = zeros(1, 2);
            F_coh = zeros(1, 2);
            F_align = zeros(1, 2);
            F_leader = zeros(1, 2);

            % Calculate separation, cohesion, and alignment for all agents
            for i = 1:num_agents
                % Separation Force (Avoidance)
                r_vec = hypothetical_pos - p(i, :);  % Vector from agent i to hypothetical position
                r = norm(r_vec);  % Distance
                if r ~= 0
                    F_sep = F_sep + k_s * (r_vec / r) * log(1 + r_0 / r);
                end
                
                % Cohesion Force (Attraction to neighbors)
                F_coh = F_coh + k_c * (p(i, :) - hypothetical_pos);
                
                % Alignment Force (Align with neighbors)
                F_align = F_align + k_a * (v(i, :) - mean(v, 1));
            end

            % Compute leader-following force (attraction and repulsion)
            r_vec_leader = hypothetical_pos - p_leader_current;  % Attraction to leader
            r_leader = norm(r_vec_leader);

            % Initialize leader force vector
            F_leader = zeros(1, 2);

            % Repulsion from the leader if too close
            if r_leader < r_0 && r_leader > 0
                % Repulsion strength is stronger the closer you get
                separation_from_leader_strength = k_s * (1/r_leader - 1/r_0);
                % Repulsion force points away from the leader (negative of r_vec_leader)
                F_leader = F_leader - separation_from_leader_strength * (r_vec_leader / r_leader);
            end

            % Attraction to leader if farther away
            if r_leader >= r_0
                leader_factor = (r_leader / r_0)^2;
                % Attraction force points toward the leader
                F_leader = F_leader + leader_factor * (k_p * r_vec_leader + k_v * (v_leader_current - mean(v, 1)));
            end

            % Store the computed force components for visualization
            u_leader(ix, iy) = -F_leader(1); % Negative because quiver should point from the agent to the leader
            v_leader(ix, iy) = -F_leader(2); % Negative because quiver should point from the agent to the leader

            % Store computed force components for visualization
            u_sep(ix, iy) = F_sep(1);
            v_sep(ix, iy) = F_sep(2);

            u_coh(ix, iy) = F_coh(1);
            v_coh(ix, iy) = F_coh(2);

            u_align(ix, iy) = F_align(1);
            v_align(ix, iy) = F_align(2);
        end
    end

    % Plot leader trajectory
    plot(x_pos(1:step), y_pos(1:step), 'r.-', 'DisplayName', 'Leader Path'); 
    hold on;

    % Plot agent paths and current positions
    for i = 1:num_agents
        traj = agent_pos_history{i};
        plot(traj(1:step, 1), traj(1:step, 2), '.-', 'DisplayName', sprintf('Agent %d Path', i));
        plot(p(i, 1), p(i, 2), '.', 'DisplayName', sprintf('Agent %d', i), "Markersize", 20);
    end

    % Plot leader's current position
    plot(p_leader_current(1), p_leader_current(2), 'k.', 'MarkerSize', 20, 'MarkerFaceColor', 'k', 'DisplayName', "Leader");

    % Plot force vector fields
    quiver(x_grid, y_grid, u_leader, v_leader, 'm', 'AutoScaleFactor', 0.8, 'DisplayName', "Leader Following");
    quiver(x_grid, y_grid, u_sep, v_sep, 'r', 'AutoScaleFactor', 0.8, 'DisplayName', "Separation");
    quiver(x_grid, y_grid, u_coh, v_coh, 'b', 'AutoScaleFactor', 0.8, 'DisplayName', "Cohesion");

    % Labeling and formatting
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    axis([-5, 5, -5, 5]);
    title(['Swarm Simulation Step ' num2str(step)]);
    legend('Location', 'northeastoutside');
    
    % Capture the current frame and write it to the video
    frame = getframe(gcf);  % Capture the current figure as a frame
    writeVideo(videoWriter, frame);  % Write the frame to the video file

    drawnow;
    pause(0.05); % Adjust for speed
end

% Close the video writer object to finalize the video file
close(videoWriter);
%%
% Plot the spiral path of the leader
figure;
plot(x_pos, y_pos, 'r.-', 'DisplayName', 'Leader Path');
xlabel('X Position (m)');
ylabel('Y Position (m)');
axis([-5, 5, -5, 5]);
title(name);

% Save the still plot as an image
saveas(gcf, sprintf('C:/Users/etuthill/Videos/QEA_2_Final/%s.png', name));  % Save as PNG image