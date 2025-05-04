%{
SWARM SIMULATION WITH LEADER FOLLOWING AND FORCE FIELD VISUALIZATION

This script simulates a simple swarm of agents following a moving leader. Agents
adjust their velocities based on local behaviors: cohesion, alignment, separation,
and attraction to the leader. The simulation also visualizes vector fields representing
each of these interaction forces over a 2d grid at every time step.

Features:
- leader follows a predefined trajectory (from pathdata.mat)
- agents follow the leader using swarm rules
- separation, cohesion, and alignment force fields are visualized
- trajectories of the leader and each agent are plotted

data file required:
- pathdata.mat: contains `leader_pos_store` and `leader_vel_store`
%}

load('pathdata.mat');

num_agents = 2;
num_steps = length(x_pos);

% extract leader path from data
x_pos = leader_pos_store(:,1)';
y_pos = leader_pos_store(:,2)';
x_vel = leader_vel_store(:,1)';
y_vel = leader_vel_store(:,2)';

% initialize agent positions and velocities
p = [-5, -1; -5, 1];  % each row is an agent
v = zeros(num_agents, 2);  % all agents start at rest

% store agent trajectories for plotting
agent_pos_history = cell(num_agents, 1);
for i = 1:num_agents
    agent_pos_history{i} = zeros(num_steps, 2);
    agent_pos_history{i}(1, :) = p(i, :);
end

% interaction parameters
k_c = 0.01;     % cohesion
k_a = 0.01;     % alignment
k_s = 1.0;      % separation
k_p = 0.05;     % attraction to leader position
k_v = 1.8;      % attraction to leader velocity
r_0 = 0.9;      % separation influence radius
dt = 0.05;      % time step
v_max = 1.0;    % max agent speed

% create a grid for visualizing force fields
[x_grid, y_grid] = meshgrid(-5:0.5:5, -5:0.5:5);

figure;

for step = 1:num_steps
    clf; % clear the figure each frame

    % get leader's current position and velocity
    p_leader_current = [x_pos(step), y_pos(step)];
    v_leader_current = [x_vel(step), y_vel(step)];

    % calculate neighbor relationships
    N = neighbor_calc(p, num_agents);

    % update agent velocities based on swarm rules
    v = agent_velocity(N, p, v, p_leader_current, v_leader_current, num_agents);

    % update agent positions
    p = position_update(p, v);

    % store current positions
    for i = 1:num_agents
        agent_pos_history{i}(step, :) = p(i, :);
    end

    % initialize force field components
    u_sep = zeros(size(x_grid)); v_sep = zeros(size(y_grid));
    u_coh = zeros(size(x_grid)); v_coh = zeros(size(y_grid));
    u_align = zeros(size(x_grid)); v_align = zeros(size(y_grid));

    % compute force field vectors at each grid point
    for ix = 1:size(x_grid, 1)
        for iy = 1:size(y_grid, 2)
            hypothetical_pos = [x_grid(ix, iy), y_grid(ix, iy)];
            F_sep = zeros(1, 2);
            F_coh = zeros(1, 2);
            F_align = zeros(1, 2);

            for i = 1:num_agents
                r_vec = hypothetical_pos - p(i, :);
                r = norm(r_vec);
                if r ~= 0
                    % compute separation from agents
                    F_sep = F_sep + k_s * (r_vec / r) * log(1 + r_0 / r);
                end
                % compute cohesion toward agents
                F_coh = F_coh + k_c * (p(i, :) - hypothetical_pos);

                % compute alignment (difference from mean velocity)
                F_align = F_align + k_a * (v(i, :) - mean(v, 1));
            end

            % store components of each force
            u_sep(ix, iy) = F_sep(1);
            v_sep(ix, iy) = F_sep(2);
            u_coh(ix, iy) = F_coh(1);
            v_coh(ix, iy) = F_coh(2);
            u_align(ix, iy) = F_align(1);
            v_align(ix, iy) = F_align(2);
        end
    end

    % plot leader trajectory
    plot(x_pos(1:step), y_pos(1:step), 'r.-', 'DisplayName', 'leader path'); 
    hold on;

    % plot agent paths and current positions
    for i = 1:num_agents
        traj = agent_pos_history{i};
        plot(traj(1:step, 1), traj(1:step, 2), '.-', 'DisplayName', sprintf('agent %d path', i));
        plot(p(i, 1), p(i, 2), '.', 'DisplayName', sprintf('agent %d', i), "Markersize", 20);
    end

    % plot current leader position
    plot(p_leader_current(1), p_leader_current(2), 'k.', 'MarkerSize', 20, 'MarkerFaceColor', 'k', 'DisplayName', "leader");

    % plot force vector fields
    quiver(x_grid, y_grid, u_sep, v_sep, 'r', 'AutoScaleFactor', 0.8, 'DisplayName', "separation");
    quiver(x_grid, y_grid, u_coh, v_coh, 'b', 'AutoScaleFactor', 0.8, 'DisplayName', "cohesion");
    %quiver(x_grid, y_grid, u_align, v_align, 'g', 'AutoScaleFactor', 0.8, 'DisplayName', "alignment");

    xlabel('x position (m)');
    ylabel('y position (m)');
    axis([-5, 5, -5, 5]);
    title(['swarm simulation step ' num2str(step)]);
    legend('Location', 'northeastoutside');
    drawnow;
    pause(0.05); % adjust pause to control animation speed
end
