function [pos_x, pos_y] = control_leader(pos) 
% CONTROL_LEADER - Records leader position over time using joystick input.
%
%   [pos_x, pos_y] = control_leader(pos)
%
%   Inputs:
%       pos - initial 1x2 vector specifying the starting [x, y] position of the leader
%
%   Outputs:
%       pos_x - 1xn array of x positions over time
%       pos_y - 1xn array of y positions over time
%
%   This function reads joystick input using vrjoystick(1) and moves the leader
%   position accordingly for a fixed duration (15 seconds). Position updates are 
%   visualized live and recorded at each step.

    joy = vrjoystick(1); 
    
    % set up live plot
    figure;
    h = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlim([-5 5]);
    ylim([-5 5]);
    
    tic
    s = toc;
    drivetime = 15;
    i = 1;
    
    while s < drivetime
        s = toc;

        % read joystick axis values
        axes_vals = axis(joy);  
        dx = axes_vals(1);      
        dy = axes_vals(2); 
    
        % invert y direction to match screen coordinates
        dy = -dy;
    
        % update position
        pos(1) = pos(1) + 0.03 * dx;
        pos(2) = pos(2) + 0.03 * dy;
    
        % store position
        pos_x(1, i) = pos(1);
        pos_y(1, i) = pos(2);
        i = i + 1;
    
        % keep position within bounds
        pos = max(min(pos, [5 5]), [-5 -5]);
    
        % update plot
        set(h, 'XData', pos(1), 'YData', pos(2));
        drawnow;
        
        pause(0.02); 
    end

    % clear figure after movement
    clf;
end
