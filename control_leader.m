function [pos_x, pos_y] = control_leader(pos) 
% pos is leader position
    joy = vrjoystick(1); 
    
    figure;
    h = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlim([-5 5]);
    ylim([-5 5]);
    
    tic
    s = toc;
    drivetime = 15;
    i = 1;
    
    while s<drivetime
        s = toc;
        axes_vals = axis(joy);  
        dx = axes_vals(1);      
        dy = axes_vals(2); 
    
        dy = -dy;
    
        pos(1) = pos(1) + 0.03 * dx;
        pos(2) = pos(2) + 0.03 * dy;
    
        pos_x(1, i) = pos(1);
        pos_y(1, i) = pos(2);
        i = i+1;
    
        pos = max(min(pos, [5 5]), [-5 -5]);
    
        set(h, 'XData', pos(1), 'YData', pos(2));
        drawnow;
        
        pause(0.02); 
    end
    clf;
end