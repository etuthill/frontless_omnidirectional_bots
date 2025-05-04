function [v] = leader_p_to_v(p) 
% LEADER_P_TO_V - Computes leader velocity from position data.
%
%   v = leader_p_to_v(p)
%
%   Input:
%       p - 1D array of positions over time
%
%   Output:
%       v - 1D array of velocity values (difference between positions)
%
%   Assumes unit time steps. The first velocity is set to 0.

    v(1) = 0;

    % compute velocity as difference between positions
    for i = length(p)-1
        v(i+1) = p(i+1) - p(i);
    end
end
