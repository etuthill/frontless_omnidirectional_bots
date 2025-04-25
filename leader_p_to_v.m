function [v] = leader_p_to_v(p) 
    v(1) = 0;
    for i = length(p)-1
        v(i+1) = p(i+1) - p(i);
    end
end