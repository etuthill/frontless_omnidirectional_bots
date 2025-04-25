function collision_matrix = check_collisions(p, collision_radius)
    num_agents = size(p, 1);
    collision_matrix = false(num_agents); 
    for i = 1:num_agents
        for j = i+1:num_agents
            dist = norm(p(i,:) - p(j,:));
            if dist < collision_radius
                collision_matrix(i,j) = true;
                collision_matrix(j,i) = true;
            end
        end
    end
end
