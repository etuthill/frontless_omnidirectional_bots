function [updated_position] = position_update(current_position, velocity)
    updated_position = current_position + velocity;
end