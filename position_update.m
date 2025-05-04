function [updated_position] = position_update(current_position, velocity)
% POSITION_UPDATE - Updates agent positions based on current velocity.
%
%   updated_position = POSITION_UPDATE(current_position, velocity) returns
%   the new position of agents after applying the velocity update.

    updated_position = current_position + velocity;  % simple euler step
end
