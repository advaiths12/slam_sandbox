function [tf] = state_to_transform(x)
    translation = x(1:3);
    rotation = quat2rotm(x(4:7)');
    tf = [[rotation; 0, 0, 0], [translation; 1]];
end