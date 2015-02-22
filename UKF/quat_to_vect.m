%%
% quat_to_vect.m
% Bhavya
% Converts 4D quaternion to 3D rotation vector
%
% Input: quaternion
%        
%
% Output: rotation vector
%         
% Usage: 

function [rotation_vector] = quat_to_vect(quat)
alpha  = 2 * acos(quat(1));
if alpha == 0
    rotation_vector=[0;0;0];
    return
end
rotation_vector = [quat(2)/sin(alpha/2);quat(3)/sin(alpha/2);quat(4)/sin(alpha/2)];
rotation_vector = rotation_vector/norm(rotation_vector);
rotation_vector = rotation_vector * alpha;
end