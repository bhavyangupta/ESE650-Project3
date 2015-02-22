%%
% vect_to_quat.m
% Bhavya
% Converts 3D rotaion vector to 4D quaternion vector
%
% Input: quaternion      
%
% Output: rotation vector
%         
% Usage: 

function [quat] = vect_to_quat(vect)
    alpha = norm(vect);
    if alpha == 0
      
        quat  = [1;0;0;0];
        return
    end    
    e = vect/alpha;
    quat = [cos(alpha/2);e(1)*sin(alpha/2);e(2)*sin(alpha/2);e(3)*sin(alpha/2)];

end