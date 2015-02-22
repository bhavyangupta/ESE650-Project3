function [euler_angles] = euler_from_rot(rot)
N = size(rot,3);
euler_angles = zeros(3,N);

for k=1 :N
    euler_angles(2,k) = atan2(-rot(1,3,k),sqrt(rot(3,2,k)^2 + rot(3,3,k)^2));
    if abs(euler_angles(2,k)-pi/2)<10e-3
        euler_angles(1,k) = 0;
        euler_angles(3,k) = 0;
    else 
        euler_angles(1,k ) = atan2(rot(2,3,k),rot(3,3,k));
        euler_angles(3,k) = atan2(rot(1,2,k),rot(1,1,k));
     end
   
    
end
end