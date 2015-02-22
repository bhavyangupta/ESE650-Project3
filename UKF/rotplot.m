function [] = rotplot(Rots,LENGTH,file_num)
% This is modified rotplot.  Plots the estimated motion with the actual
% vicon motion
%
% Input: all rotation matrices to be plotted , Length of simulation,
% file_number of the corresponding Vicon file
%
% Output: visualisation
% return value: none
% 

flag = 0;
lx = 3.0;
ly = 1.5;
lz = 1.0;

x = .5*[+lx -lx +lx -lx +lx -lx +lx -lx;
        +ly +ly -ly -ly +ly +ly -ly -ly;
        +lz +lz +lz +lz -lz -lz -lz -lz];
ifront = [1 3 7 5 1];
iback = [2 4 8 6 2];
itop = [1 2 4 3 1];
ibottom = [5 6 8 7 5];

% load(['viconRot',num2str(file_num),'.mat']);


for i = 1:LENGTH
    disp(i);
R = Rots(:,:,i);
xp = R*x;

% xp_rots = rots(:,:,i)*x;

if flag == 0
    subplot(2,1,1)
    top = plot3(xp(1,itop), xp(2,itop), xp(3,itop), 'k-');
    hold on;
    bottom = plot3(xp(1,ibottom), xp(2,ibottom), xp(3,ibottom), 'k-');
    
    front = patch(xp(1,ifront), xp(2,ifront), xp(3,ifront), 'b');
    back = patch(xp(1,iback), xp(2,iback), xp(3,iback), 'r');
    hold off;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    axis([-2 2 -2 2 -2 2]);
    title('estimate')
    subplot(2,1,2)
%     
%     vicon_top = plot3(xp(1,itop), xp(2,itop), xp(3,itop), 'k-');
%     hold on;
%     vicon_bottom = plot3(xp(1,ibottom), xp(2,ibottom), xp(3,ibottom), 'k-');
%     
%     vicon_front = patch(xp(1,ifront), xp(2,ifront), xp(3,ifront), 'b');
%     vicon_back = patch(xp(1,iback), xp(2,iback), xp(3,iback), 'r');
%     hold off;
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     axis equal;
%     axis([-2 2 -2 2 -2 2]);
%      title('actual')

    flag =1;
    
else
    set(top,'xdata',xp(1,itop),'ydata',xp(2,itop),'zdata',xp(3,itop));
    set(bottom,'xdata',xp(1,ibottom),'ydata',xp(2,ibottom),'zdata',xp(3,ibottom));
    set(front,'xdata',xp(1,ifront),'ydata',xp(2,ifront),'zdata',xp(3,ifront));
    set(back,'xdata',xp(1,iback),'ydata',xp(2,iback),'zdata',xp(3,iback));
    
%     set(vicon_top,'xdata',xp_rots(1,itop),'ydata',xp_rots(2,itop),'zdata',xp_rots(3,itop));
%     set(vicon_bottom,'xdata',xp_rots(1,ibottom),'ydata',xp_rots(2,ibottom),'zdata',xp_rots(3,ibottom));
%     set(vicon_front,'xdata',xp_rots(1,ifront),'ydata',xp_rots(2,ifront),'zdata',xp_rots(3,ifront));
%     set(vicon_back,'xdata',xp_rots(1,iback),'ydata',xp_rots(2,iback),'zdata',xp_rots(3,iback));
%     
end

drawnow

end
clear flag
end
