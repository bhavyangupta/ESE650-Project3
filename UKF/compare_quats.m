%%
% compare_quats.m
% Bhavya
%
% Plots the vicon expected quaternion components vs estimated(red)
% Usage:  Run UKF.m before running this file

load('viconRot1')
%load R_out.mat
figure
subplot(2,2,1)
actual_1 = plot(0,0);
axis([0,T_sim,-1.5,1.5])
hold on
est_1 = plot(0,0,'r');
axis([0,T_sim,-1.5,1.5])
ylabel('q(1)')

subplot(2,2,2)
actual_2 = plot(0,0);
axis([0,T_sim,-1.5,1.5])
hold on
est_2 = plot(0,0,'r');
axis([0,T_sim,-1.5,1.5])
ylabel('q(2)')

subplot(2,2,3)
actual_3 = plot(0,0);
axis([0,T_sim,-1.5,1.5])
hold on
est_3 = plot(0,0,'r');
axis([0,T_sim,-1.5,1.5])
ylabel('q(3)')

subplot(2,2,4)
actual_4 = plot(0,0);
axis([0,T_sim,-1.5,1.5])
hold on
est_4 = plot(0,0,'r');
axis([0,T_sim,-1.5,1.5])
ylabel('q(4)')

q1_hist = [];
q2_hist = [];
for i = 1:T_sim
    i
    q1 = qGetQ(rots(:,:,i));
    q1_hist = [q1_hist,q1];
    q2 = qGetQ(R_out(:,:,i));
    q2_hist = [q2_hist,q2];
    set(actual_1,'ydata',q1_hist(1,:),'xdata',1:i);
    set(actual_2,'ydata',q1_hist(2,:),'xdata',1:i);
    set(actual_3,'ydata',q1_hist(3,:),'xdata',1:i);
    set(actual_4,'ydata',q1_hist(4,:),'xdata',1:i);
   
    set(est_4,'ydata',q2_hist(4,:),'xdata',1:i);
    set(est_3,'ydata',q2_hist(3,:),'xdata',1:i);
    set(est_2,'ydata',q2_hist(2,:),'xdata',1:i);
    set(est_1,'ydata',q2_hist(1,:),'xdata',1:i);
     drawnow
    
end
