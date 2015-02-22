%%
% UKF.m
% Bhavya
%
% Input: 
%        
%
% Output: 
%         
%         
%
% Usage: run directly from command window - no inputs
%% File parameters
FILE_NUMBER = 1;
%  source_name = 'imu_processed.mat';
source_name = 'circle12_raw_data.mat';
load(source_name);
%gyro_out = bsxfun(@plus,zeros(3,1000),[0.1;0;0]);
[~,T_sim] = size(gyro_out);

%% Initialisation 
init_UKF; % initialises the covariances and means
NUM_MEASUREMENTS = 6;
global state_cov;
global x_init;

 x_t = x_init;
 R_out =  zeros(3,3,T_sim);
% handle_innovation = plot3(0,0,0,'x');
% xlabel('X');
% xlabel('Y');
% xlabel('Z');
% grid on
% axis ([-1,1,-1,1,-1,1]);
 innovation_history = [];
% 
% figure
% handle_omega_estimate = plot3(0,0,0,'x');
% hold on
% handle_omega_actual = plot3(0,0,0,'o','Color','r');
% xlabel('X');
% xlabel('Y');
% xlabel('Z');
% grid on
% axis ([-1,1,-1,1,-1,1]);
 omega_history = [];

figure
subplot(3,1,1)
gyro_x = plot(0,0);
axis([0,2000,-2,2]);
hold on  
est_x = plot(0,0,'r');
axis([0,2000,-2,2]);
ylabel('Gyro X')
grid on

subplot(3,1,2)
gyro_y = plot(0,0);
axis([0,2000,-2,2]);
hold on  
est_y = plot(0,0,'r');
axis([0,2000,-2,2]);
ylabel('Gyro Y ')
grid on

subplot(3,1,3)
gyro_z = plot(0,0);
axis([0,2000,-2,2]);
hold on  
est_z = plot(0,0,'r');
axis([0,2000,-2,2]);
ylabel('Gyro z')
grid on

figure
subplot(3,1,1)
accel_x = plot(0,0);
axis([0,2000,-2,2]);
hold on
est_accel_x = plot(0,0);
axis([0,2000,-2,2]);
ylabel('accel x')
grid on

subplot(3,1,2)
accel_y = plot(0,0);
axis([0,2000,-2,2]);
hold on
est_accel_y = plot(0,0);
axis([0,2000,-2,2]);
ylabel('accel y')
grid on

subplot(3,1,3)
accel_z = plot(0,0);
axis([0,2000,-2,2]);
hold on
est_accel_z = plot(0,0);
axis([0,2000,-2,2]);
ylabel('accel z')
grid on

accel_history =[];

for t = 1:T_sim
    disp(t)
    
     accel_out(:,t) = (accel_out(:,t)/norm(accel_out(:,t)));
%      accel_out(1:2,t) = -(accel_out(1:2,t));
   % accel_out(:,t) = accel_out(:,t);
    Z_t =[ gyro_out(:,t);accel_out(1,t);accel_out(2,t);accel_out(3,t)];
    q_t = x_t(1:4);
    
    %sdisp((2* acos(q_t(1))));
    w_t = x_t(5:end);
    %disp([Z_t,w_t]);
    omega_history = [omega_history,w_t];
    
    %set(handle_omega_estimate,'xdata',omega_history(1,:),'ydata',omega_history(2,:),'zdata',omega_history(3,:));
    %set(handle_omega_actual,'xdata',gyro_out(1,1:t),'ydata',gyro_out(2,1:t),'zdata',gyro_out(3,1:t));
    
    set(accel_x,'ydata',accel_out(1,1:t),'xdata',1:t)
    set(accel_y,'ydata',accel_out(2,1:t),'xdata',1:t)
    set(accel_z,'ydata',accel_out(3,1:t),'xdata',1:t)
    
    set(gyro_x,'ydata',gyro_out(1,1:t),'xdata',1:t);
    set(gyro_y,'ydata',gyro_out(2,1:t),'xdata',1:t);
    set(gyro_z,'ydata',gyro_out(3,1:t),'xdata',1:t);
    
    set(est_x,'ydata',omega_history(1,:),'xdata',1:t);
    set(est_y,'ydata',omega_history(2,:),'xdata',1:t);
    set(est_z,'ydata',omega_history(3,:),'xdata',1:t);
    drawnow;
    R_out(:,:,t) = qGetR(q_t); % for visualisation purposes
    
    
    X_i = sigma_pts(x_t);
    [~,set_size]  = size(X_i);
%% code to check if mean of sigma pts is actually equal to the state value
     M = zeros(size(X_i(1:4,1)*X_i(1:4,1)')); % should be square
     for i = 1:set_size
     M = M + X_i(1:4,i)*X_i(1:4,i)';
     end
       eig_val = eig(M/set_size);
     [~,idx] = max(eig_val);
     [x_t_hat_bar,eig_val] = eig(M/set_size);
     x_t_hat_bar = x_t_hat_bar(:,idx);
      x_t_hat_bar(1:4) = qNormalize(x_t_hat_bar(1:4));
 %% Unscented Transformed states 
    Y_i = zeros(size(X_i));
    
    for i = 1:set_size
    Y_i(:,i) = process_model(X_i(:,i),0); % disable noise
    end
    M = zeros(size(Y_i(1:4,1)*Y_i(1:4,1)')); % should be square
   
    Z_i = zeros(NUM_MEASUREMENTS,set_size); % +1 added 
%% Compute mean of transformed a priori estimate
     for i = 1:set_size
     M = M + Y_i(1:4,i)*Y_i(1:4,i)';
     Z_i(:,i) = measurement_model(Y_i(:,i),0); % also, to save a loop,compute measurement update
     end
%     eig_val = eig(M/set_size);
%     [~,idx] = max(eig_val);
%     [x_t_hat_bar,eig_val] = eig(M/set_size);
%     x_t_hat_bar = x_t_hat_bar(:,idx);
%     x_t_hat_bar(1:4) = qNormalize(x_t_hat_bar(1:4));
 %     M = Y_i(1:4,:) * Y_i(1:4,:)';
      eig_val = eig(M/set_size);
     [~,idx] = max(eig_val);
     [x_t_hat_bar,eig_val] = eig(M/set_size);
     x_t_hat_bar = x_t_hat_bar(:,idx);
      x_t_hat_bar(1:4) = qNormalize(x_t_hat_bar(1:4)) ;
      x_t_hat_bar = [x_t_hat_bar;mean(Y_i(5:end,:),2)];
%% PROBLEM WAS HERE!!      
    % ** Compute transformed Belief covariance **
     M = zeros(6,6);
     W_t_prime = zeros(6,set_size);
    for i = 1:set_size
        omega_w_prime = Y_i(5:end,i) - x_t_hat_bar(5:end);
        r_w_prime = (qMul(Y_i(1:4,i) , qInv(x_t_hat_bar(1:4))));
    %    r_w_prime = qNormalize(Y_i(1:4,i) - x_t_hat_bar(1:4));  % WRONG!!
        W_t_prime(:,i) = [(quat_to_vect(r_w_prime));omega_w_prime];  %% CHECK THE TRUNCATION OF r_w_prime
        M = M + (W_t_prime(:,i) * W_t_prime(:,i)');
    end
    
    P_k_bar = (1/set_size)*M; % 
%     figure 
%      pcolor(P_k_bar);

%%  Compute mean of expected measurement
%     M  = zeros(3,3)
%     for i= 1:set_size
%         M = M + Z_i(4:end,i) *  Z_i(4:end,i)';
%     end
%    eig_val = eig(M/set_size);
%      [~,idx] = max(eig_val);
%      [Z_t_bar,eig_val] = eig(M/set_size);
%      Z_t_bar = Z_t_bar(:,idx);
%      Z_t_bar = [Z_t_bar;mean(Z_i(1:3,:),2)];

     Z_t_bar = mean(Z_i,2);
%     Z_t_bar = measurement_model(x_t_hat_bar,0);
    Z_t_bar(4:end) = (Z_t_bar(4:end)/norm(Z_t_bar(4:end))) ;
%     disp('estimate')
%     disp(Z_t_bar(4:end));
    accel_history = [accel_history,Z_t_bar(4:end)];
    set(est_accel_x,'ydata',accel_history(1,:),'xdata',1:t)
    set(est_accel_y,'ydata',accel_history(2,:),'xdata',1:t)
    set(est_accel_z,'ydata',accel_history(3,:),'xdata',1:t)
    drawnow;
    
%%  Compute Covariance of expected measurment
    P_zz =cov(Z_i');
     
%% Compute Innovation
     innovation = Z_t - Z_t_bar;
     innovation_history = [innovation_history,innovation];
 
     P_vv = P_zz + noise_cov_measurement;

     P_xz  = zeros(6,NUM_MEASUREMENTS);
     for i = 1:set_size
         P_xz = P_xz + W_t_prime(:,i) * (Z_i(:,i) - Z_t_bar)';
     end
     P_xz = (1/set_size ) * (P_xz);
        
     Kalman_Gain_6d = P_xz * inv(P_vv);
     update_vector_6d = Kalman_Gain_6d * innovation;
     update_vector_7d = [(vect_to_quat(update_vector_6d(1:3)));update_vector_6d(4:6)];

     x_t = qMul(qNormalize(x_t_hat_bar(1:4)),qNormalize(update_vector_7d(1:4)));
     x_t = [x_t;x_t_hat_bar(5:end)+update_vector_7d(5:end)];
     x_t(1:4) = qNormalize(x_t(1:4));
%      figure;
%        pcolor(state_cov);
     state_cov = P_k_bar - (Kalman_Gain_6d * P_vv * (Kalman_Gain_6d') );
      
%        pcolor(state_cov);
%     
%     
%     x_bar_t_plus_1 = process_model(x_t,1,delta_t);
%     x_t = x_bar_t_plus_1;
%     
end
figure;
eulr = euler_from_rot(R_out);
subplot(3,1,1)
roll = plot(1:t,eulr(1,:));
subplot (3,1,2) 
pitch = plot(1:t,eulr(2,:));
subplot(3,1,3)
yaw = plot(1:t,eulr(3,:));