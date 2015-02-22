%%
% process_model.m
% Bhavya
%
% Input: previous state, noise_flag - set this if you want to include
% noise
%        
%
% Output: a priori state belief of next state
%         
%         
% Usage: 


function [priori_belief] = process_model(x_t,noise_flag)
%% Distribution moments in init_UKF
global noise_mean_process
global noise_cov_process
global delta_t

%% Parse parameters
w_t =  x_t(5:end);
q_t = x_t(1:4);
%% Incremental quaternion change
 %disp(norm(w_t));
 alpha_del = norm(w_t) * delta_t;
 %disp(rad2deg(alpha_del));
 e_del = w_t/norm(w_t); 
 q_del = [cos(alpha_del/2);e_del(1)*sin(alpha_del/2);e_del(2) * sin(alpha_del/2);e_del(3) * sin(alpha_del/2)];
    
 if noise_flag == 1
     noise_process = (mvnrnd(noise_mean_process,noise_cov_process))';
     noise_omega = noise_process(4:end);
     noise_rotation_vector = noise_process(1:3);
     alpha_w = norm(noise_rotation_vector);
     e_w = noise_rotation_vector/alpha_w ;
     q_w = [cos(alpha_w/2);e_w(1) * sin(alpha_w/2);e_w(2) * sin(alpha_w/2);e_w(3) * sin(alpha_w/2)];
 else 
     q_w = [1;0;0;0];
     noise_omega = [0;0;0];
 end
 
 q_t = (qMul(q_t,q_w));
 temp = (qMul(q_t,q_del));
 temp = qNormalize(temp);
 priori_belief = [temp; w_t + noise_omega];

end
