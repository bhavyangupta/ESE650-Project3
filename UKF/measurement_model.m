%%
% measurement_model.m
% Bhavya
%
% Input: state, noise_flag - set this if you want to include noise
%        
%
% Output: measurement likelihood
%         
%         
% Usage: 

function [measurement_likelihood] = measurement_model(x_t,noise_flag) 
%% Distribution moments in init_UKF
global noise_cov_measurement
global noise_mean_measurement
gravity_true = [0;0;-1;0]; % quaternion form
%% Gyroscope model
if noise_flag == 1

H1_t = x_t(5:end) + ( mvnrnd(noise_mean_measurement,noise_cov_measurement)') ;

else
H1_t = x_t(5:end);

end
%measurement_likelihood = H1_t;
%% Accelerometer model
gravity_expected = qMul(x_t(1:4),gravity_true);
gravity_expected = qMul(gravity_expected,qInv(x_t(1:4)));

% gravity_expected = qMul(qInv(x_t(1:4)),gravity_true);
% gravity_expected = qMul(gravity_expected,x_t(1:4));


H2_t = gravity_expected;
%% Stack and return
measurement_likelihood =[ H1_t; H2_t(2:end)];
end
