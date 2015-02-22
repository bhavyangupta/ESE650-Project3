%%
% init_UKF.m
% Bhavya
%
% Input: none
%        
%
% Output: initsialises the mean and covariance matrices.
%         
%       
% Usage: run directly from command window - no inputs

%%
global noise_mean_process
global noise_cov_process
global noise_cov_measurement
global noise_mean_measurement
global state_cov 
global delta_t
global x_init
%% Process Noise distribution moments
 noise_mean_process = zeros(6,1);
 noise_cov_process = 10e-3* [10000,0,0,0,0,0;...
                             0,10000,0,0,0,0;...
                             0,0,10000,0,0,0;...
                             0,0,0,5,0,0;...
                             0,0,0,0,5,0;...
                             0,0,0,0,0,5];

%% Measurement Noise distribution moments
 noise_cov_measurement = 10e-3* [1,0.00001,0.00001,0.00001,0.00001,0.00001;...
                                  0.00001,1,0.00001,0.00001,0.00001,0.00001;...
                                  0.00001,0.00001,1,0.00001,0.00001,0.00001;...
                                  0.00001,0.00001,0.00001,10,0.00001,0.00001;...
                                  0.00001,0.00001,0.00001,0.00001,10,0.00001;...
                                  0.00001,0.00001,0.00001,0.00001,0.00001,10;...
                                  ];
 
 noise_mean_measurement = [0,0,0,0,0,0];
 
%% State covariance matrix 
 state_cov =10e-1 *  [0.01,0.0000001,0.0000001,0.0000001,0.0000001,0.0000001;...
                     0.0000001,0.01,0.0000001,0.0000001,0.0000001,0.0000001;...
                     0.0000001,0.0000001,0.01,0.0000001,0.0000001,0.0000001;...
                     0.0000001,0.0000001,0.0000001,10,0.0000001,0.0000001;...
                     0.0000001,0.0000001,0.0000001,0.0000001,10,0.0000001;...
                     0.0000001,0.0000001,0.0000001,0.0000001,0.0000001,10];

 %% Loop rate
 delta_t = 0.01; % seconds

 %% State initialisation
q_init  = [1;0;0;0]; % unit quatertion
w_init = [0.00001;0.00;0.0];
x_init = [q_init;w_init];
