%%
% HMM_train_circle.m
% Bhavya 
% Trains HMM for circle gesture
%
%% Initialize
HMM_init_all;
global Xi;
global Xi_accum;
global gamma;
global gamma_accum;
global NUM_STATES;
global NUM_CENTROIDS;
B_bar = zeros(NUM_CENTROIDS,NUM_STATES);
gamma_first = zeros(1,NUM_STATES);

%% Load observations
load ../Data/train/circle/circle_codebook
load ../Data/train/circle/circle_train_discrete
%% Replace observations with indices
observations = zeros(length(gyro_filtered),1);
T = length(observations);

for i = 1:length(observations)
    gyro_filtered(i,:);
    if gyro_filtered(i,:) == codebook(1,4:end)
        observations (i,1) = 1;
    elseif gyro_filtered(i,:) == codebook(2,4:end)
        observations (i,1) = 2;
    elseif gyro_filtered(i,:) == codebook(3,4:end)
        observations(i,1) = 3;
    end
end

%% Reestimation


iter = 1;
while iter<30
    iter
    [likelihood,alpha] = forward_prop (observations(:),true);
    beta = backward_prop (observations(:));
    
    for t = 1:T-1
        x = bsxfun(@times,A,alpha(t,:)');
        y = bsxfun(@times,x,B(observations(t+1),:));
        Xi =  bsxfun(@times,y,beta(t+1,:));
        Xi_accum = Xi_accum + Xi;
        gamma = sum(Xi,1);
        if t ==1
            gamma_first = gamma;
        end
        gamma_accum = gamma_accum + gamma;
        B_bar(observations(t),:) = B_bar(observations(t),:) + gamma;
        % image(Xi)
    end
    
    pie = gamma_first /sum(gamma_first);
    A = bsxfun(@rdivide,Xi_accum,gamma_accum);
    gamma_accum = gamma_accum + (alpha(T,:).*beta(T,:))/sum(alpha(T,:).*beta(T,:)) ;
     B = bsxfun(@rdivide, B_bar,gamma_accum);
    B = bsxfun(@rdivide,B,sum(B,1))
    A = bsxfun(@rdivide,A, sum(A,2))
    iter = iter+ 1;
end