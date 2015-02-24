%%
% forward_prop.m
% Bhavya 
% Forward procedure 
% Arguments: observation sequence 1xT array, flag that tells which c to use
% Dependency: HMM_init_all for model params
%% 
function [likelihood,alpha_bar]= forward_prop(observations,init_c_flag)
global A
global B
global pie
global NUM_STATES
global c
global D
T = size(observations,1);
% alpha = zeros(T,NUM_STATES);
% 
% alpha(1,:) = pie .* B(observations(1),:) ;

% for t = 1:T-1
% %     disp(t)
%     alpha(t+1,:)  = sum(bsxfun(@times,A,alpha(t,:) ),1 ).* B(observations(t+1),:);
% % bsxfun(@times,A,alpha(t,:) ) 
% % B(observations(t+1),:)
% % break
% end
alpha_bar = zeros(T,NUM_STATES);
alpha_hat = zeros(T,NUM_STATES);
if init_c_flag
c= zeros(T,1);
end

alpha_bar(1,:) = pie .* B(observations(1),:);
% alpha_bar(1,:) = alpha_bar(1,:)/sum(alpha_bar(1,:)); % Normalise across states
if init_c_flag
c(1) = 1/sum(alpha_bar(1,:));
end
alpha_hat(1,:) = c(1) * alpha_bar(1,:);

for  t = 1:T-1
    x = bsxfun(@times,A',alpha_hat(t,:));
    y = bsxfun(@times,x,B(observations(t),:)');
    
    alpha_bar(t+1,:) = sum(y,2)';
%      alpha_bar(t+1,:) = alpha_bar(t+1,:)/sum(alpha_bar(t+1,:));
    if init_c_flag
    c(t+1) = 1/sum(alpha_bar(t+1,:));
    end
    
    alpha_hat(t+1,:) = c(t+1) * alpha_bar(t+1,:);
    D(t) = prod(c(t:T));
end

D(T) = c(T);

% likelihood = sum(alpha_hat(T,:));
log_likelihood  = -sum(log(c));
likelihood = log_likelihood;
% likelihood = sum(alpha(T,:));

end