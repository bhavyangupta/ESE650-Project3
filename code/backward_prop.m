%%
% forward_prop.m
% Bhavya 
% Backward procedure 
% Arguments: observation sequence 1xT array
% Dependency: HMM_init_all for model params
%%
function beta = backward_prop(observations)
global A
global B
global pie
global NUM_STATES
global c
T = size(observations,1);
% beta = zeros(T,NUM_STATES);
% beta(T,:) = ones(1,NUM_STATES);
% % for t = T-1:-1:1
%     disp(t)
% %     beta(t,:)= sum(((bsxfun(@times,A,B(observations(t+1,:)))) .* beta(t+1,:)),1 );
% x = bsxfun(@times,A,B(observations(t+1,:)'));
% y= bsxfun(@times, x, beta(t+1,:)');
% beta(t,:) = (sum(y,2))'
% 
% end

beta_bar = zeros(T,NUM_STATES);
beta_hat = zeros(T,NUM_STATES);

beta_hat(T,:) = ones(1,NUM_STATES);
beta_bar(T,:) = c(T) * beta_hat(T,:);
beta_bar(T,:) = beta_bar(T,:)/sum(beta_bar(T,:));
% beta_bar(T,:) = beta_hat(T,:);
for t = (T-1):-1:1
%     disp(t)
    x = bsxfun(@times,A,B(observations(t+1,:)));
    y= bsxfun(@times, x, beta_hat(t+1,:)');
    beta_bar(t,:) = sum(y,1);
%      beta_bar(t,:) = beta_bar(t,:)/sum(beta_bar(t,:));
    beta_hat(t,:) = c(t) * beta_bar(t,:);
end
beta = beta_bar;

end
