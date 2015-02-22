%%
% sigma_pts.m
% Bhavya
%
% Input: A priori estimate of state nX1
%
% Output: Set of Projected a priori states nxn
%             
%
% Usage: 
function [X_i] = sigma_pts(x_bar_t_plus_1)
global state_cov
global noise_cov_process

S =   (state_cov + noise_cov_process);
S =  sqrt(12)*chol(S); % diagonal atrix
% W = x_bar_t_plus_1 + S;
% X_i = [W, x_bar_t_plus_1 - S];

q_k = x_bar_t_plus_1(1:4);
w_k = x_bar_t_plus_1(5:end);
[~,set_size] = size(S);
X_i = zeros(7,2*set_size);

for i = 1:set_size
%     temp = S(1:3,i);
%     temp_angle= norm(temp);
%     temp_e = temp/temp_angle;
%     temp = [cos(temp_angle/2);temp_e(1) * sin(temp_angle/2);temp_e(2) * sin(temp_angle/2) ; temp_e(3) *  sin(temp_angle/2)];
    temp = vect_to_quat(S(1:3,i));
    X_i(1:4,i) =qNormalize( qMul(q_k',temp')');
    X_i(5:end,i) = w_k + S(4:end,i);
    
%     temp = -S(1:3,i);
%     temp_angle= norm(temp);
%     temp_e = temp/temp_angle;
%     temp = [cos(temp_angle/2);temp_e(1) * sin(temp_angle/2);temp_e(2) * sin(temp_angle/2) ; temp_e(3) *  sin(temp_angle/2)];
    temp = vect_to_quat(S(1:3,i));
     temp(2:end) = -temp(2:4);
    X_i(1:4,i+set_size) = qNormalize(qMul(q_k',temp')');
    X_i(5:end,i+set_size) = w_k - S(4:end,i);
    
end

end
