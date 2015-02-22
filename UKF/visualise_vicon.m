%%
% visualise_vicon.m
% Bhavya
%
% Input: viconRot
%          rots: 3x3xnumberofsamples 
%
% Output: visualisation using rotplot.m 
%
% Usage: set the file number and run directly from command window 

FILE_NUMBER = 1;
source_name = ['viconRot',num2str(FILE_NUMBER),'.mat'];

load(source_name);
[~,~,LENGTH] = size(rots);
figure;
flag = 0;

rotplot(rots,LENGTH); % just send the entire thing to rotplot - modified to
                      % handle plots the way i want to 

