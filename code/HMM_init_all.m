%%
% HMM_init_all.m
% Bhavya 
% Initialises paramaters common to all HMMs
%% Load codebook
load ../Data/train/circle/circle_codebook.mat
codebook = centroids;
clear centroids;
%% Declare
global A
global B
global NUM_STATES 
global NUM_MEASUREMENTS
global NUM_CENTROIDS
global Xi
global pie
global c
global D
global gamma
global gamma_accum
global Xi_accum
%% Define
NUM_MEASUREMENTS = 6;
NUM_STATES = 3;
NUM_CENTROIDS = size(codebook,1);
 pie = rand(1,NUM_STATES);
% pie = zeros(1,NUM_STATES);
% pie(1) = 1;
pie = pie/sum(pie);
A = [0.9, 0.5, 0;...
     0, 0.5, 0.5; ...
     0.1, 0, 0.5];
B = rand(NUM_CENTROIDS,NUM_STATES); % elements refer to the probabilities
%  Normalise B:
B = bsxfun(@rdivide,B,sum(B,1))
Xi = zeros(NUM_STATES,NUM_STATES);
Xi_accum = zeros(NUM_STATES,NUM_STATES);
c = [];
D = [];
gamma = zeros(1,NUM_STATES);
gamma_accum = zeros(1,NUM_STATES);
