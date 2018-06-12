close all

addpath('~/Dropbox/3DVision/renderer/data');

%% Set resolution
width = 100;

%% Set magnitude of gamma, acceleration, translation and rotation 
% if vectors given: all possible combinations of values will be rendered
gamma_values = 0.8;
k_values = 1;
w_values = 0;
v_values = 0.02;

%% Generate data
start_generating(width, gamma_values, k_values, w_values, v_values);

