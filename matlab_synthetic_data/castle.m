close all

addpath('~/Dropbox/3DVision/renderer/data');

%% Set resolution
width = 600;

%% Set magnitude of gamma, acceleration, translation and rotation 
% if vectors given: all possible combinations of values will be rendered

% Data for figure 4 (left)
k_values = 0;
w_values = 0;
v_values = 0.03;
gamma_values = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1];
start_generating(width, gamma_values, k_values, w_values, v_values);

% Data for figure 4 (middle)
k_values = [0, 0.5, 1, 1.5];
w_values = 0.05;
v_values = 0.02;
gamma_values = 0.8;
start_generating(width, gamma_values, k_values, w_values, v_values);

% Data for figure 4 (right)
k_values = 0;
w_values = 0.05;
v_values = [0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06];
gamma_values = 0.8;
start_generating(width, gamma_values, k_values, w_values, v_values);

