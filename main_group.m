close all;
clear;  
clc;

addpath('Functions')
addpath('Classes')
addpath('Scripts')


% Load the configuration parameters for the algorithms
config

%%%%%%%%%%  SETTINGS    %%%%%%%%%%%%%
% Define position of the tag
tag_position = [0,0];
% Define the number of robots you want to have
nRobots = 3;
KL = 0.01*10^-2; % noise parameters of odometry
KR = 0.01*10^-2;
sigma_phi = 0.2; % [rad] standard deviation of the angle measurement
msg = 50;        %number of consensus protocol messages exchanged by the nodes in the network

CONSENSUS = true;
ANIMATION = false;
DRAW = true;
%%%%%%%%%%  END SETTINGS    %%%%%%%%%%%%%

initialization

RFID_identification_algorithm

fprintf('           END SIMULATION\n')

%% Animation of the estimation algorithm
if ANIMATION == true
    animation
end
fprintf('           END ANIMATION\n')
%% Final plot situation
if DRAW == true
    final_plot
end

%ERROR_x = abs(robots(1).best_tag_estimation(1) - tag_position(1));
%ERROR_y = abs(robots(1).best_tag_estimation(2) - tag_position(2));
%
%fprintf('Estimation error along x: %d cm\n',round(ERROR_x,3)*100);
%fprintf('Estimation error along y: %d cm\n',round(ERROR_y,3)*100);
