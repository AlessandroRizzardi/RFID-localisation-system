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
tag_position = generateRandomPointInCircle([0,0], radius_map - max_range);
nRobots = 4;
KL = 0.01*10^-2; % noise parameters of odometry
KR = 0.01*10^-2;
sigma_phi = 0.2; % [rad] standard deviation of the angle measurement
msg = 50;        %number of consensus protocol messages exchanged by the nodes in the network
CONSENSUS = true;
ANIMATION = false;
PLOTS = true;
%%%%%%%%%%  END SETTINGS    %%%%%%%%%%%%%

initialization

RFID_identification_algorithm

fprintf('           END SIMULATION\n')

%%
if ANIMATION == true
    animation
end
fprintf('           END ANIMATION\n')
%%
if PLOTS == true
    plots
end

ERROR_x = abs(robots(1).best_tag_estimation(1) - tag_position(1));
ERROR_y = abs(robots(1).best_tag_estimation(2) - tag_position(2));

fprintf('Estimation error along x: %d cm\n',round(ERROR_x,3)*100);
fprintf('Estimation error along y: %d cm\n',round(ERROR_y,3)*100);
