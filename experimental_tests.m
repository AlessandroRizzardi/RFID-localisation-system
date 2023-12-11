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
nRobots = 1;
KL = 0.01*10^-2; % noise parameters of odometry
KR = 0.01*10^-2;
sigma_phi = 0.2; % [rad] standard deviation of the angle measurement
msg = 50;        %number of consensus protocol messages exchanged by the nodes in the network
CONSENSUS = false;
ANIMATION = false;
PLOTS = true;
%%%%%%%%%%  END SETTINGS    %%%%%%%%%%%%%

TEST = 1;


if TEST == 1
    nExp = 10;
    x_error = ones(nExp,1);
    y_error = ones(nExp,1);
    dist_error = ones(nExp,1);
    exp = 1:nExp;

    for trial=1:nExp
        initialization
        RFID_identification_algorithm
        x_est = robots(1).best_tag_estimation(1);
        y_est = robots(1).best_tag_estimation(2);

        error_x = x_est-tag_position(1);
        error_y = y_est-tag_position(2);
        dist_error = sqrt(error_x^2+error_y^2);

        fprintf("dist_error:%d \n",round(dist_error,3));
        x_error(trial) = error_x;
        y_error(trial) = error_y;
        dist_error(trial) = dist_error;
    end

    figure
    hold on
    plot(exp,x_error,'bo');
    %plot(exp,y_error);
    %plot(exp,dist_error);
end