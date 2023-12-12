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

% Monte-CArlo simulation with single robot to obtained average accuracy
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

    mean_x_error = mean(x_error);
    mean_y_error = mean(y_error);
    mean_dist_error = mean(dist_error);

    fprintf('Mean error along x: %d', mean_x_error);
    fprintf('Mean error along y: %d', mean_y_error);
    fprintf('Mean error in distance: %d', mean_dist_error);

    figure (1)
    hold on
    plot(exp,x_error,'bo');
    plot(mean_x_error,exp);
    title('Error in x');
    xlabel('Time [s]');
    ylabel('Error [m]');

    figure (2)
    hold on
    plot(exp,y_error,'bo');
    plot(mean_y_error,exp);
    title('Error in y');
    xlabel('Time [s]');
    ylabel('Error [m]');

    figure (3)
    hold on
    plot(exp,dist_error,'bo');
    plot(mean_dist_error,exp);
    title('Error in total distance');
    xlabel('Time [s]');
    ylabel('Error [m]');
    
end


% Test with single robot over different odometry noise
if TEST == 2
    K_list = [0.01*10^-1, 0.01*10^-2, 0.01*10^-3, 0.01*10^-4];

    for exp=1:length(K_list)
        KR = K_list(exp);
        KL = K_list(exp);

        initialization
        RFID_identification_algorithm
        x_est = robots(1).best_tag_estimation(1);
        y_est = robots(1).best_tag_estimation(2);

        error_x = x_est-tag_position(1);
        error_y = y_est-tag_position(2);
        dist_error = sqrt(error_x^2+error_y^2);

        fprintf('Error along x with K = %d : %d', KR, error_x);
        fprintf('Error along y with K = %d : %d', KR, error_y);
        fprintf('Error in distance with K = %d : %d', KR, dist_error);
    end
end 

if TEST == 3
    phase_error_list = [0, 0.1, 0.2 , 0.3];

    for exp = 1:length(phase_error_list)
        sigma_phi = phase_error_list(exp);

        initialization
        RFID_identification_algorithm

        x_est = robots(1).best_tag_estimation(1);
        y_est = robots(1).best_tag_estimation(2);

        error_x = x_est-tag_position(1);
        error_y = y_est-tag_position(2);
        dist_error = sqrt(error_x^2+error_y^2);

        fprintf('Error along x : %d', error_x);
        fprintf('Error along y : %d', error_y);
        fprintf('Error in distance: %d', dist_error);
    end


end

if TEST == 4
    swarm_dimension = [3,5,10,15,20,100];

    for exp = 1:length(swarm_dimension)
        nRobots = swarm_dimension(exp);
        CONSENSUS = true;

        initialization
        RFID_identification_algorithm

        x_est = robots(1).best_tag_estimation(1);
        y_est = robots(1).best_tag_estimation(2);

        error_x = x_est-tag_position(1);
        error_y = y_est-tag_position(2);
        dist_error = sqrt(error_x^2+error_y^2);

        fprintf('Error along x : %d',error_x);
        fprintf('Error along y : %d',error_y);
        fprintf('Error in distance: %d', dist_error);
    end

end