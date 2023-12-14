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

KL = 0.01*10^-2; % noise parameters of odometry
KR = 0.01*10^-2;
sigma_phi = 0.2; % [rad] standard deviation of the angle measurement
msg = 50;        %number of consensus protocol messages exchanged by the nodes in the network
CONSENSUS = false;
ANIMATION = false;
PLOTS = true;
%%%%%%%%%%  END SETTINGS    %%%%%%%%%%%%%

TEST = 2; % 1: Monte-Carlo simulation with single robot
          % 2: Test with single robot over different odometry noise
          % 3: Test with single robot over different phase error
          % 4: Test with different swarm dimension

% Monte-Carlo simulation with single robot to obtained average accuracy
if TEST == 1
    fprintf('Performing different simulations with single robot\n')
    nRobots = 1;
    CONSENSUS = false;
    nExp = 10;
    mean_x = ones(nExp,1);
    mean_y = ones(nExp,1);
    mean_dist = ones(nExp,1);

    var_x = ones(nExp,1);
    var_y = ones(nExp,1);
    var_dist = ones(nExp,1);
    exp= 1:nExp;

    for trial=1:nExp
        error_x = [];
        error_y = [];
        error_dist = [];

        initialization
        RFID_identification_algorithm

        clean_estimation_history = remove_NaN_values(robots(1).tag_estimation_history); % remove NaN values

        for i = 1:length(clean_estimation_history)
            error_x(i) = clean_estimation_history(i,1)-tag_position(1)*10^2;
            error_y(i) = clean_estimation_history(i,2)-tag_position(2)*10^2;
            error_dist(i) = sqrt(error_x(i)^2+error_y(i)^2)*10^2;

        end

        error_x = abs(error_x);
        error_y = abs(error_y);

        mean_x(trial) = mean(error_x(end-1800:end));
        mean_y(trial) = mean(error_y(end-1800:end));
        mean_dist(trial) = mean(error_dist(end-1800:end));

        var_x(trial) = sum((error_x - mean_x(trial)).^2)/length(error_x);
        var_y(trial) = sum((error_y - mean_y(trial)).^2)/length(error_y);
        var_dist(trial) = sum((error_dist - mean_dist(trial)).^2)/length(error_dist);

        fprintf('--------- Trial: %d ---------\n',trial);
    end

    figure (1)
    subplot(3,1,1)
    grid on
    plot(exp,mean_x,'bo','MarkerSize',10)  
    hold on   
    yline(mean(mean_x),'r--','LineWidth',2)         
    title('Error for X coordinate');                            
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend('Error','Mean Error','Location','best');
    hold off
    
    subplot(3,1,2)
    grid on
    plot(exp,mean_y,'bo');
    hold on
    yline(mean(mean_y), 'r--', 'LineWidth', 2)
    title('Error for Y coordinate');
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend('Error','Mean Error','Location','best');
    hold off

    subplot(3,1,3)
    grid on
    hold on
    plot(exp,mean_dist,'bo');
    yline(mean(mean_dist), 'r--', 'LineWidth', 2)
    title('Distance Error');
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend('Error','Mean Error','Location','best');

    t = fprintf('Estimation Errors along %d experiments', nExp);
    sgtitle(t);  

    hold off

    fprintf('Mean error along x: %d\n', round(mean(mean_x),3));
    fprintf('Mean error along y: %d\n', round(mean(mean_y),3));
    fprintf('Mean error distance error: %d\n', round(mean(mean_dist),3));
    

    figure(2)
    subplot(3,1,1)
    plot(error_x, 'LineWidth', 2)
    xlabel('Iterations');
    ylabel('Error [cm]');
    title('Error for X coordinate');   

    subplot(3,1,2)
    plot(error_y, 'LineWidth', 2)
    title('Error for Y coordinate');
    xlabel('Iterations');
    ylabel('Error [cm]');

    subplot(3,1,3)
    plot(error_dist, 'LineWidth', 2)
    xlabel('Iterations');
    ylabel('Error [cm]');
    title('Distance Error');

    sgtitle('Tendency of Estimation Errors');
end


% Test with single robot over different odometry noise
if TEST == 2
    fprintf('Performing different simulations with single robot and different encoder readings noises\n')
    nRobots = 1;
    CONSENSUS = false;
    nExp = 10;
    mean_x = ones(nExp,1);
    mean_y = ones(nExp,1);
    mean_dist = ones(nExp,1);

    var_x = ones(nExp,1);
    var_y = ones(nExp,1);
    var_dist = ones(nExp,1);
    
    K_list = [0.01*10^-1, 0.01*10^-2, 0.01*10^-3, 0.01*10^-4];

    for exp=1:length(K_list)
        KR = K_list(exp);
        KL = K_list(exp);

        for trial=1:nExp
            error_x = [];
            error_y = [];
            error_dist = [];
    
            initialization
            RFID_identification_algorithm
    
            clean_estimation_history = remove_NaN_values(robots(1).tag_estimation_history); % remove NaN values
    
            for i = 1:length(clean_estimation_history)
                error_x(i) = clean_estimation_history(i,1)-tag_position(1)*10^2;
                error_y(i) = clean_estimation_history(i,2)-tag_position(2)*10^2;
                error_dist(i) = sqrt(error_x(i)^2+error_y(i)^2)*10^2;
    
            end
            error_x = abs(error_x);
            error_y = abs(error_y);
    
            mean_x(trial) = mean(error_x(end-1800:end));
            mean_y(trial) = mean(error_y(end-1800:end));
            mean_dist(trial) = mean(error_dist(end-1800:end));

            var_x(trial) = sum((error_x - mean_x(trial)).^2)/length(error_x);
            var_y(trial) = sum((error_y - mean_y(trial)).^2)/length(error_y);
            var_dist(trial) = sum((error_dist - mean_dist(trial)).^2)/length(error_dist);

            fprintf('--------- Trial: %d ---------\n',trial);
        end
        
        mean_x_error(exp) = mean(mean_x);
        mean_y_error(exp) = mean(mean_y);
        mean_dist_error(exp) = mean(mean_dist);

        fprintf('Exp with K = %d has mean error along x equal to %d cm with variance %d cm^2\n', K_list(exp), mean(mean_x), mean(var_x));
        fprintf('Exp with K = %d has mean error along y equal to %d cm with variance %d cm^2\n', K_list(exp), mean(mean_y), mean(var_y));
        fprintf('Exp with K = %d has mean distance error equal to %d cm with variance %d cm^2\n', K_list(exp), mean(mean_dist), mean(var_dist));

    end

    % Vector of different colors
    line_color = ['r','b','g','m','y','c'];
%%
    figure (1)
    subplot(3,1,1)
    grid on
    leg = {};
    for i = 1:length(K_list)
        yline(mean_x_error(i),line_color(i),'LineWidth',2) 
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error for X coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off
    
    subplot(3,1,2)
    grid on
    leg = {};
    for i = 1:length(K_list)
        yline(mean_y_error(i),line_color(i),'LineWidth',2) 
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error for X coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off

    subplot(3,1,3)
    grid on
    leg = {};
    for i = 1:length(K_list)
        yline(mean_dist_error(i),line_color(i),'LineWidth',2) 
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error for X coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off

   sgtitle('Estimation Errors along numerous simulations for different encoder readings noises');  

end 

if TEST == 3
    nRobots = 1;
    CONSENSUS = false;
    nExp = 10;
    mean_x = ones(nExp,1);
    mean_y = ones(nExp,1);
    mean_dist = ones(nExp,1);

    var_x = ones(nExp,1);
    var_y = ones(nExp,1);
    var_dist = ones(nExp,1);
    phase_error_list = [0, 0.1, 0.2 , 0.3];

    for phi_error = 1:length(phase_error_list)

        sigma_phi = phase_error_list(phi_error);

        for exp=1:nExp
            initialization
            RFID_identification_algorithm

        end



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