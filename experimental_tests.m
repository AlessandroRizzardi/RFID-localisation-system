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

nExp = 100; % Simulations to do for every test

TEST = 2;  % 1: Monte-Carlo simulation with single robot
           % 2: Test with single robot over different odometry noise
           % 3: Test with single robot over different phase error
           % 4: Test with different swarm dimension



% Monte-Carlo simulation with single robot to obtained average accuracy
if TEST == 1
    fprintf('!!!! TEST CASE: Performing different simulations with single robot !!!!\n')
    nRobots = 1;
    CONSENSUS = false;
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
            error_x(i) = (clean_estimation_history(i,1)-tag_position(1))*10^2;
            error_y(i) = (clean_estimation_history(i,2)-tag_position(2))*10^2;
            error_dist(i) = sqrt(error_x(i)^2+error_y(i)^2);

        end

        mean_x(trial) = mean(error_x(end-1800:end));
        mean_y(trial) = mean(error_y(end-1800:end));
        mean_dist(trial) = mean(error_dist(end-1800:end));

        fprintf('--------- Trial: %d ---------\n',trial);
    end

    figure (1)
    subplot(3,1,1)
    grid on
    plot(exp,mean_x,'bo')  
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

    sgtitle('Estimation Errors along %d experiments', nExp);  

    hold off

    fprintf('Mean error along x: %d\n', round(mean(mean_x),3));
    fprintf('Mean error along y: %d\n', round(mean(mean_y),3));
    fprintf('Mean error distance error: %d\n', round(mean(mean_dist),3));

    fprintf('Mean Squared Error along x: %d\n', round(sum((mean_x - mean(mean_x))^2)/length(mean_x),3));
    fprintf('Mean Squared Error along y: %d\n', round(sum((mean_y - mean(mean_y))^2)/length(mean_y),3));
    fprintf('Mean Squared Error distance error: %d\n', round(sum((mean_dist - mean(mean_dist))^2)/length(mean_dist),3));
    

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
    fprintf('!!!! TEST CASE: Performing different simulations with single robot and different encoder readings noises !!!!\n')
    nRobots = 1;
    CONSENSUS = false;
    mean_x = ones(nExp,1);
    mean_y = ones(nExp,1);
    mean_dist = ones(nExp,1);

    var_x = ones(nExp,1);
    var_y = ones(nExp,1);
    var_dist = ones(nExp,1);
    
    %K_list = [0.01*10^-1, 0.01*10^-2, 0.01*10^-3, 0.01*10^-4];
    K_list = [0.01*10^-2, 0.01*10^-3];

    for exp=1:length(K_list)
        KR = K_list(exp);
        KL = K_list(exp);
   
        fprintf(' ------ Processing Case: %d -----\n', exp)

        for trial=1:nExp
            error_x = [];
            error_y = [];
            error_dist = [];
    
            initialization
            RFID_identification_algorithm
    
            clean_estimation_history = remove_NaN_values(robots(1).tag_estimation_history); % remove NaNA values
    
            for i = 1:length(clean_estimation_history)
                error_x(i) = (clean_estimation_history(i,1)-tag_position(1))*10^2;
                error_y(i) = (clean_estimation_history(i,2)-tag_position(2))*10^2;
                error_dist(i) = sqrt(error_x(i)^2+error_y(i)^2);
    
            end
            
            mean_x(exp,trial) = mean(error_x(end-1500:end));
            mean_y(exp,trial) = mean(error_y(end-1500:end));
            mean_dist(exp,trial) = mean(error_dist(end-1500:end));

            fprintf('--------- Trial: %d ---------\n',trial);
        end
        
        
        %Mean squared error
        ms_error_x(exp) = sum((mean_x(exp,:) - mean(mean_x(exp,:))).^2)/length(mean_x(exp,:));
        ms_error_y(exp) = sum((mean_y(exp,:) - mean(mean_y(exp,:))).^2)/length(mean_y(exp,:));
        ms_error_dist(exp) = sum((mean_dist(exp,:) - mean(mean_dist(exp,:))).^2)/length(mean_dist(exp,:));
        
        fprintf('Exp with K = %d has mean error along x equal to %d cm  \n' , K_list(exp), round(   mean(mean_x(exp,:))    ,3) );
        fprintf('Exp with K = %d has mean error along y equal to %d cm  \n' , K_list(exp), round(   mean(mean_y(exp,:))    ,3) );
        fprintf('Exp with K = %d has mean distance error equal to %d cm \n' , K_list(exp), round(   mean(mean_dist(exp,:)) ,3) );

        fprintf('Mean Squared Error along x: %d\n'          , round(    ms_error_x(exp)     ,3) );
        fprintf('Mean Squared Error along y: %d\n'          , round(    ms_error_y(exp)     ,3) );
        fprintf('Mean Squared Error distance error: %d\n'   , round(    ms_error_dist(exp)  ,3) );

    end
    
    % Vector of different colors
    line_color = ['#D95319','#0072BD',"#77AC30","#EDB120","#4DBEEE","#7E2F8E"];

    figure (1)
    subplot(3,1,1)
    grid on
    leg = {};
    for i = 1:length(K_list)
        plot(1:nExp,mean_x(i,:),'bo', 'Color', line_color(i))  
        yline(mean(mean_x(i,:)),'Color',line_color(i),'LineWidth',2) 
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
        plot(1:nExp,mean_y(i,:),'bo','Color',line_color(i))  
        yline(mean(mean_y(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error for Y coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off

    subplot(3,1,3)
    grid on
    leg = {};
    for i = 1:length(K_list)
        plot(1:nExp,mean_dist(i,:),'bo','Color',line_color(i))  
        yline(mean(mean_dist(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error of distance estimates');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off

   sgtitle('Estimation Errors along numerous simulations for different encoder readings noises');  

end 


if TEST == 3
    fprintf('!!!! TEST CASE: Performing different simulations with single robot and different phase measurement error !!!!\n')
    
    nRobots = 1;
    CONSENSUS = false;
    mean_x = ones(nExp,1);
    mean_y = ones(nExp,1);
    mean_dist = ones(nExp,1);

    var_x = ones(nExp,1);
    var_y = ones(nExp,1);
    var_dist = ones(nExp,1);
    phase_error_list = [0.3, 0.2, 0.1, 0];

    for exp = 1:length(phase_error_list)

        sigma_phi = phase_error_list(exp);
    
        fprintf(' ------ Processing Case: %d -----\n', exp)

        for trial=1:nExp

            error_x = [];
            error_y = [];
            error_dist = [];
    
            initialization
            RFID_identification_algorithm
    
            clean_estimation_history = remove_NaN_values(robots(1).tag_estimation_history); % remove NaN values
    
            for i = 1:length(clean_estimation_history)
                error_x(i) = (clean_estimation_history(i,1)-tag_position(1))*10^2;
                error_y(i) = (clean_estimation_history(i,2)-tag_position(2))*10^2;
                error_dist(i) = sqrt(error_x(i)^2+error_y(i)^2);
    
            end
        
            mean_x(exp,trial) = mean(error_x(end-1500:end));
            mean_y(exp,trial) = mean(error_y(end-1500:end));
            mean_dist(exp,trial) = mean(error_dist(end-1500:end));

            % fprintf('--------- Trial: %d ---------\n',trial);
        end
        
        %Mean squared error
        ms_error_x(exp) = sum((mean_x(exp,:) - mean(mean_x(exp,:)))^2)/length(mean_x(exp,:));
        ms_error_y(exp) = sum((mean_y(exp,:) - mean(mean_y(exp,:)))^2)/length(mean_y(exp,:));
        ms_error_dist(exp) = sum((mean_dist(exp,:) - mean(mean_dist(exp,:)))^2)/length(mean_dist(exp,:));


        fprintf('Exp with sigma_phi = %d has mean error along x equal to %d cm ', phase_error_list(exp), mean(mean_x)       );
        fprintf('Exp with sigma_phi = %d has mean error along y equal to %d cm ', phase_error_list(exp), mean(mean_y)       );
        fprintf('Exp with sigma_phi = %d has mean distance error equal to %d cm', phase_error_list(exp), mean(mean_dist)    );

        fprintf('Mean Squared Error along x: %d\n'        , round(    ms_error_x(exp)     ,3) );
        fprintf('Mean Squared Error along y: %d\n'        , round(    ms_error_y(exp)     ,3) );
        fprintf('Mean Squared Error distance error: %d\n' , round(    ms_error_dist(exp)  ,3) );
        
    end

    % Vector of different colors
    line_color = ['#D95319','#0072BD',"#77AC30","#EDB120","#4DBEEE","#7E2F8E"];

    figure (1)
    subplot(3,1,1)
    grid on
    leg = {};
    for i = 1:length(phase_error_list)
        plot(1:nExp,mean_x,'bo','Color',line_color(i)) 
        yline(mean_x_error(i),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Phae Measurement Error: ', num2str(phase_error_list(i))]; 
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
    plot(1:nExp,mean_y,'bo','Color',line_color(i)) 
    for i = 1:length(phase_error_list)
        yline(mean_y_error(i),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Phae Measurement Error: ', num2str(phase_error_list(i))]; 
        hold on   
    end      
    title('Mean Error for Y coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off

    subplot(3,1,3)
    grid on
    leg = {};
    plot(1:nExp,mean_dist,'bo','Color',line_color(i)) 
    for i = 1:length(phase_error_list)
        yline(mean_dist_error(i),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Phae Measurement Error: ', num2str(phase_error_list(i))]; 
        hold on   
    end      
    title('Mean Error of distance estimates');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off

    sgtitle('Estimation Errors along numerous simulations for different phase measurement errors');  
end



if TEST == 4
    fprintf('!!!! TEST CASE: Performing different simulations with different number of robots !!!!\n')
    
    swarm_dimension = [3, 10];
    
    for exp = 1:length(swarm_dimension)
        nRobots = swarm_dimension(exp);
        CONSENSUS = true;
        
        fprintf(' ------ Processing Case: %d -----\n', exp)

        for trial=1:nExp
            error_x = [];
            error_y = [];
            error_dist = [];
    
            initialization
            RFID_identification_algorithm
    
            clean_estimation_history = remove_NaN_values(robots(1).tag_estimation_history); % remove NaN values
    
            for i = 1:length(clean_estimation_history)
                error_x(i) = (clean_estimation_history(i,1)-tag_position(1))*10^2;
                error_y(i) = (clean_estimation_history(i,2)-tag_position(2))*10^2;
                error_dist(i) = sqrt(error_x(i)^2+error_y(i)^2);
    
            end
    
            mean_x(exp,trial) = mean(error_x(end-1500:end));
            mean_y(exp,trial) = mean(error_y(end-1500:end));
            mean_dist(exp,trial) = mean(error_dist(end-1500:end));
    
            
    
            % fprintf('--------- Trial: %d ---------\n',trial);
        end

        %Mean squared error
        ms_error_x(exp) = sum((mean_x(exp,:) - mean(mean_x(exp,:)))^2)/length(mean_x(exp,:));
        ms_error_y(exp) = sum((mean_y(exp,:) - mean(mean_y(exp,:)))^2)/length(mean_y(exp,:));
        ms_error_dist(exp) = sum((mean_dist(exp,:) - mean(mean_dist(exp,:)))^2)/length(mean_dist(exp,:));

        fprintf('Exp with %d ROBOTS has mean error along y equal to %d cm ' , swarm_dimension(exp), mean(mean_y)    );
        fprintf('Exp with %d ROBOTS has mean error along x equal to %d cm ' , swarm_dimension(exp), mean(mean_x)    );
        fprintf('Exp with %d ROBOTS has mean distance error equal to %d cm' , swarm_dimension(exp), mean(mean_dist) );

        fprintf('Exp with %d ROBOTS Mean Squared Error along x: %d\n'        , swarm_dimension(exp), round(    ms_error_x(exp)     ,3) );
        fprintf('Exp with %d ROBOTS Mean Squared Error along y: %d\n'        , swarm_dimension(exp), round(    ms_error_y(exp)     ,3) );
        fprintf('Exp with %d ROBOTS Mean Squared Error distance error: %d\n' , swarm_dimension(exp), round(    ms_error_dist(exp)  ,3) );

    end

    % Vector of different colors
    line_color = ['#D95319','#0072BD',"#77AC30","#EDB120","#4DBEEE","#7E2F8E"];

    figure (1)
    subplot(3,1,1)
    grid on
    leg = {};
    for i = 1:length(swarm_dimension)
        plot(1:nExp,mean_x,'bo','Color',line_color(i)) 
        yline(mean_x_error(i),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Num Robots: ', num2str(swarm_dimension(i))]; 
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
    for i = 1:length(swarm_dimension) 
        plot(1:nExp,mean_y,'bo','Color',line_color(i)) 
        yline(mean_y_error(i),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Num Robots: ', num2str(swarm_dimension(i))]; 
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
    for i = 1:length(swarm_dimension)
        plot(1:nExp,mean_y,'bo','Color',line_color(i)) 
        yline(mean_dist_error(i),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = ['Num Robots: ', num2str(swarm_dimension(i))]; 
        hold on   
    end      
    title('Mean Error for total distance');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    hold off

    sgtitle('Estimation Errors along numerous simulations for different robot swarm dimensions'); 
    
    
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
