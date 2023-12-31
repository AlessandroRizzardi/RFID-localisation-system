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

KL = 0.01*10^-3; % noise parameters of odometry
KR = 0.01*10^-3;
sigma_phi = 0.2; % [rad] standard deviation of the angle measurement
msg = 50;        %number of consensus protocol messages exchanged by the nodes in the network
CONSENSUS = false;
ANIMATION = false;
PLOTS = true;
%%%%%%%%%%  END SETTINGS    %%%%%%%%%%%%%

nExp = 500; % Simulations to do for every test

TEST = 1;  % 1: Monte-Carlo simulation with single robot

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
    hold on   
    plot(exp,mean_x,'bo','MarkerSize', 4)  
    yline(mean(mean_x),'r--','LineWidth', 2)         
    title('Error for X coordinate');                            
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend('Error','Mean Error','Location','best');
    pbaspect([4 1 1])
    hold off
    
    subplot(3,1,2)
    grid on
    hold on
    plot(exp,mean_y,'bo','MarkerSize', 4);
    yline(mean(mean_y), 'r--', 'LineWidth', 2)
    title('Error for Y coordinate');
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend('Error','Mean Error','Location','best');
    pbaspect([4 1 1])
    hold off

    subplot(3,1,3)
    grid on
    hold on
    plot(exp,mean_dist,'bo','MarkerSize', 4);
    yline(mean(mean_dist), 'r--', 'LineWidth', 2)
    title('Distance Error');
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend('Error','Mean Error','Location','best');
    pbaspect([4 1 1])

    sgtitle(sprintf('Estimation Errors along %d experiments', nExp));  

    hold off

    fprintf('Mean error along x: %d\n', round(mean(mean_x),3));
    fprintf('Mean error along y: %d\n', round(mean(mean_y),3));
    fprintf('Mean error distance error: %d\n', round(mean(mean_dist),3));

    root_ms_error_x = sqrt(sum((mean_x - mean(mean_x)).^2)/length(mean_x));
    root_ms_error_y = sqrt(sum((mean_y - mean(mean_y)).^2)/length(mean_y));
    root_ms_error_dist = sqrt(sum((mean_dist - mean(mean_dist)).^2)/length(mean_dist));

    fprintf('Root Mean Squared Error along x: %d         \n', round( root_ms_error_x,    3));
    fprintf('Root Mean Squared Error along y: %d         \n', round( root_ms_error_y,    3));
    fprintf('Root Mean Squared Error distance error: %d  \n', round( root_ms_error_dist, 3));
    

    figure(2)
    subplot(3,1,1)
    plot(error_x, 'LineWidth', 2)
    xlabel('Iterations');
    ylabel('Error [cm]');
    pbaspect([4 1 1]);
    title('Error for X coordinate');   

    subplot(3,1,2)
    plot(error_y, 'LineWidth', 2)
    title('Error for Y coordinate');
    xlabel('Iterations');
    pbaspect([4 1 1]);
    ylabel('Error [cm]');

    subplot(3,1,3)
    plot(error_dist, 'LineWidth', 2)
    xlabel('Iterations');
    ylabel('Error [cm]');
    pbaspect([4 1 1]);
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
    
    K_list = [0.01*10^-2, 0.01*10^-3, 0.01*10^-4];
   

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
        
        % 1/N sum (x_i - mu)^2
        %Mean squared error
        root_ms_error_x(exp) = sqrt(sum((mean_x(exp,:) - mean(mean_x(exp,:))).^2)/length(mean_x(exp,:)));
        root_ms_error_y(exp) = sqrt(sum((mean_y(exp,:) - mean(mean_y(exp,:))).^2)/length(mean_y(exp,:)));
        root_ms_error_dist(exp) = sqrt(sum((mean_dist(exp,:) - mean(mean_dist(exp,:))).^2)/length(mean_dist(exp,:)));
        
        fprintf('Exp with K = %d has mean error along x equal to %d cm  \n' , K_list(exp), round(   mean(mean_x(exp,:))    ,3) );
        fprintf('Exp with K = %d has mean error along y equal to %d cm  \n' , K_list(exp), round(   mean(mean_y(exp,:))    ,3) );
        fprintf('Exp with K = %d has mean distance error equal to %d cm \n' , K_list(exp), round(   mean(mean_dist(exp,:)) ,3) );

        fprintf('Root Mean Squared Error along x: %d         \n',    round(    root_ms_error_x(exp)     ,3) );
        fprintf('Root Mean Squared Error along y: %d         \n',    round(    root_ms_error_y(exp)     ,3) );
        fprintf('Root Mean Squared Error distance error: %d  \n',    round(    root_ms_error_dist(exp)  ,3) );

    end
    %%
    % Vector of different colors
    line_color = ['#D95319','#0072BD',"#77AC30","#EDB120","#4DBEEE","#7E2F8E"];

    figure (1)
    subplot(3,1,1)
    grid on
    hold on
    leg = {};
    for i = 1:length(K_list)
        plot(1:nExp,mean_x(i,:),'bo', 'Color', line_color(i),'MarkerSize',3)  
        yline(mean(mean_x(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error for X coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off
    
    subplot(3,1,2)
    grid on
    hold on
    leg = {};
    for i = 1:length(K_list)
        plot(1:nExp,mean_y(i,:),'bo','Color',line_color(i),'MarkerSize',3)  
        yline(mean(mean_y(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error for Y coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off

    subplot(3,1,3)
    grid on
    hold on
    leg = {};
    for i = 1:length(K_list)
        plot(1:nExp,mean_dist(i,:),'bo','Color',line_color(i),'MarkerSize',3)  
        yline(mean(mean_dist(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['Odometry Error: ', num2str(K_list(i))]; 
        hold on   
    end      
    title('Mean Error of distance estimates');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off

   sgtitle('Estimation Errors along numerous simulations for different encoder readings noises');  

end 

% Test with single robot over different phase error
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
    phase_error_list = [0.2, pi/4 , pi/2, 3*pi/4, pi , 2*pi];
    
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
        root_ms_error_x(exp) = sqrt(sum((mean_x(exp,:) - mean(mean_x(exp,:))).^2)/length(mean_x(exp,:)));
        root_ms_error_y(exp) = sqrt(sum((mean_y(exp,:) - mean(mean_y(exp,:))).^2)/length(mean_y(exp,:)));
        root_ms_error_dist(exp) = sqrt(sum((mean_dist(exp,:) - mean(mean_dist(exp,:))).^2)/length(mean_dist(exp,:)));


        fprintf('Exp with sigma_phi = %d has mean error along x equal to %d cm \n ', phase_error_list(exp), mean(mean_x)       );
        fprintf('Exp with sigma_phi = %d has mean error along y equal to %d cm \n', phase_error_list(exp), mean(mean_y)       );
        fprintf('Exp with sigma_phi = %d has mean distance error equal to %d cm\n', phase_error_list(exp), mean(mean_dist)    );

        fprintf('Root Mean Squared Error along x: %d\n'        , round(    root_ms_error_x(exp)     ,3) );
        fprintf('Root Mean Squared Error along y: %d\n'        , round(    root_ms_error_y(exp)     ,3) );
        fprintf('Root Mean Squared Error distance error: %d\n' , round(    root_ms_error_dist(exp)  ,3) );
        
    end
%%
    % Vector of different colors
    line_color = ['#D95319','#0072BD',"#77AC30","#EDB120","#4DBEEE","#7E2F8E"];

    figure (1)
    subplot(3,1,1)
    grid on
    hold on
    leg = {};
    for i = 1:length(phase_error_list)
        plot(1:nExp,mean_x(i,:),'bo', 'MarkerSize',3,'Color',line_color(i)) 
        yline(mean(mean_x(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['\sigma_{\phi}: ', num2str(phase_error_list(i))]; 
        hold on   
    end      
    title('Mean Error for X coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off
    
    subplot(3,1,2)
    grid on
    hold on
    leg = {};
    for i = 1:length(phase_error_list)
        plot(1:nExp,mean_y(i,:),'bo','MarkerSize',3,'Color',line_color(i)) 
        yline(mean(mean_y(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['\sigma_{\phi}: ', num2str(phase_error_list(i))]; 
        hold on   
    end      
    title('Mean Error for Y coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off

    subplot(3,1,3)
    grid on
    hold on
    leg = {};
    for i = 1:length(phase_error_list)
        plot(1:nExp,mean_dist(i,:),'bo','MarkerSize',3,'Color',line_color(i)) 
        yline(mean(mean_dist(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['\sigma_{\phi}:', num2str(phase_error_list(i))]; 
        hold on   
    end      
    title('Mean Error of distance estimates');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off
    
    sgtitle('Estimation Errors along numerous simulations for different phase measurement errors');  

    mean_dist_error_list = [mean(mean_dist(1,:)),mean(mean_dist(2,:)), mean(mean_dist(3,:)),...
                            mean(mean_dist(4,:)), mean(mean_dist(5,:)), mean(mean_dist(6,:))];     

    figure(2)
    grid on
    hold on
    plot(phase_error_list, mean_dist_error_list, 'LineWidth', 2)
    title('Mean distance error for different phase measurement errors');
    xlabel('Phase measurement error [rad]');
    ylabel('Error [cm]');
    hold off

end


% Test with different swarm dimension
if TEST == 4
    fprintf('!!!! TEST CASE: Performing different simulations with different number of robots !!!!\n')
    
    swarm_dimension = [5, 10, 30];
    error_x = cell(length(swarm_dimension),1);
    error_y = cell(length(swarm_dimension),1);
    error_dist = cell(length(swarm_dimension),1);

    for exp = 1:length(swarm_dimension)
        nRobots = swarm_dimension(exp);
        CONSENSUS = true;
        
        fprintf(' ------ Processing Case: %d -----\n', exp)

        for trial=1:nExp
            
            initialization
            RFID_identification_algorithm
    
            clean_estimation_history = remove_NaN_values(robots(1).tag_estimation_history); % remove NaN values
    
            for i = 1:length(clean_estimation_history)
                error_x{exp,i} = (clean_estimation_history(i,1)-tag_position(1))*10^2;
                error_y{exp,i} = (clean_estimation_history(i,2)-tag_position(2))*10^2;
                error_dist{exp,i} = sqrt(error_x{exp,i}^2+error_y{exp,i}^2);
    
            end
    
            mean_x(exp,trial) = mean(cell2mat(error_x(exp,end-1500:end)));
            mean_y(exp,trial) = mean(cell2mat(error_y(exp,end-1500:end)));
            mean_dist(exp,trial) = mean(cell2mat(error_dist(exp,end-1500:end)));
    
            
    
            % fprintf('--------- Trial: %d ---------\n',trial);
        end

        %Mean squared error
        root_ms_error_x(exp) = sqrt(sum((mean_x(exp,:) - mean(mean_x(exp,:))).^2)/length(mean_x(exp,:)));
        root_ms_error_y(exp) = sqrt(sum((mean_y(exp,:) - mean(mean_y(exp,:))).^2)/length(mean_y(exp,:)));
        root_ms_error_dist(exp) = sqrt(sum((mean_dist(exp,:) - mean(mean_dist(exp,:))).^2)/length(mean_dist(exp,:)));

        fprintf('Exp with %d ROBOTS has mean error along y equal to %d cm \n' , swarm_dimension(exp), mean(mean_y(exp,:))     );
        fprintf('Exp with %d ROBOTS has mean error along x equal to %d cm \n' , swarm_dimension(exp), mean(mean_x(exp,:))     );
        fprintf('Exp with %d ROBOTS has mean distance error equal to %d cm \n', swarm_dimension(exp), mean(mean_dist(exp,:)) );

        fprintf('Exp with %d ROBOTS, Root Mean Squared Error along x: %d\n'        , swarm_dimension(exp), round(    root_ms_error_x(exp)     ,3) );
        fprintf('Exp with %d ROBOTS, Root Mean Squared Error along y: %d\n'        , swarm_dimension(exp), round(    root_ms_error_y(exp)     ,3) );
        fprintf('Exp with %d ROBOTS, Root Mean Squared Error distance error: %d\n' , swarm_dimension(exp), round(    root_ms_error_dist(exp)  ,3) );

    end
%%
    % Vector of different colors
    line_color = ['#D95319','#0072BD',"#77AC30","#EDB120","#4DBEEE","#7E2F8E"];

    figure (1)
    subplot(3,1,1)
    grid on
    hold on
    leg = {};
    for i = 1:length(swarm_dimension)
        plot(1:nExp,mean_x(i,:),'bo','MarkerSize',3,'Color',line_color(i)) 
        yline(mean(mean_x(i,:)),'Color',line_color(i),'LineWidth',2)
        leg{end+1} = ''; 
        leg{end+1} = ['Num Robots: ', num2str(swarm_dimension(i))]; 
        hold on   
    end      
    title('Mean Error for X coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off
    
    subplot(3,1,2)
    grid on
    hold on
    leg = {};
    for i = 1:length(swarm_dimension) 
        plot(1:nExp,mean_y(i,:),'bo','MarkerSize',3,'Color',line_color(i)) 
        yline(mean(mean_y(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['Num Robots: ', num2str(swarm_dimension(i))]; 
        hold on   
    end      
    title('Mean Error for Y coordinate');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off

    subplot(3,1,3)
    grid on
    hold on
    leg = {};
    for i = 1:length(swarm_dimension)
        plot(1:nExp,mean_dist(i,:),'bo','MarkerSize',3,'Color',line_color(i)) 
        yline(mean(mean_dist(i,:)),'Color',line_color(i),'LineWidth',2) 
        leg{end+1} = '';
        leg{end+1} = ['Num Robots: ', num2str(swarm_dimension(i))]; 
        hold on   
    end      
    title('Mean Error for total distance');                      
    xlabel('Num Experiments');
    ylabel('Error [cm]');
    legend(leg);
    pbaspect([4 1 1])
    hold off

    sgtitle('Estimation Errors along numerous simulations for different robot swarm dimensions'); 
    
    leg = {};
    figure(2)
    subplot(3,1,1)
    grid on
    hold on
    for exp = 1:length(swarm_dimension)
        plot(cell2mat(error_x(exp,:)), 'LineWidth', 2,'Color',line_color(exp))
        leg{end+1} = ['Num robots: ',num2str(swarm_dimension(exp))];
    end
    xlabel('Iterations');
    ylabel('Error [cm]');
    title('Error for X coordinate');
    ylim([-100 100])
    legend(leg)
    pbaspect([4 1 1])
    hold off
    
    leg = {};
    subplot(3,1,2)
    grid on
    hold on
    for exp = 1:length(swarm_dimension)
        plot(cell2mat(error_y(exp,:)), 'LineWidth', 2,'Color',line_color(exp))
        leg{end+1} = ['Num robots: ',num2str(swarm_dimension(exp))];
    end
    title('Error for Y coordinate');
    xlabel('Iterations');
    ylabel('Error [cm]');
    ylim([-100 100])
    legend(leg)
    pbaspect([4 1 1])
    hold off
    
    leg = {};
    subplot(3,1,3)
    grid on
    hold on
    for exp = 1:length(swarm_dimension)
        plot(cell2mat(error_dist(exp,:)), 'LineWidth', 2,'Color',line_color(exp))
        leg{end+1} = ['Num robots: ',num2str(swarm_dimension(exp))];
    end
    xlabel('Iterations');
    ylabel('Error [cm]');
    title('Distance Error');
    ylim([-100 100])
    legend(leg)
    pbaspect([4 1 1])
    hold off

    sgtitle('Tendency of Estimation Errors');
end
