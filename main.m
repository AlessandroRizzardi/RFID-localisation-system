%% Initialization2  
close all;
clear;  
clc;

addpath('Data')
addpath('Scripts')
addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config

robot = RobotReader([0,0,0]);

T_sim = 100;

% given a point to reach in the map the robot will move towards it, when it reaches it (check every iteration), another point is
% generated randomly and the robot moves towards it again
% the robot will move in a straight line towards the point, with a costant linear velocity

% generate random first point in the map that needs to be in the map 15x15
point = [randi([0,30]),randi([0,30])];
vector_points = [point];

% set the velocity of the robot
v = 1;
omega = robot.compute_angular_velocity(point, Ts);

robot = RobotReader([0,0,0]);

pos_robot_x(1) = robot.x(1);
pos_robot_y(1) = robot.x(2);
pos_robot{1,1} = [robot.x(1),robot.x(2),robot.x(3)];

point_reached = false;

k = 2;
for i = 2:Ts:T_sim

    next_state = robot.update_state_robot([v,omega], Ts);
    pos_robot{k,1} = next_state;
    pos_robot_x(k) = next_state(1);
    pos_robot_y(k) = next_state(2);
    omega = 0;

    point_reached = robot.reached_point(point);
    if point_reached == true
        point = [randi([0,30]),randi([0,30])];
        omega = robot.compute_angular_velocity(point, Ts);
        vector_points = [vector_points;point];
    end
    
    k = k +1;
end 

% plot the trajectory of the robot and the points
figure(1),clf, hold on;
plot(vector_points(:,1),vector_points(:,2),'*b','MarkerSize',10)
plot(pos_robot_x,pos_robot_y,'-r')
legend('Points','Robot','Location','best')
title('Robot trajectory')
xlim([0,30])
ylim([0,30])


%{

% Load the data
load_data

gt = readmatrix(select_GT);

% Run EKF
EKF

%% Ground truth comparison
if GT

    figure(1),clf, hold on;
    plot(gt(:,2),gt(:,3),'-k');
    
    for i = 1:length(pos_robot)
        
        posx(i) = pos_robot{i,1}(1);
        posy(i) = pos_robot{i,1}(2);
        th(i) = pos_robot{i,1}(3);

        covx(i) = cov_robot{i,1}(1,1);
        covy(i) = cov_robot{i,1}(2,2);  
        covt(i) = cov_robot{i,1}(3,3);  
    end
    
    % Plot the tags
    for i = 1:length(tags)
        tag = tags(i);
        plot(tag.x(1),tag.x(2),'*b','MarkerSize',10)
    end
    
    plot(posx,posy,'-r')
    %legend('Ground truth','Tags','Estimated','Location','best')
    title('GT Vs. EST')
    xlabel ('x [m]');
    ylabel ('y [m]');
    
    
    figure(2),clf
    errx = gt(:,2)' - posx;
    plot(1:1:length(gt),errx)
    hold on
    erry = gt(:,3)' - posy;
    plot(1:1:length(gt),erry)
    legend('Est. error x','Est. error y','Location','best')
    title('Estimation errors x and y')
    xlabel ('Iteration');
    ylabel ('[m]');

    %
    figure(3), clf;
    plot(1:1:length(gt),errx)
    hold on
    plot(1:1:length(gt),2*covx,'r');
    hold on
    plot(1:1:length(gt),-2*covx,'r');
    hold on
    plot(1:1:length(gt),3*covx,'g');
    hold on
    plot(1:1:length(gt),-3*covx,'g');

    legend('Est. error x','Est. covariance x (97%)','','Est. covariance x (99%)','Location','best')
    title('Estimation errors x')
    xlabel ('Iteration');
    ylabel ('[m]');
    %
    figure(7), clf;
    plot(1:1:length(gt),erry)
    hold on
    plot(1:1:length(gt),2*covy,'r');
    hold on
    plot(1:1:length(gt),-2*covy,'r');
    hold on
    plot(1:1:length(gt),3*covy,'g');
    hold on
    plot(1:1:length(gt),-3*covy,'g');

    legend('Est. error y','Est. covariance y (97%)','','Est. covariance y (99%)','Location','best')
    title('Estimation errors y')
    xlabel ('Iteration');
    ylabel ('[m]');

    %
    figure(12), clf;

    errt = gt(:,4)' - th;
    plot(1:1:length(gt),errt);
    hold on
    plot(1:1:length(gt),2*covt,'r');
    hold on
    plot(1:1:length(gt),-2*covt,'r');
    hold on
    plot(1:1:length(gt),3*covt,'g');
    hold on
    plot(1:1:length(gt),-3*covt,'g');

    legend('Est. error theta','Est. covariance theta (97%)','','Est. covariance theta (99%)','Location','best')
    title('Estimation errors theta')
    xlabel ('Iteration');
    ylabel ('[rad]');
    
end

%}