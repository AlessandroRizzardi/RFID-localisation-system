close all;
clear;  
clc;

addpath('Scripts')
addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config

l = 1;
r = sqrt(l^2/2);


x1 = r*cos(pi/4);
y1 = r*sin(pi/4);

x2 = r*cos(3*pi/4);
y2 = r*sin(3*pi/4);

x3 = r*cos(5*pi/4);
y3 = r*sin(5*pi/4);

x4 = r*cos(7*pi/4);
y4 = r*sin(7*pi/4);

robot_1 = DifferentialDriveRobot([x1;y1;0],R,d,KR,KL,dt);
robot_2 = DifferentialDriveRobot([x2;y2;0],R,d,KR,KL,dt);
robot_3 = DifferentialDriveRobot([x3;y3;0],R,d,KR,KL,dt);
robot_4 = DifferentialDriveRobot([x4;y4;0],R,d,KR,KL,dt);

phantom_bot = DifferentialDriveRobot([0;0;0],R,d,KR,KL,dt);

robots = {robot_1,robot_2,robot_3,robot_4};

initial_swarm_position = [0,0];

swarm = Swarm(4,initial_swarm_position,l);

Tf = 3; % simulation time
steps = floor(Tf/robot_1.dt);

swarm_target = [6,6];
swarm_points_vector = [];
swarm_points_vector(1,1) = swarm_target(1);
swarm_points_vector(1,2) = swarm_target(2);
targets = swarm.compute_targets(swarm_target);

for i=1:steps

    distance_phantom = sqrt((phantom_bot.x(1) - swarm_target(1))^2 + (phantom_bot.x(2) - swarm_target(2))^2);

    if distance_phantom < 0.5
        swarm_target = rand(2,1)*16-8;
        targets = swarm.compute_targets(swarm_target);
        swarm_points_vector(end+1,:) = [swarm_target(1),swarm_target(2)];

        [v_phantom, omega_phantom] = greedy_controller(Kp_v1, Kp_w1, swarm_target(1),swarm_target(2),phantom_bot.x);

        [v_1, omega_1] = greedy_controller(Kp_v1, Kp_w1, targets{1,1}(1), targets{1,1}(2),robot_1.x_est);
        [v_2, omega_2] = greedy_controller(Kp_v1, Kp_w1, targets{2,1}(1), targets{2,1}(2),robot_2.x_est);
        [v_3, omega_3] = greedy_controller(Kp_v1, Kp_w1, targets{3,1}(1), targets{3,1}(2),robot_3.x_est);
        [v_4, omega_4] = greedy_controller(Kp_v1, Kp_w1, targets{4,1}(1), targets{4,1}(2),robot_4.x_est);
        
    else
        [v_phantom, omega_phantom] = greedy_controller(Kp_v1,Kp_w1, swarm_target(1),swarm_target(2),phantom_bot.x);

        [v_1, omega_1] = greedy_controller(Kp_v1, Kp_w1, targets{1,1}(1), targets{1,1}(2),robot_1.x_est);
        [v_2, omega_2] = greedy_controller(Kp_v1, Kp_w1, targets{2,1}(1), targets{2,1}(2),robot_2.x_est);
        [v_3, omega_3] = greedy_controller(Kp_v1, Kp_w1, targets{3,1}(1), targets{3,1}(2),robot_3.x_est);
        [v_4, omega_4] = greedy_controller(Kp_v1, Kp_w1, targets{4,1}(1), targets{4,1}(2),robot_4.x_est);
    end

    x_next_phantom = phantom_bot.dynamics(v_phantom,omega_phantom);
    phantom_history{i,1} = x_next_phantom;

    x_next_1 = robot_1.dynamics(v_1,omega_1);
    robot_1.odometry_step(v_1,omega_1);
    dynamics_history_1{i,1} = x_next_1;

    x_next_2 = robot_2.dynamics(v_2,omega_2);
    robot_3.odometry_step(v_2,omega_2);
    dynamics_history_2{i,1} = x_next_2;

    x_next_3 = robot_3.dynamics(v_3,omega_3);
    robot_3.odometry_step(v_3,omega_3);
    dynamics_history_3{i,1} = x_next_3;

    x_next_4 = robot_4.dynamics(v_4,omega_4);
    robot_4.odometry_step(v_4,omega_4);
    dynamics_history_4{i,1} = x_next_4;
end

x_dynamics_phantom = zeros(length(phantom_history),1);
y_dynamics_phantom = zeros(length(phantom_history),1);

for i = 1:length(phantom_history)
    x_dynamics_phantom(i) = phantom_history{i,1}(1);
    y_dynamics_phantom(i) = phantom_history{i,1}(2);
end

x_dynamics_1 = zeros(length(dynamics_history_1),1);
y_dynamics_1 = zeros(length(dynamics_history_1),1);

for i = 1:length(dynamics_history_1)
    x_dynamics_1(i) = dynamics_history_1{i,1}(1);
    y_dynamics_1(i) = dynamics_history_1{i,1}(2);
end

x_dynamics_2 = zeros(length(dynamics_history_2),1);
y_dynamics_2 = zeros(length(dynamics_history_2),1);

for i = 1:length(dynamics_history_2)
    x_dynamics_2(i) = dynamics_history_2{i,1}(1);
    y_dynamics_2(i) = dynamics_history_2{i,1}(2);
end

x_dynamics_3 = zeros(length(dynamics_history_3),1);
y_dynamics_3 = zeros(length(dynamics_history_3),1);

for i = 1:length(dynamics_history_3)
    x_dynamics_3(i) = dynamics_history_3{i,1}(1);
    y_dynamics_3(i) = dynamics_history_3{i,1}(2);
end

x_dynamics_4 = zeros(length(dynamics_history_4),1);
y_dynamics_4 = zeros(length(dynamics_history_4),1);

for i = 1:length(dynamics_history_4)
    x_dynamics_4(i) = dynamics_history_4{i,1}(1);
    y_dynamics_4(i) = dynamics_history_4{i,1}(2);
end


% Plottings
figure(1)
plot(x_dynamics_phantom, y_dynamics_phantom, 'b')
hold on
plot(x_dynamics_1,y_dynamics_1,'r')
hold on
plot(x_dynamics_2,y_dynamics_2,'y')
hold on
plot(x_dynamics_3,y_dynamics_3,'g')
hold on
plot(x_dynamics_4,y_dynamics_4,'k')
hold on
plot(swarm_points_vector(:,1),swarm_points_vector(:,2),'b*','MarkerSize',10)
hold on

plot(targets{1,1}(1),targets{1,1}(2), 'r*', 'MarkerSize', 10)
hold on
plot(targets{2,1}(1),targets{2,1}(2), 'y*', 'MarkerSize', 10)
hold on
plot(targets{3,1}(1),targets{3,1}(2), 'g*', 'MarkerSize', 10)
hold on
plot(targets{4,1}(1),targets{4,1}(2), 'k*', 'MarkerSize', 10)
xlabel('x [m]')
ylabel('y [m]')
xlim([-10 10])
ylim([-10 10])
