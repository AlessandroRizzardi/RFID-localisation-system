close all;
clear;  
clc;

addpath('Functions')
addpath('Classes')


% Load the configuration parameters for the algorithms
config

% Initialize the robot
robot = DifferentialDriveRobot([0;0;0],R,d,KR,KL,dt);

Tf = 50; % simulation time
steps = floor(Tf/robot.dt);

% Define position of the target randomly with x and y between -8 and 8
target_point = [4;4];

points_vector = [];
points_vector(1,1) = target_point(1);
points_vector(1,2) = target_point(2);


for i=1:steps

    distance = sqrt((robot.x_est(1) - target_point(1))^2 + (robot.x_est(2) - target_point(2))^2);

    if distance < 0.5
        target_point(1) = x_range(1) + (x_range(2)-x_range(1))*rand();
        target_point(2) = y_range(1) + (y_range(2)-y_range(1))*rand();
        points_vector(end+1,:) = [target_point(1),target_point(2)];
        [v,dtheta] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2),robot.x_est);
    else
        [v,dtheta] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);
    end

    x_next = robot.dynamics(v,dtheta);
    dynamics_history{i,1} = x_next;

    odometry_estimation = robot.odometry_step(v,dtheta);
    odometry_history{i,1} = robot.x_est;

end


x_odometry = zeros(length(odometry_history),1);
y_odometry = zeros(length(odometry_history),1);
x_dynamics = zeros(length(dynamics_history),1);
y_dynamics = zeros(length(dynamics_history),1);

for i = 1:length(odometry_history)
    x_odometry(i) = odometry_history{i,1}(1);
    y_odometry(i) = odometry_history{i,1}(2);

    x_dynamics(i) = dynamics_history{i,1}(1);
    y_dynamics(i) = dynamics_history{i,1}(2);
end

% Plottings
figure(1)
plot(x_odometry,y_odometry,'b')
hold on
plot(x_dynamics,y_dynamics,'r')
hold on
plot(points_vector(:,1),points_vector(:,2),'g*','MarkerSize',10)
hold on
legend('Odometry','Dynamics','Target Position')
xlabel('x [m]')
ylabel('y [m]')
xlim([-10 10])
ylim([-10 10])