clc
clear
close all

% main script to test the functionality of our MHEKF algorithm that tries
% to localize a tag. The robot moves randomly in a map and is always in the
% range of the RFID tag

addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config

% Initialize the robot
robot = DifferentialDriveRobot([0;0;0],R,d,KR,KL,dt);

% Define position of the tag
tag_position = [1.6;1.6];

% 1st virtual target point
% random number between -5 and 5
target_point(1) = x_range(1) + (x_range(2)-x_range(1))*rand();
target_point(2) = y_range(1) + (y_range(2)-y_range(1))*rand();
points_vector = [];
points_vector(1,1) = target_point(1);
points_vector(1,2) = target_point(2);

% inTagRange = false; % set the status of the robot if it is in the range of the RFID tag or not

for i = 1:nM
    EKF_instances(i) = EKF();
end

steps_in_range = 0;

weights_vec = zeros(nM,1);
weight_init = 1/nM;

weights_tmp = zeros(nM,1);
weights = zeros(nM,1);
weights_prev = 10^-6*ones(nM,1);

instance_selected = 0; 

go_in = true;  % flag to check if the robot is going towards the tag or not

steps = Tf/robot.dt;

% EKF initialization
% first phase measured
phase_measured = robot.phaseMeasured(tag_position, lambda , sigma_phi);
phase_history(1,1) = phase_measured;

% Initialize nM EKF instances (l = 1,2,...,nM)
for l = 1:nM
    EKF_instances(l).EKF_init(phase_measured,l,lambda,sigma_phi,weight_init);

    % Save the state history of each EKF instance
    EKF_instances(l).state_history{1,1} = EKF_instances(l).x;

end
rho_est = EKF_instances(nM).x(1);
beta_est = EKF_instances(nM).x(2);
best_state_estimate{1,1} = [rho_est;beta_est];


for k = 1:steps
    
    % move the robot
    distance = sqrt((robot.x_est(1) - target_point(1))^2 + (robot.x_est(2) - target_point(2))^2);
    if distance < 0.3
        target_point(1) = x_range(1) + (x_range(2)-x_range(1))*rand();
        target_point(2) = y_range(1) + (y_range(2)-y_range(1))*rand();
        points_vector(end+1,:) = [target_point(1),target_point(2)];
        [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2),robot.x_est);
    else
        [v,omega] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);
    end

    x_next = robot.dynamics(v,omega);
    dynamics_history{k,1} = x_next;
    odometry_estimation = robot.odometry_step(v,omega);
    odometry_history{k,1} = robot.x_est;


    % RFID measurements
    phase_measured = robot.phaseMeasured(tag_position, lambda , sigma_phi);
    phase_history(k,1) = phase_measured;
    
   
    % Prediction and Correction EKF
    for l = 1:nM
        EKF_instances(l).EKF_predict(odometry_estimation, d);
        EKF_instances(l).EKF_correct(K, sigma_phi, phase_measured);

        weights_vec(l) = EKF_instances(l).weight;

        % Save the state history of each EKF instance
        EKF_instances(l).state_history{k+1,1} = EKF_instances(l).x;

    end

    % Correction of non-positive range estimation
    for l = 1:nM
        if EKF_instances(l).x(1) <= 10^-6
            EKF_instances(l).x(1) = max(abs(EKF_instances(l).x(1)),10^-6);
            EKF_instances(l).x(2) = EKF_instances(l).x(2) + pi;
        end
        EKF_instances(l).x(2) = atan2(sin(EKF_instances(l).x(2)),cos(EKF_instances(l).x(2)));
    end
        
    if steps_in_range >= Ns

        %for l = 1:nM
        %    %display(k)
        %    weights_tmp(l) = EKF_instances(l).EKF_weight_tmp(k, odometry_history, phase_history, Ns, weights_prev(l), c1, c2, K,l);
        %end
        %
        %eta = compute_eta(weights_tmp);
        %
        %for l = 1:nM
        %    weights(l) = EKF_instances(l).EKF_weight(weights_tmp(l), eta);
        %end
        %
        %weights_prev = weights;


        % Take as final estimates ρ^k and β^k the ones provided by the EKF instance with the largest weight
        %[max_value,instance_selected] = max(weights);
        
        [max_value,instance_selected] = max(weights_vec);


        rho_est = EKF_instances(instance_selected).x(1);
        beta_est = EKF_instances(instance_selected).x(2);

        best_state_estimate{k+1,1} = [rho_est;beta_est];

    else % we don't have enough measurements to weigh the EKF instances --> final estimates are the ones of the last EKF instance

        rho_est  = EKF_instances(nM).x(1);
        beta_est = EKF_instances(nM).x(2);

        best_state_estimate{k+1,1} = [rho_est;beta_est];

    end        

    steps_in_range = steps_in_range + 1;
    
    
    display('---------------------ITERATION K:-------------------.');
    display(k);
    
end

best_tag_estimation_x = odometry_history{end,1}(1) + best_state_estimate{end,1}(1) * cos(odometry_history{end,1}(3) - best_state_estimate{end,1}(2));
best_tag_estimation_y = odometry_history{end,1}(2) + best_state_estimate{end,1}(1) * sin(odometry_history{end,1}(3) - best_state_estimate{end,1}(2));

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

angle = 0:0.1:2*pi;
rad = max_range; %[m]
x_coord = tag_position(1) + rad*cos(angle);
y_coord = tag_position(2) + rad*sin(angle);

% Plottings
figure(1)
plot(x_odometry,y_odometry,'b')
hold on
plot(x_dynamics,y_dynamics,'r')
hold on
plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
hold on
plot(best_tag_estimation_x, best_tag_estimation_y,'b*','MarkerSize',10)
plot(x_coord,y_coord);
hold on
legend('Odometry','Dynamics','Tag Position','Best estimation')
xlabel('x [m]')
ylabel('y [m]')
xlim(x_range)
ylim(y_range)
