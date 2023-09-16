%% Initialization2  
close all;
clear;  
clc;

addpath('Scripts')
addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config

% Initialize the robot
robot = DifferentialDriveRobot([0;0;0],R,d,KR,KL);

% Define position of the tag randomly with x and y between -8 and 8
tag_position = rand(2,1)*16-8;

target_point = rand(2,1)*16-8;

inTagRange = false;

for i = 1:nM
    EKF_instances(i) = EKF();
end

steps_in_range = 0;



weights_tmp = zeros(nM,1);
weights = zeros(nM,1);
weights_prev = 10^-6*ones(nM,1);

instance_selected = 0; %

go_in = true;  % flag to check if the robot is going towards the tag or not

j = 1;

for k = 1:Ts:Tf

    if inTagRange == false && robot.inTagRange(tag_position, max_range) == true        % check if it is the first time the phase measurement is available

        inTagRange = true;
        phase_measured = robot.phaseMeasured(tag_position, lambda , true, sigma_phi);
        phase_history(j,1) = phase_measured;

        % Initialize nM EKF instances (l = 1,2,...,nM)
        for l = 1:nM
            EKF_instances(l).EKF_init(phase_measured,l,lambda,sigma_phi);
        end

        rho_est = EKF_instances(nM).x(1);
        beta_est = EKF_instances(nM).x(2);

        state_history{j,1} = [rho_est;beta_est];

        if rho_est < 0.3 || go_in == false
            go_in = false;
            target_point = rand(2,1)*40-20;
            [v,omega] = greedy_controller(Kp_v,target_point(1),target_point(2),robot.x_est);
        else
            [v,omega] = tag_pursuit_controller(Kp_v, Kp_w, rho_est,beta_est);
        end

        x_next = robot.dynamics(v,omega);
        dynamics_history{j,1} = x_next;

        odometry_estimation = robot.odometry_step(v,omega);

        odometry_history{j,1} = robot.x_est;

        steps_in_range = steps_in_range + 1;
   
    elseif inTagRange == true && robot.inTagRange(tag_position, max_range) == true        % check if the robot is still in range of the tag

        phase_measured = robot.phaseMeasured(tag_position, lambda , true, sigma_phi);
        phase_history(j,1) = phase_measured;

        % Prediction and Correction EKF
        for l = 1:nM
            EKF_instances(l).EKF_predict(odometry_estimation, d);
            EKF_instances(l).EKF_correct(K, sigma_phi, phase_measured); 
        end

        % Correction of non-positive range estimation
        for l = 1:nM
            if EKF_instances(l).x(1) <= 0
                EKF_instances(l).x(1) = max(abs(EKF_instances(l).x(1)),10^-6);
                EKF_instances(l).x(2) = EKF_instances(l).x(2) + pi;
            end
        end

        
        if steps_in_range >= Ns
            % Weighing Step
            for l = 1:nM
                weights_tmp(l) = EKF_instances(l).EKF_weight_tmp(j, state_history, odometry_history, phase_history, Ns, weights_prev(l), c1, c2, K);
            end

            eta = compute_eta(weights_tmp);

            for l = 1:nM
                weights(l) = EKF_instances(l).EKF_weight(weights_tmp, eta);
            end

            weights_prev = weights;

            % Take as final estimates ρ^k and β^k the ones provided by the EKF instance with the largest weight
            [~,instance_selected] = max(weights);

            rho_est = EKF_instances(instance_selected).x(1);
            beta_est = EKF_instances(instance_selected).x(2);

            state_history{j,1} = [rho_est;beta_est];

            if rho_est < 0.3 || go_in == false
                go_in = false;
                target_point = rand(2,1)*16-8;
                [v,omega] = greedy_controller(Kp_v, target_point(1),target_point(2),robot.x_est);
            else
                [v,omega] = tag_pursuit_controller(Kp_v, Kp_w, rho_est,beta_est);
            end

            x_next = robot.dynamics(v,omega);
            dynamics_history{j,1} = x_next;

            odometry_estimation = robot.odometry_step(v,omega);

            odometry_history{j,1} = robot.x_est;

        else % we don't have enough measurements to weigh the EKF instances --> final estimates are the ones of the last EKF instance

            rho_est  = EKF_instances(nM).x(1);
            beta_est = EKF_instances(nM).x(2);

            state_history{j,1} = [rho_est;beta_est];

            if rho_est < 0.3 || go_in == false
                go_in = false;
                target_point = rand(2,1)*16-8;
                [v,omega] = greedy_controller(Kp_v, target_point(1),target_point(2),robot.x_est);
            else
                [v,omega] = tag_pursuit_controller(Kp_v, Kp_w, rho_est,beta_est);
            end

            x_next = robot.dynamics(v,omega);
            dynamics_history{j,1} = x_next;

            odometry_estimation = robot.odometry_step(v,omega);

            odometry_history{j,1} = robot.x_est;

        end        

        steps_in_range = steps_in_range + 1;

    elseif  robot.inTagRange(tag_position, max_range) == false       % check if the robot is out of range of the tag
        
        state_history{j,1} = [];
        phase_history(j,1) = 0;

        x_point = target_point(1);
        y_point = target_point(2);
        distance = sqrt((robot.x_est(1) - x_point)^2 + (robot.x_est(2) - y_point)^2);

        if distance < 0.2
            target_point = rand(2,1)*16-8;
            [v,omega] = greedy_controller(Kp_v, target_point(1),target_point(2),robot.x_est);
        else
            [v,omega] = greedy_controller(Kp_v, target_point(1),target_point(2),robot.x_est);
        end


        x_next = robot.dynamics(v,omega);
        dynamics_history{j,1} = x_next;

        odometry_estimation = robot.odometry_step(v,omega);
        
        odometry_history{j,1} = robot.x_est;

        inTagRange = false;
    end

    j = j + 1;

end

%best_tag_estimation_x = odometry_history{end,1}(1) + state_history{end,1}(1) * cos(odometry_history{end,1}(3) - state_history{end,1}(2));
%best_tag_estimation_y = odometry_history{end,1}(2) + state_history{end,1}(1) * sin(odometry_history{end,1}(3) - state_history{end,1}(2));

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
plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
hold on
plot(target_point(1),target_point(2),'g*','MarkerSize',10)
%plot(best_tag_estimation_x,best_tag_estimation_y,'g*','MarkerSize',10)
hold on
legend('Odometry','Dynamics','Tag Position','Target Point')
xlabel('x [m]')
ylabel('y [m]')
xlim([-10 10])
ylim([-10 10])



