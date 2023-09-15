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
robot = DifferentialDriveRobot([0;0;0],R,L,KR,KL);

% Define position of the tag randomly with x and y between -20 and 20
tag_position = rand(2,1)*40-20;

target_point = rand(2,1)*40-20;

inTagRange = false;

EKF_instances = cell{nM,1};

steps_in_range = 0;

state_history = {}; % state of EKF
dynamics_history = {}; 
odometry_history = {}; 
phase_history = [];

weights_tmp = zeros(nM,1);
weights = zeros(nM,1);
weights_prev = 10^-6*ones(nM,1);

instance_selected = 0; %

for k = 1:Ts:Tf

    if inTagRange == false && robot.inTagRange(tag_position, max_range) == true        % check if it is the first time the phase measurement is available

        inTagRange = true;
        phase_measured = robot.phaseMeasured(tag_position, lambda , true, sigma_phi);
        phase_history(end+1,1) = phase_measured;

        % Initialize nM EKF instances (l = 1,2,...,nM)
        for l = 1:nM
            EKF_instances{l} = EKF();

            EKF_instances{l} = EKF_instances{l}.EKF_init(phase_measured,l,lambda,sigma_phi);
        end
        rho_est = EKF_instances{nM}.x(1);
        beta_est = EKF_instances{nM}.x(2);

        state_history{end+1,1} = [rho_est;beta_est];

        [v,omega] = tag_pursuit_controller(Kp_v, Kp_w, rho_est,beta_est);

        x_next = robot.dynamics(v,omega,Ts);
        dynamics_history{end+1,1} = x_next;

        odometry_estimation = robot.odometry_step(v,omega,Ts);
        odometry_history{end+1,1} = odometry_estimation;

        steps_in_range = steps_in_range + 1;
   
    elseif inTagRange == true && robot.inTagRange(tag_position, max_range) == true        % check if the robot is still in range of the tag

        phase_measured = robot.phaseMeasured(tag_position, lambda , true, sigma_phi);

        % Prediction and Correction EKF
        for l = 1:nM
            EKF_instances{l} = EKF_instances{l}.EKF_predict(odometry_estimation, L);

            EKF_instances{l} = EKF_instances{l}.EKF_correct(lambda, sigma_phi, phase_measured); 
        end

        % Correction of non-positive range estimation
        for l = 1:nM
            if EKF_instances{l}.x(1) <= 0
                EKF_instances{l}.x(1) = max(abs(EKF_instance{l}.x(1)),10^-6);
                EKF_instances{l}.x(2) = EKF_instances{l}.x(2) + pi;
            end
        end

        if steps_in_range >= Ns
            % Weighing Step
            for l = 1:nM
                weights_tmp(l) = EKF_instances{l}.EKF_weight_tmp(k, state_history, odometry_history, phase_history, Ns, weights_prev(l), c1, c2);
            end

            eta = compute_eta(weights_tmp);

            for l = 1:nM
                weights(l) = EKF_instances{l}.EKF_weight(weights_tmp, eta);
            end

            weights_prev = weights;

            % Take as final estimates ρ^k and β^k the ones provided by the EKF instance with the largest weight
            [~,instance_selected] = max(weights);

            rho_est = EKF_instances{instance_selected}.x(1);
            beta_est = EKF_instances{instance_selected}.x(2);

            state_history{end+1,1} = [rho_est;beta_est];

            [v,omega] = tag_pursuit_controller(Kp_v, Kp_w, rho_est,beta_est);

            x_next = robot.dynamics(v,omega,Ts);
            dynamics_history{end+1,1} = x_next;

            odometry_estimation = robot.odometry_step(v,omega,Ts);
            odometry_history{end+1,1} = odometry_estimation;

            steps_in_range = steps_in_range + 1;
        end

        % ---------------------- ATTENZIONE ----------------------
        % else --> steps_in_range < Ns --> che facciamo???
        
        
    elseif inTagRange == true && robot.inTagRange(tag_position, max_range) == false        % check if the robot is out of range of the tag
        
        % Reset everything
        
        inTagRange =false;
        EKF_instances = cell{nM,1};

        steps_in_range = 0;

        state_history = {}; % state of EKF
        dynamics_history = {}; 
        odometry_history = {}; 
        phase_history = [];

        weights_tmp = zeros(nM,1);
        weights = zeros(nM,1);
        weights_prev = 10^-6*ones(nM,1);

        instance_selected = 0; %

        [v,omega] = pure_pursuit_controller(Kp_v,Kp_w,target_point(1),target_point(2),robot.get_odometry_state());

        x_next = robot.dynamics(v,omega,Ts);
        dynamics_history{end+1,1} = x_next;

        odometry_estimation = robot.odometry_step(v,omega,Ts);
        odometry_history{end+1,1} = odometry_estimation;

    end

    




end

