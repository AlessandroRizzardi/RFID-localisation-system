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

N = floor((Tf-1)/Ts);  %number of steps --> per la creazione delle histories altrimenti non saprei come fare

state_history = cell(N,3); % state of EKF
odometry_history = cell(N,3); 
phase_history = cell(N);
weights = zeros(N);

for k = 1:Ts:Tf

    if inTagRange == false && robot.inTagRange(tag_position, max_range) == true        % check if it is the first time the phase measurement is available

        inTagRange = true;
        phase_measured = robot.phaseMeasured(tag_position, lambda , true, sigma_phi);

        % Initialize nM EKF instances (l = 1,2,...,nM)
        for l = 1:nM
            EKF_instances{l} = EKF();

            EKF_instances{l} = EKF_instances{l}.EKF_init(phase_measured,l,lambda,sigma_phi);
        end
        rho_est = EKF_instances{nM}.x(1);
        beta_est = EKF_instances{nM}.x(2);

        [v,omega] = tag_pursuit_controller(Kp_v, Kp_w, rho_est,beta_est);

        x_next = robot.dynamics(v,omega,Ts);
        odometry_estimation = robot.odometry_step(v,omega,Ts);

        steps_in_range = steps_in_range + 1;
   
    elseif inTagRange == true && robot.inTagRange(tag_position, max_range) == true

        phase_measured = robot.phaseMeasured(tag_position, lambda , true, sigma_phi);

        % Prediction EKF
        for l = 1:nM
            EKF_instances{l} = EKF_instances{l}.EKF_prediction(odometry_estimation, L); % DA QUA
        end

        % Correction EKF
        for l = 1:nM
            EKF_instances{l} = EKF_instances{l}.EKF_correct(lambda, sigma_phi, phase_measured);    
        end

        % Correction of non-positive range estimation
        for l = 1:nM
            if EKF_instances{l}.x(1) <= 0
                EKF_instances{l}.x(1) = max(abs(EKF_instance{l}.x(1)),10^-6);
                EKF_instances{l}.x(2) = EKF_instances{l}.x(2) + pi;
            end
        end

        % Weighting step
        for l = 1:nm
            weight_tmp = EKF_instances{l}.EKF_weight_tmp() 

        end

    end

    [v,omega] = pure_pursuit_controller(Kp_v,Kp_w,target_point(1),target_point(2),robot.get_odometry_state());




end

