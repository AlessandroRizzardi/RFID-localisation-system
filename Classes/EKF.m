classdef EKF < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 

    x; % filter of the state (ro, beta,x_robot, y_robot, theta_robot)
    P; % covariance matrix of the state
    weight; % weight of the EKF instance  
    weight_history; % history of the weights of the EKF instance   
    state_history; % history of the states of the EKF instance 
    innovation_history; % history of the innovations of the EKF instance        
end % properties

%  ____        _     _ _        __  __                _                                                             
% |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___                                           
% | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|                                          
% |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \                                          
% |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/                                          
%                                                                                                               
methods 

    % Constructor 
    function obj = EKF() % constructor
        obj.x = zeros(5,1);
        obj.P = zeros(5,5);
        obj.weight = 0;
        obj.weight_history = [];
        obj.state_history = []; 
        obj.innovation_history = [];
    end

    % Constructor that corresponds to initialization step of the filter
    % z0 is the measurement at time 0
    function EKF_init(obj,phi_0, l, lambda, sigma_phi,weight_init, robot_pose, robot_cov_matrix) % constructor
              
        K = (2*pi)/lambda;
        ro_0 = -phi_0/(2*K) + (l*lambda)/2;
        beta_0 = 0;
        x_0 = robot_pose(1);
        y_0 = robot_pose(2);
        theta_0 = robot_pose(3);

        obj.x = [ro_0; beta_0; x_0; y_0; theta_0];

        sigma_d = sigma_phi/(2*K);
        obj.P = [sigma_d^2, 0       , 0                    , 0                    , 0                    ;...
                    0     , (pi/3)^2, 0                    , 0                    , 0                    ;...
                    0     , 0       , robot_cov_matrix(1,1), robot_cov_matrix(1,2), robot_cov_matrix(1,3);...
                    0     , 0       , robot_cov_matrix(2,1), robot_cov_matrix(2,2), robot_cov_matrix(2,3);...
                    0     , 0       , robot_cov_matrix(3,1), robot_cov_matrix(3,2), robot_cov_matrix(3,3)];

        obj.weight = weight_init;
        obj.weight_history = [obj.weight_history; obj.weight];

    end
 
    % function that computes the predicti9on step of the filter
    function EKF_predict(obj, encoder_readings, d) % constructor
        % initialize cell array encoder readings

        u = encoder_readings{1,1}(1);
        omega = encoder_readings{1,1}(2);
        Q = encoder_readings{1,2};

        ro_curr = obj.x(1);
        beta_curr = obj.x(2);
        x_curr = obj.x(3);
        y_curr = obj.x(4);
        theta_curr = obj.x(5);
        P_curr = obj.P;

        ro_next = ro_curr - u*cos(beta_curr);
        beta_next = beta_curr + omega + (u/ro_curr)*sin(beta_curr);
        x_next = x_curr + u*cos(theta_curr);
        y_next = y_curr + u*sin(theta_curr);
        theta_next = theta_curr + omega;



        % F, W are respectively Jacobian matrices of the state dynamics with respect to the state and the encoder noise
        F = [1                            , u*sin(beta_curr)              , 0 , 0 , 0                 ;...
             -(u/ro_curr^2)*sin(beta_curr), 1 + (u/ro_curr)*cos(beta_curr), 0 , 0 , 0                 ;...
             0                            , 0                             , 1 , 0 , -u*sin(theta_curr);...
             0                            , 0                             , 0 , 1 , u*cos(theta_curr) ;...
             0                            , 0                             , 0 , 0 , 1];

        W = [ -0.5*cos(beta_curr)                 , -0.5*cos(beta_curr)                  ;...
              1/d + (1/(2*ro_curr))*sin(beta_curr), -1/d + (1/(2*ro_curr))*sin(beta_curr);...
              0.5*cos(theta_curr)                 , 0.5*cos(theta_curr)                  ;...
              0.5*sin(theta_curr)                 , 0.5*sin(theta_curr)                  ;...
              1/d                                 , -1/d];
        
        P_next = F*P_curr*F' + W*Q*W';
        
        
        obj.x = [ro_next; beta_next; x_next; y_next; theta_next];
        obj.P = P_next;

    end

    % function that computes the correction step of the filter
    function EKF_correct(obj, K, sigma_phi, phi_meas) % constructor
        
        ro_curr = obj.x(1);
        beta_curr = obj.x(2);
        x_curr = obj.x(3);
        y_curr = obj.x(4);
        theta_curr = obj.x(5);

        P_curr = obj.P;

        R = sigma_phi^2;

        H = [-2*K, 0, 0, 0, 0];
        
        Kalman_gain = P_curr*H'*pinv(H*P_curr*H' + R);

        phi_expected = mod(-2*K*ro_curr,2*pi);

        innovation = phi_meas - phi_expected;
        innovation = atan2(sin(innovation),cos(innovation));
        obj.innovation_history = [obj.innovation_history; innovation];

        state_next = [ro_curr; beta_curr; x_curr; y_curr; theta_curr] + Kalman_gain*(innovation);

        P_next = (eye(5) - Kalman_gain*H)*P_curr;

        ro_next = state_next(1);
        beta_next = state_next(2);
        x_next = state_next(3);
        y_next = state_next(4);
        theta_next = state_next(5);

        % update the weight of the EKF instance
        obj.weight = obj.weight*exp(-0.5*innovation^2/(H*P_next*H' + R));
        obj.weight_history = [obj.weight_history; obj.weight];

        obj.x = [ro_next; beta_next; x_next; y_next; theta_next];
        obj.P = P_next;
    end

    % each EKF instance is weighed The weight is computed taking into account two kinds of metric computed 
    % considering the phase measurements collected over a time window comprising the last Ns = 50 steps
    % 1) the movement of the tag position estimate in the last Ns steps (metric M1 );
    % 2) the agreement of the tag position estimate with the last Ns measurements (metric M2 ).
    % function weight_tmp = EKF_weight_tmp(obj, k, odometry_history, phase_history, Ns, weight_prec, c1, c2, K, l)
    % 
    %     sum_phase_diff = 0;
    % 
    %     for i = (k-Ns+1) : (k-1)
    % 
    %         x_tag =  odometry_history{i,1}(1) + obj.state_history{i,1}(1)*cos(odometry_history{i,1}(3) - obj.state_history{i,1}(2));
    %         y_tag =  odometry_history{i,1}(2) + obj.state_history{i,1}(1)*sin(odometry_history{i,1}(3) - obj.state_history{i,1}(2));
    % 
    % 
    %         if i == (k-Ns+1)
    %             x_min_tag = x_tag;
    %             y_min_tag = y_tag;
    %             x_max_tag = x_tag;
    %             y_max_tag = y_tag;
    %         end
    % 
    %         if x_tag < x_min_tag
    %             x_min_tag = x_tag;
    %         end
    %         if y_tag < y_min_tag
    %             y_min_tag = y_tag;
    %         end
    %         if x_tag > x_max_tag
    %             x_max_tag = x_tag;
    %         end
    %         if y_tag > y_max_tag
    %             y_max_tag = y_tag;
    %         end
    % 
    %         D = sqrt((odometry_history{i,1}(1) - x_tag)^2 + (odometry_history{i,1}(2) - y_tag)^2);
    %         phi_expected = mod(-2*K*D,2*pi);
    % 
    %         phase_diff = (phase_history(i) - phi_expected)^2;
    % 
    %         sum_phase_diff = sum_phase_diff + phase_diff;
    % 
    %     end
    % 
    %     M1 = 1/(1 + sqrt((x_max_tag - x_min_tag)^2 + (y_max_tag - y_min_tag)^2));
    % 
    %     M2 = 1/sqrt(sum_phase_diff);
    % 
    %     % Print x_max_tag, x_min_tag, y_max_tag, y_min_tag, phi_expected
    %     % fprintf('EKF INSTANCE: %d \n',l);
    %     % fprintf('x_max_tag = %f, x_min_tag = %f, y_max_tag = %f, y_min_tag = %f \n',x_max_tag, x_min_tag, y_max_tag, y_min_tag);
    %     % fprintf('D: %f, -2*K*D: %f, Phi expected = %f\n',D, -2*K*D, phi_expected);
    % 
    %     weight_tmp = weight_prec + c1*M1 + c2*M2; 
    % 
    % end
    % 
    % 
    % function weight = EKF_weight(obj, weight_tmp, eta)
    %     weight = eta*weight_tmp;
    % end

%  ____       _            _         __  __                _                   
% |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
% |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
%
% Here are defined auxiliary functions used in the public members or for other simpler computations



end % methods
end % Laserscan class