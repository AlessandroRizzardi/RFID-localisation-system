classdef EKF < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 

    x; % filter of the state (ro, beta)
    P; % covariance matrix of the state
                 
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
        obj.x = zeros(2,1);
        obj.P = zeros(2,2);
    end



    % Constructor that corresponds to initialization step of the filter
    % z0 is the measurement at time 0
    function EKF_instance = EKF_init(obj,phi_0, l, lambda, sigma_phi) % constructor
        
        K = (2*pi)/lambda;
        ro_0 = -phi_0/(2*K) + (l*lambda)/2;
        beta_0 = 0;

        obj.x = [ro_0,beta_0];

        sigma_d = sigma_phi/(2*K);
        obj.P = [sigma_d^2, 0; 0, (pi/3)^2];
        
        EKF_instance{1,1} = obj.x;
        EKF_instance{2,1} = obj.P;
    end
    
    % function that computes the prediction step of the filter
    function EKF_instance_est = EKF_predict(obj, encoder_readings, d) % constructor
        % initialize cell array encoder readings

        u = encoder_readings{1,1}(1);
        omega = encoder_readings{1,1}(2);
        Q = encoder_readings{2,1};

        ro_curr = obj.x(1);
        beta_curr = obj.x(2);
        P_curr = obj.P;

        ro_next = ro_curr - u*cos(beta_curr);
        beta_next = beta_curr + omega + (u/ro_curr)*sin(beta_curr);


        % F, W are respectively Jacobian matrices of the state dynamics with respect to the state and the encoder noise
        F = [                            1,                 u*sin(beta_curr);...
             -(u/ro_curr^2)*sin(beta_curr),   1 + (u/ro_curr)*cos(beta_curr)];

        W = [                  -0.5*cos(beta_curr),                     -0.5*cos(beta_curr);...
              1/d + (1/(2*ro_curr))*sin(beta_curr),    1/d + (1/(2*ro_curr))*sin(beta_curr)];
        
        P_next = F*P_curr*F' + W*Q*W';
        
        
        EKF_instance_est{1,1} = [ro_next;beta_next];
        EKF_instance_est{2,1} = P_curr;

        obj.x = [ro_next;beta_next];
        obj.P = P_next;
    end

    % function that computes the correction step of the filter
    function EKF_instance_est = EKF_correct(obj, lambda, sigma_phi, phi_meas) % constructor
        
        ro_curr = obj.x(1);
        beta_curr = obj.x(2);
        P_curr = obj.P;

        K = (2*pi)/lambda;

        R = sigma_phi^2;

        H = [-2*K, 0];
        
        K_gain = P_curr*H'/(H*P_curr*H' + R);

        phi_expected = mod(-2*K*ro_curr,2*pi);

        x_next = [ro_curr;beta_curr] + K_gain*(phi_meas - phi_expected);

        P_next = (eye(2) - K*H)*P_curr;

        ro_next = x_next(1);
        beta_next = x_next(2);

        EKF_instance_est{1,1} = [ro_next;beta_next];
        EKF_instance_est{2,1} = P_next;

        obj.x = [ro_next;beta_next];
        obj.P = P_next;
    end

    % each EKF instance is weighed The weight is computed taking into account two kinds of metric computed 
    % considering the phase measurements collected over a time window comprising the last Ns = 50 steps
    % 1) the movement of the tag position estimate in the last Ns steps (metric M1 );
    % 2) the agreement of the tag position estimate with the last Ns measurements (metric M2 ).
    function weight_tmp = EKF_weight_tmp(obj, k, state_history, odometry_history, phase_history, Ns, weight_prec, c1, c2)

        ro_hist = state_history{1,1};
        beta_hist = state_history{2,1};

        x_robot_hist = odometry_history{1,1};
        y_robot_hist = odometry_history{2,1};
        theta_robot_hist = odometry_history{3,1};

        sum_phase_diff = 0;

        for i = (k-Ns+1) : k
            x_tag(i) =  x_robot_hist(i) + ro_hist(i)*cos(theta_robot_hist(i) - beta_hist(i));
            y_tag(i) =  y_robot_hist(i) + ro_hist(i)*sin(theta_robot_hist(i) - beta_hist(i));

            if i == (k-Ns+1)
                x_min_tag = x_tag(i);
                y_min_tag = y_tag(i);
                x_max_tag = x_tag(i);
                y_max_tag = y_tag(i);
            end

            if x_tag(i) < x_min_tag
                x_min_tag = x_tag(i);
            end
            if y_tag(i) < y_min_tag
                y_min_tag = y_tag(i);
            end
            if x_tag(i) > x_max_tag
                x_max_tag = x_tag(i);
            end
            if y_tag(i) > y_max_tag
                y_max_tag = y_tag(i);
            end

            D = sqrt((x_robot_hist(i) - x_tag(i))^2 + (y_robot_hist(i) - y_tag(i))^2);
            phi_expected = mod(-2*K*D,2*pi);

            phase_diff = (phase_history(i) - phi_expected)^2;

            sum_phase_diff = sum_phase_diff + phase_diff;

            
        end

        M1 = 1/(1 + sqrt((x_max_tag - x_min_tag)^2 + (y_max_tag - y_min_tag)^2));

        M2 = 1/sqrt(sum_phase_diff);
        
        weight_tmp = weight_prec + c1*M1 + c2*M2; 
       
    end


    function weight = EKF_weight(obj, weight_tmp, eta)
        weight = eta*weight_tmp;
    end

%  ____       _            _         __  __                _                   
% |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
% |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
%
% Here are defined auxiliary functions used in the public members or for other simpler computations



end % methods
end % Laserscan class