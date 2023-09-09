classdef RobotReader < handle 


%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties
    x;          % current estimated position of the robot. Is a 3x1 vector of the form
                %       [x; y; theta]       x, y [m], theta [rad]
    P;          % 6x6 covariance matrix of the state space of the system
end % properties

%  ____        _     _ _        __  __                _                                                             
% |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___                                           
% | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|                                          
% |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \                                          
% |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/                                          
%                                                                                                           
methods 

    function obj = RobotReader() % constructor
        obj.x = zeros(3, 1);
        obj.P = zeros(3, 3);
    end
    

    % Given a odometry measurement, update the robot position estimate and it's covariance
    % v and omega are the input variables of the robot

    % We compute the jacobian of the dynamic model function w.r.t. the robot state X_curr = [x_curr, y_curr, theta_curr]
    % and the jacobian of the dynamic model function w.r.t. the perturbation 
    function jac_x = JF_x(obj,odometries, inputs) 

        v = inputs(1);
        omega = inputs(2);

        x_curr = obj.x(1);
        y_curr = obj.x(2);
        theta_curr = obj.x(3);

        jac_x    = [1,  0, - sin(theta_curr) * (odx) - cos(theta_curr) * (ody), 0, 0, 0; ...
                    0,  1,   cos(theta_curr) * (odx) - sin(theta_curr) * (ody), 0, 0, 0; ...
                    0,  0,                          1,                          0, 0, 0; ...
                    1,  0,                          0,                          0, 0, 0; ...
                    0,  1,                          0,                          0, 0, 0; ...
                    0,  0,                          1,                          0, 0, 0  ];
    end

    function jac_n = JF_n(obj) 

        theta  = obj.x(3);
        jac_n    = [cos(theta), -sin(theta), 0; ...
                    sin(theta),  cos(theta), 0; ...
                    0,           0,          1; ...
                    0,           0,          0; ...
                    0,           0,          0; ...
                    0,           0,          0  ];
    end
   
    % Given a odometry measurement, update the robot position estimate 
    % Assuming that the RFID readings are measured with period Ts and that the command variable v and ω 
    % are approximately constant throughout the sampling period, it is possible to find the following discrete 
    % time Zero-order-Hold equivalent dynamics
    function next_state = update_state_robot(obj, inputs)
        v = inputs(1);
        omega = inputs(2);

        x_curr = obj.x(1);
        y_curr = obj.x(2);
        theta_curr = obj.x(3);
        
        if omega == 0
            x_next = x_curr + v * Ts* cos(theta_curr);
            y_next = y_curr + v * Ts* sin(theta_curr);
        else
            x_next = x_curr + (2 * (v/omega) * sin(omega * Ts / 2) * cos(theta_curr + omega * Ts / 2) );
            y_next = y_curr + (2 * (v/omega) * sin(omega * Ts / 2) * sin(theta_curr + omega * Ts / 2) );
        end
    
        theta_next = theta_curr + omega * Ts;

        obj.x       = [x_next; y_next; theta_next];
        next_state  = obj.x; 
         
    end

    % In order to solve phase-ambuguity, we incorporate the two consecutive robot positions
    % in the filter state. The filter state is represented by the vector:
    % X = [x; y; theta; x_old; y_old; theta_old]
    % Therefore, the measurement function is given by:
    % h(x) = [h1; h2;...;hM] = [d1,k - d1,k-1; d2,k - d2,k-1;...;dM,k - dM,k-1]
    % where d1,k is the distance between the robot and the 1st tag at time k
    % In the following function, we compute the measurement function h(x) and its Jacobian H(x)
    function [h,H] = measurement_function(obj, tag_vector, num_tags)
        
        h = zeros(num_tags, 1);
        H = zeros(num_tags, 6);
        X = [obj.x(1); obj.x(2); obj.x(3); obj.x(4); obj.x(5); obj.x(6)];

        for i = 1:num_tags
            tag = tag_vector(i);
            d_act = tag.compute_distance([X(1); X(2)]);
            d_old = tag.compute_distance([X(4); X(5)]);

            h(i) = d_act - d_old;

            H(i,1) = - (tag.x(1) - X(1)) / d_act;
            H(i,2) = - (tag.x(2) - X(2)) / d_act;
            H(i,4) =  (tag.x(1) - X(4)) / d_old;
            H(i,5) =  (tag.x(2) - X(5)) / d_old;

        end
        
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