classdef DifferentialDriveRobot < handle 

    %     _   _   _        _ _           _            
    %    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
    %   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
    %  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
    % /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
    %                                                 
    properties 
        x;  % current position of the robot. Is a 3x1 vector of the form
            %   [x; y; theta]       x, y [m], theta [rad]

        x_est; % current estimated position of the robot. Is a 3x1 vector of the form [x_est [m]; y_est [m]; theta [rad]]

        d;  % the distance between the 2 wheels [m]
        R;  % radius of the wheel [m]
        KL;
        KR;
        dt;

        instance_selected;
        steps_in_range;
        weights_vec;
        
        best_tag_estimation;

        dynamics_history;
        odometry_history;
        tag_estimation_history;

        init_flag;

        odometry_estimation;


    end % properties

    %  ____        _     _ _        __  __                _                                                             
    % |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___                                           
    % | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|                                          
    % |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \                                          
    % |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/                                          
    %
    % Even if Matlab do not provide an "easy" way to discriminate public and private member functions,
    % here we firstly define the functions that are intended to be called in the main program                                                                                                                   
    methods 

        function obj = DifferentialDriveRobot(initial_state, R, d, KR, KL, dt, nM) % constructor
            obj.x = zeros(3,1);
            obj.x(1) = initial_state(1);
            obj.x(2) = initial_state(2);
            obj.x(3) = initial_state(3);

            obj.x_est(1) = initial_state(1);
            obj.x_est(2) = initial_state(2);
            obj.x_est(3) = initial_state(3);

            obj.R = R;
            obj.d = d;
            obj.KR = KR;
            obj.KL = KL;

            obj.dt = dt; % integration-scheme time step

            obj.instance_selected = 0;
            obj.steps_in_range = 0;

            obj.weights_vec = (1/nM) *ones(nM,1);

            obj.best_tag_estimation = [NaN,NaN];

            obj.dynamics_history = {};
            obj.odometry_history = {};
            obj.tag_estimation_history = {};


            obj.init_flag = false;

            obj.odometry_estimation = {[0,0], diag([0,0])};

        end

        function state = get_state(obj)
            state = obj.x;
        end

        function state_odometry = get_odometry_state(obj)
            state_odometry = obj.x_est;
        end

        function x_next = dynamics(obj,v,omega)
            [uR,uL] = vwTovv(obj,v,omega);

            obj.x(1) = obj.x(1) + ((uR + uL)/2)*cos(obj.x(3));
            obj.x(2) = obj.x(2) + ((uR + uL)/2)*sin(obj.x(3));
            obj.x(3) = obj.x(3) + ((uR - uL)/obj.d);
            

            x_next = obj.x; 
        end

        
        function odometry_estimation = odometry_step(obj,v,omega)
            [uR,uL] = vwTovv(obj,v,omega);

            uR = uR + normrnd(0,sqrt(obj.KR*abs(uR)));
            uL = uL + normrnd(0,sqrt(obj.KL*abs(uL)));

            u_est = (uR + uL)/2;
            omega_est =(uR - uL)/(obj.d);

            obj.x_est(1) = obj.x_est(1) + u_est*cos(obj.x_est(3)); 
            obj.x_est(2) = obj.x_est(2) + u_est*sin(obj.x_est(3)); 
            obj.x_est(3) = obj.x_est(3) + omega_est;
            
            Q = [obj.KR * abs(uR) , 0 ; 0, obj.KL*abs(uL)];

            obj.odometry_estimation = {[u_est,omega_est],Q}; 
            odometry_estimation = obj.odometry_estimation;

    
        end

        function inRange = inTagRange(obj,tag_position, max_range)
            dist = obj.getTagDistance(tag_position);

            if dist <= max_range
                inRange = true;
            else 
                inRange = false;
            end
        end

        function phase_measured = phaseMeasured(obj, tag_position, lambda , sigma_phi)
            distance = obj.getTagDistance(tag_position);
        
            phase = (distance * 4 * pi)/lambda;

            phase_measured = mod(-phase + normrnd(0,sigma_phi) , 2*pi) ;

        end

    %  ____       _            _         __  __                _                   
    % |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
    % | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
    % |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
    % |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
    %
    % Here are defined auxiliary functions used in the public members or for other simpler computations
        function [uR,uL] = vwTovv(obj,v, omega) % switch from v, omega to vR,vL

            uR = (v + obj.d*omega/2)*obj.dt;  
            uL = (v - omega*obj.d/2)*obj.dt;
        end

        %function [v,w] = vvTovw(obj,vR,vL)    %switch from vR.vL to v, omega
        %    v = obj.R*(vR+vL)/2;
        %    w = obj.R*(vR-vL)/2;
        %end

        function distance = getTagDistance(obj,tag_position)
            x_tag =  tag_position(1);
            y_tag = tag_position(2);
            distance = sqrt((obj.x(1) - x_tag)^2 + (obj.x(2) - y_tag)^2);
            
        end

        function distance = distanceFromPoint(obj, point)
            distance = sqrt((obj.x_est(1) - point(1))^2 + (obj.x_est(2) - point(2))^2);
        end


    end % methods

end % DifferentialDriveRobot class

