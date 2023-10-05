classdef Swarm < handle 

    %     _   _   _        _ _           _            
    %    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
    %   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
    %  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
    % /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
    %                                                 
    properties 
        Nrobots;
        swarm_position;
        robots_distance;
                     
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
    
        function obj = Swarm(Nrobots,swarm_initial_position, robots_distance) % constructor
            obj.Nrobots = Nrobots;
            obj.swarm_position = swarm_initial_position;
            obj.robots_distance = robots_distance;



        end

        function target_points = compute_targets(obj,swarm_target)

            target_orientation = atan2(swarm_target(2) - obj.swarm_position(2), swarm_target(1) - obj.swarm_position(1));
            position_in_square = obj.robots_square_position();

            
            x1 = swarm_target(1) + (-sin(pi/4)*sin(target_orientation) + cos(pi/4)*cos(target_orientation))*position_in_square{1,1}(1) + (-sin(pi/4)*cos(target_orientation) - sin(target_orientation)*cos(pi/4))*position_in_square{1,1}(2);
            y1 = swarm_target(2) + ( sin(pi/4)*cos(target_orientation) + sin(target_orientation)*cos(pi/4))*position_in_square{1,1}(2) + (-sin(pi/4)*sin(target_orientation) + cos(pi/4)*cos(target_orientation))*position_in_square{1,1}(2);

            x2 = swarm_target(1) + (-sin(3*pi/4)*sin(target_orientation) + cos(3*pi/4)*cos(target_orientation))*position_in_square{2,1}(1) + (-sin(3*pi/4)*cos(target_orientation) - sin(target_orientation)*cos(3*pi/4))*position_in_square{2,1}(2);
            y2 = swarm_target(2) + ( sin(3*pi/4)*cos(target_orientation) + sin(target_orientation)*cos(3*pi/4))*position_in_square{2,1}(2) + (-sin(3*pi/4)*sin(target_orientation) + cos(3*pi/4)*cos(target_orientation))*position_in_square{2,1}(2);

            x3 = swarm_target(1) + (-sin(5*pi/4)*sin(target_orientation) + cos(5*pi/4)*cos(target_orientation))*position_in_square{3,1}(1) + (-sin(5*pi/4)*cos(target_orientation) - sin(target_orientation)*cos(5*pi/4))*position_in_square{3,1}(2);
            y3 = swarm_target(2) + ( sin(5*pi/4)*cos(target_orientation) + sin(target_orientation)*cos(5*pi/4))*position_in_square{3,1}(2) + (-sin(5*pi/4)*sin(target_orientation) + cos(5*pi/4)*cos(target_orientation))*position_in_square{3,1}(2);

            x4 = swarm_target(1) + (-sin(7*pi/4)*sin(target_orientation) + cos(7*pi/4)*cos(target_orientation))*position_in_square{4,1}(1) + (-sin(7*pi/4)*cos(target_orientation) - sin(target_orientation)*cos(7*pi/4))*position_in_square{4,1}(2);
            y4 = swarm_target(2) + ( sin(7*pi/4)*cos(target_orientation) + sin(target_orientation)*cos(7*pi/4))*position_in_square{4,1}(2) + (-sin(7*pi/4)*sin(target_orientation) + cos(7*pi/4)*cos(target_orientation))*position_in_square{4,1}(2);

            
            
            target_points{1,1} = [x1,y1];
            target_points{2,1} = [x2,y2];
            target_points{3,1} = [x3,y3];
            target_points{4,1} = [x4,y4];

        end

    %  ____       _            _         __  __                _                   
    % |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
    % | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
    % |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
    % |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
    %
    % Here are defined auxiliary functions used in the public members or for other simpler computations
    
        function circle_radius = compute_swarm_radius(obj)
            circle_radius = sqrt(obj.robots_distance^2/2);
        end

        function positions = robots_square_position(obj)
            r = obj.compute_swarm_radius();

            x1 = r*cos(pi/4);
            y1 = r*sin(pi/4);

            x2 = r*cos(3*pi/4);
            y2 = r*sin(3*pi/4);

            x3 = r*cos(5*pi/4);
            y3 = r*sin(5*pi/4);

            x4 = r*cos(5*pi/4);
            y4 = r*sin(5*pi/4);

            positions{1,1} = [x1,y1];
            positions{2,1} = [x2,y2];
            positions{3,1} = [x3,y3];
            positions{4,1} = [x4,y4];
        end
    end % methods


end % Swarm class