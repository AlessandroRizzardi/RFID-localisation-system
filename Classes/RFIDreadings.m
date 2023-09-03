classdef RFIDreadings < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 

    tags_vector; % vector of tags (objects of the class Tag)
    R;           % covariance matrix of the uncertainty acting on the offset-affected RFID ranges
    num_tags;    % number of tags
                 
end % properties

%  ____        _     _ _        __  __                _                                                             
% |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___                                           
% | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|                                          
% |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \                                          
% |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/                                          
%                                                                                                               
methods 

    
    % function that initializes the object
    function obj = RFIDreadings(tags_vector, num_tags)
        
        obj.tags_vector = tags_vector;
        
        I_M = eye(num_tags);
        obj.R = 2*0.1^2*I_M;

        obj.num_tags = num_tags;
        
    end % function initialize
    

    % function that add a new tag to the vector of tags
    function add_tag(obj, tag)
        
        obj.tags_vector = [obj.tags_vector; tag];
        obj.num_tags = obj.num_tags + 1;
        
    end % function add_tag

    % function that deletes last tag from the vector of tags
    function delete_last_tag(obj)
        
        obj.tags_vector = obj.tags_vector(1:end-2);
        obj.num_tags = obj.num_tags - 1;
        
    end % function delete_last_tag
    



%  ____       _            _         __  __                _                   
% |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
% |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
%
% Here are defined auxiliary functions used in the public members or for other simpler computations



end % methods
end % Laserscan class