for k = 1:steps

    phases = [];

    for i=1:length(robots)
        if robots(i).inTagRange(tag_position, max_range) == true 
            robots(i).phaseMeasured(tag_position, lambda , sigma_phi);
            phases = [phases; robots(i).phase_measured];
            tag_flag_vector(i) = true;
        else
            tag_flag_vector(i) = false;
        end
    end

    for i=1:length(robots)
        
        if robots(i).init_flag == false && robots(i).inTagRange(tag_position,max_range) == true
            
            %fprintf('Tag found by robot %d \n', i);
           
            init_alghoritm
        
        elseif robots(i).init_flag && robots(i).inTagRange(tag_position, max_range) == true

            EKF_alghoritm

            robots(i).steps_in_range = robots(i).steps_in_range + 1;

        elseif robots(i).inTagRange(tag_position, max_range) == false   % check if the robot is out of range of the tag
        
            robots(i).steps_in_range = 0;
    
            outOfRange_alghoritm  
        end

        x_next = robots(i).dynamics(v,omega);
        robots(i).dynamics_history{k,1} = x_next;

        robots(i).odometry_estimation = robots(i).odometry_step(v,omega);
        robots(i).odometry_history{k,1} = robots(i).x_est;

        robots(i).covariance_update();
    
    end
    
    % Consensus algorithm has to run only if there is at least one value true in the tag_flag_vector
    if CONSENSUS == true
        if sum(tag_flag_vector) > 0
            consensus_algorithm
        end
    end

    % check which robot has more steps in range
    [max_steps, index] = max([robots.steps_in_range]);
    % communicate to the robots the new update position to go to, 
    % that is the best estimation of the robot with the most steps in range
    if max_steps == 250
        tag_found_position = [robots(index).best_tag_estimation(1,1), robots(index).best_tag_estimation(2,1)];
    end

    if mod(k,500) == 0    
        fprintf('--------- Step numero: %d ---------\n',k);
    end
end