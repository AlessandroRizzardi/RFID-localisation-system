
%phase_history(k,1) = 0;

if tag_found_flag == false
   if robots(i).distanceFromPoint(targets(i,:)) <= 0.8 
    targets(i,:) = generateRandomPointInCircle([0,0],radius_map);
   end

   target_point = targets(i,:);
   [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);

elseif tag_found_flag == true

    if robots(i).distanceFromPoint(tag_found_position) <= 0.8
        
        tag_found_position = generateRandomPointInCircle([border_rangetag_position(1), border_rangetag_position(2)],1);
        
    end

    targets(i,:) = tag_found_position;

    target_point = targets(i,:);
    [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);  

end
    
robots(i).tag_estimation_history{k,1} = [NaN;NaN];

robots(i).init_flag = false;



%TODO: keep track of steps in range for every robot and if a robot was in the range for more than TOT steps
%      then the target point is the best tag estimation he has and not the tag_found_position as it is now