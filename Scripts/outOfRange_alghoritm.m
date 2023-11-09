
%phase_history(k,1) = 0;

if tag_found_flag == false
   if robots(i).distanceFromPoint(targets(i,:)) <= 0.3 
    targets(i,:) = generateRandomPointInCircle([0,0],radius_map);
   end

   target_point = targets(i,:);
   [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);

elseif tag_found_flag == true

    if isnan(robots(i).best_tag_estimation(1))
        targets(i,:) = tag_found_position;
    else
        targets(i,:) = robots(i).best_tag_estimation;
    end

    target_point = targets(i,:);
    [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);  

end

robots(i).tag_estimation_history = [robots(i).tag_estimation_history; NaN, NaN];




